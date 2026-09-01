#include <cstring>
#include <movutl/asset/image.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/abi.h>
#include <movutl/plugin/output.hpp>
#include <movutl/plugin/plugin.hpp>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

namespace mu::detail {

struct VideoExportHandle {
  AVFormatContext* fmt_ctx = nullptr;
  AVStream* stream         = nullptr;
  AVCodecContext* enc_ctx  = nullptr;
  SwsContext* sws          = nullptr;
  AVFrame* frame           = nullptr;
  AVPacket* pkt            = nullptr;
  int width                = 0;
  int height               = 0;
  int64_t next_pts         = 0;

  // ---------- audio(音声トラックが無い/音声非対応拡張子の場合はaudio_stream==nullptrのまま) ----------
  AVStream* audio_stream        = nullptr;
  AVCodecContext* audio_enc_ctx = nullptr;
  SwrContext* audio_swr         = nullptr;
  AVFrame* audio_frame          = nullptr; // encoder->frame_size分ずつエンコードするための作業バッファ
  AVPacket* audio_pkt           = nullptr;
  std::vector<int16_t> audio_accum; // fn_write_audioで貯める、frame_size未満の端数
  int audio_channels     = 0;
  int64_t audio_next_pts = 0;
};

static bool fn_init(cutil::PropInfo* props, cutil::Prop* defaults) {
  props->fields.push_back(cutil::PropInfo::Field("bitrate_kbps", 0, cutil::prop_info_of<int32_t>()));
  props->fields.back().set_label("ビットレート(kbps)");
  props->fields.back().min_value = 100.0f;
  props->fields.back().max_value = 100000.0f;
  defaults->set<int32_t>("bitrate_kbps", 8000);
  return true;
}

static bool fn_exit() { return true; }

// エンコード済みパケットを吐き出し切るまでreceive_packetを回す(映像/音声どちらのエンコーダにも使う汎用版)
static void drain_packets(VideoExportHandle* h, AVCodecContext* enc_ctx, AVStream* stream, AVPacket* pkt) {
  while(avcodec_receive_packet(enc_ctx, pkt) == 0) {
    av_packet_rescale_ts(pkt, enc_ctx->time_base, stream->time_base);
    pkt->stream_index = stream->index;
    av_interleaved_write_frame(h->fmt_ctx, pkt);
    av_packet_unref(pkt);
  }
}

static void close_and_free(VideoExportHandle* h) {
  if(h->sws) sws_freeContext(h->sws);
  if(h->frame) av_frame_free(&h->frame);
  if(h->pkt) av_packet_free(&h->pkt);
  if(h->enc_ctx) avcodec_free_context(&h->enc_ctx);
  if(h->audio_swr) swr_free(&h->audio_swr);
  if(h->audio_frame) av_frame_free(&h->audio_frame);
  if(h->audio_pkt) av_packet_free(&h->audio_pkt);
  if(h->audio_enc_ctx) avcodec_free_context(&h->audio_enc_ctx);
  if(h->fmt_ctx) {
    if(h->fmt_ctx->pb && !(h->fmt_ctx->oformat->flags & AVFMT_NOFILE)) avio_closep(&h->fmt_ctx->pb);
    avformat_free_context(h->fmt_ctx);
  }
}

// AACエンコーダのストリームを追加する(失敗しても音声無しで動画出力自体は続けられるようbool戻り値のみで通知)
static bool setup_audio_stream(VideoExportHandle* h, int sample_rate, int channels) {
  const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
  if(codec == nullptr) return false;

  h->audio_stream  = avformat_new_stream(h->fmt_ctx, nullptr);
  h->audio_enc_ctx = avcodec_alloc_context3(codec);
  if(h->audio_stream == nullptr || h->audio_enc_ctx == nullptr) return false;

  h->audio_enc_ctx->sample_rate = sample_rate;
  h->audio_enc_ctx->sample_fmt  = codec->sample_fmts ? codec->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
  h->audio_enc_ctx->bit_rate    = 192000;
  h->audio_enc_ctx->time_base   = AVRational{1, sample_rate};
  av_channel_layout_default(&h->audio_enc_ctx->ch_layout, channels);
  if(h->fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER) h->audio_enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  if(avcodec_open2(h->audio_enc_ctx, codec, nullptr) < 0) return false;
  avcodec_parameters_from_context(h->audio_stream->codecpar, h->audio_enc_ctx);
  h->audio_stream->time_base = h->audio_enc_ctx->time_base;

  if(swr_alloc_set_opts2(&h->audio_swr, &h->audio_enc_ctx->ch_layout, h->audio_enc_ctx->sample_fmt, sample_rate, &h->audio_enc_ctx->ch_layout, AV_SAMPLE_FMT_S16, sample_rate, 0, nullptr) < 0) return false;
  if(h->audio_swr == nullptr || swr_init(h->audio_swr) < 0) return false;

  h->audio_frame = av_frame_alloc();
  h->audio_pkt   = av_packet_alloc();
  if(h->audio_frame == nullptr || h->audio_pkt == nullptr) return false;
  h->audio_frame->format      = h->audio_enc_ctx->sample_fmt;
  h->audio_frame->sample_rate = sample_rate;
  av_channel_layout_copy(&h->audio_frame->ch_layout, &h->audio_enc_ctx->ch_layout);
  h->audio_frame->nb_samples = h->audio_enc_ctx->frame_size > 0 ? h->audio_enc_ctx->frame_size : 1024;
  if(av_frame_get_buffer(h->audio_frame, 0) < 0) return false;

  h->audio_channels = channels;
  return true;
}

// audio_accumからencoder->frame_size分ずつ切り出してエンコードする(端数は次回へ持ち越す)
static void encode_accumulated_audio(VideoExportHandle* h, bool flush) {
  if(h->audio_enc_ctx == nullptr) return;
  const int frame_size = h->audio_frame->nb_samples;
  size_t pos           = 0;
  while(true) {
    size_t avail = h->audio_accum.size() / h->audio_channels - pos;
    if((int)avail < frame_size && !(flush && avail > 0)) break;
    int n = flush ? (int)avail : frame_size;

    av_frame_make_writable(h->audio_frame);
    const uint8_t* in_planes[1] = {(const uint8_t*)&h->audio_accum[pos * h->audio_channels]};
    swr_convert(h->audio_swr, h->audio_frame->data, n, in_planes, n);
    h->audio_frame->nb_samples = n;
    h->audio_frame->pts        = h->audio_next_pts;
    h->audio_next_pts += n;

    if(avcodec_send_frame(h->audio_enc_ctx, h->audio_frame) == 0) drain_packets(h, h->audio_enc_ctx, h->audio_stream, h->audio_pkt);
    pos += n;
    if(flush) break;
  }
  h->audio_accum.erase(h->audio_accum.begin(), h->audio_accum.begin() + (ptrdiff_t)(pos * h->audio_channels));
}

static void* fn_open(const char* path, int width, int height, float framerate, int audio_sample_rate, int audio_channels, const cutil::Prop& props) {
  if(path == nullptr || width <= 0 || height <= 0) return nullptr;
  auto h = new(std::nothrow) VideoExportHandle();
  if(h == nullptr) return nullptr;
  h->width  = width;
  h->height = height;

  // format_nameをnullptrにしてpathの拡張子(mp4/mov/mkv/avi等)からffmpegに自動でmuxerを選ばせる
  if(avformat_alloc_output_context2(&h->fmt_ctx, nullptr, nullptr, path) < 0 || h->fmt_ctx == nullptr) {
    LOG_F(ERROR, "Failed to allocate output context (unsupported extension?): %s", path);
    delete h;
    return nullptr;
  }

  const AVCodec* codec = avcodec_find_encoder_by_name("libx264");
  if(codec == nullptr) codec = avcodec_find_encoder(AV_CODEC_ID_MPEG4);
  if(codec == nullptr) {
    LOG_F(ERROR, "No usable H.264/MPEG4 encoder found");
    close_and_free(h);
    delete h;
    return nullptr;
  }

  h->stream  = avformat_new_stream(h->fmt_ctx, nullptr);
  h->enc_ctx = avcodec_alloc_context3(codec);
  if(h->stream == nullptr || h->enc_ctx == nullptr) {
    close_and_free(h);
    delete h;
    return nullptr;
  }

  const int fps_num     = framerate > 0 ? (int)(framerate * 1000 + 0.5f) : 30000;
  const int fps_den     = 1000;
  h->enc_ctx->width     = width;
  h->enc_ctx->height    = height;
  h->enc_ctx->time_base = AVRational{fps_den, fps_num};
  h->enc_ctx->framerate = AVRational{fps_num, fps_den};
  h->enc_ctx->pix_fmt   = codec->pix_fmts ? codec->pix_fmts[0] : AV_PIX_FMT_YUV420P;
  h->enc_ctx->bit_rate  = (int64_t)cutil::get_or<int32_t>(props, "bitrate_kbps", 8000) * 1000;
  h->enc_ctx->gop_size  = 12;
  if(h->fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER) h->enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  if(avcodec_open2(h->enc_ctx, codec, nullptr) < 0) {
    LOG_F(ERROR, "Failed to open video encoder: %s", path);
    close_and_free(h);
    delete h;
    return nullptr;
  }
  avcodec_parameters_from_context(h->stream->codecpar, h->enc_ctx);
  h->stream->time_base = h->enc_ctx->time_base;

  if(audio_sample_rate > 0 && audio_channels > 0) {
    if(!setup_audio_stream(h, audio_sample_rate, audio_channels)) LOG_F(WARNING, "Failed to setup AAC audio stream, exporting video-only: %s", path);
  }

  if(!(h->fmt_ctx->oformat->flags & AVFMT_NOFILE) && avio_open(&h->fmt_ctx->pb, path, AVIO_FLAG_WRITE) < 0) {
    LOG_F(ERROR, "Failed to open output file: %s", path);
    close_and_free(h);
    delete h;
    return nullptr;
  }
  if(avformat_write_header(h->fmt_ctx, nullptr) < 0) {
    LOG_F(ERROR, "Failed to write header: %s", path);
    close_and_free(h);
    delete h;
    return nullptr;
  }

  h->frame = av_frame_alloc();
  h->pkt   = av_packet_alloc();
  if(h->frame == nullptr || h->pkt == nullptr) {
    close_and_free(h);
    delete h;
    return nullptr;
  }
  h->frame->format = h->enc_ctx->pix_fmt;
  h->frame->width  = width;
  h->frame->height = height;
  if(av_frame_get_buffer(h->frame, 32) < 0) {
    close_and_free(h);
    delete h;
    return nullptr;
  }

  return h;
}

static bool fn_write_frame(void* handle, const Image* img, int) {
  if(handle == nullptr || img == nullptr) return false;
  auto h = (VideoExportHandle*)handle;

  h->sws = sws_getCachedContext(h->sws, h->width, h->height, AV_PIX_FMT_BGRA, h->width, h->height, h->enc_ctx->pix_fmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if(h->sws == nullptr) return false;

  const uint8_t* src_data[1] = {(const uint8_t*)&(*img)[0]};
  const int src_linesize[1]  = {h->width * 4};
  sws_scale(h->sws, src_data, src_linesize, 0, h->height, h->frame->data, h->frame->linesize);
  h->frame->pts = h->next_pts++;

  if(avcodec_send_frame(h->enc_ctx, h->frame) < 0) return false;
  drain_packets(h, h->enc_ctx, h->stream, h->pkt);
  return true;
}

static bool fn_write_audio(void* handle, const int16_t* pcm, int n_samples) {
  if(handle == nullptr || pcm == nullptr || n_samples <= 0) return false;
  auto h = (VideoExportHandle*)handle;
  if(h->audio_enc_ctx == nullptr) return false; // 音声ストリーム未セットアップ(fn_open時にaudio_sample_rate=0だった等)

  h->audio_accum.insert(h->audio_accum.end(), pcm, pcm + (size_t)n_samples * h->audio_channels);
  encode_accumulated_audio(h, false);
  return true;
}

static bool fn_close(void* handle) {
  if(handle == nullptr) return false;
  auto h = (VideoExportHandle*)handle;
  avcodec_send_frame(h->enc_ctx, nullptr); // ドレインモードへ移行
  drain_packets(h, h->enc_ctx, h->stream, h->pkt);
  if(h->audio_enc_ctx != nullptr) {
    encode_accumulated_audio(h, true); // 端数を吐き出す
    avcodec_send_frame(h->audio_enc_ctx, nullptr);
    drain_packets(h, h->audio_enc_ctx, h->audio_stream, h->audio_pkt);
  }
  av_write_trailer(h->fmt_ctx);
  close_and_free(h);
  delete h;
  return true;
}

OutputPluginTable out_export_video = {
  0x00000102, // guid
  "Video (FFmpeg)",
  "FFmpeg(libx264)を用いた動画書き出しプラグイン(mp4/mov/mkv/avi)",
  false, // is_sequence
  {"mp4", "mov", "mkv", "avi", "", "", "", "", "", ""},
  fn_init,
  fn_exit,
  fn_open,
  fn_write_frame,
  fn_write_audio,
  fn_close,
};

} // namespace mu::detail

namespace {

void plugin_init(mu::ABIContext* abi) { abi->register_output_plugin(&mu::detail::out_export_video); }
void plugin_exit(mu::ABIContext*) {}

} // namespace

extern "C" void plugin_entry(mu::ABIContext*, mu::PluginTable* table) {
  std::memset(table, 0, sizeof(*table));
  std::strncpy(table->name, "Video Exporter", sizeof(table->name) - 1);
  std::strncpy(table->description, "FFmpegを用いた動画書き出しプラグイン(mp4/mov/mkv/avi)", sizeof(table->description) - 1);
  table->plugin_init = &plugin_init;
  table->plugin_exit = &plugin_exit;
}
