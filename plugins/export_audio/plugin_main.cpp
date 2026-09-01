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
}

namespace mu::detail {

struct AudioExportHandle {
  AVFormatContext* fmt_ctx = nullptr;
  AVStream* stream         = nullptr;
  AVCodecContext* enc_ctx  = nullptr;
  SwrContext* swr          = nullptr;
  AVFrame* frame           = nullptr; // encoder->frame_size分ずつエンコードするための作業バッファ
  AVPacket* pkt            = nullptr;
  std::vector<int16_t> accum; // fn_write_audioで貯める、frame_size未満の端数
  int channels     = 0;
  int64_t next_pts = 0;
};

static bool fn_init(cutil::PropInfo* props, cutil::Prop* defaults) {
  props->fields.push_back(cutil::PropInfo::Field("bitrate_kbps", 0, cutil::prop_info_of<int32_t>()));
  props->fields.back().set_label("ビットレート(kbps, mp3のみ)");
  props->fields.back().min_value = 32.0f;
  props->fields.back().max_value = 320.0f;
  defaults->set<int32_t>("bitrate_kbps", 192);
  return true;
}

static bool fn_exit() { return true; }

static void drain_packets(AudioExportHandle* h) {
  while(avcodec_receive_packet(h->enc_ctx, h->pkt) == 0) {
    av_packet_rescale_ts(h->pkt, h->enc_ctx->time_base, h->stream->time_base);
    h->pkt->stream_index = h->stream->index;
    av_interleaved_write_frame(h->fmt_ctx, h->pkt);
    av_packet_unref(h->pkt);
  }
}

static void close_and_free(AudioExportHandle* h) {
  if(h->swr) swr_free(&h->swr);
  if(h->frame) av_frame_free(&h->frame);
  if(h->pkt) av_packet_free(&h->pkt);
  if(h->enc_ctx) avcodec_free_context(&h->enc_ctx);
  if(h->fmt_ctx) {
    if(h->fmt_ctx->pb && !(h->fmt_ctx->oformat->flags & AVFMT_NOFILE)) avio_closep(&h->fmt_ctx->pb);
    avformat_free_context(h->fmt_ctx);
  }
}

// 拡張子(fmt_ctx->oformatから判別)に応じてPCM(wav)かlibmp3lame(mp3)のエンコーダを選ぶ
static const AVCodec* pick_codec(AVFormatContext* fmt_ctx) {
  if(fmt_ctx->oformat->audio_codec == AV_CODEC_ID_MP3) return avcodec_find_encoder_by_name("libmp3lame");
  return avcodec_find_encoder(fmt_ctx->oformat->audio_codec != AV_CODEC_ID_NONE ? fmt_ctx->oformat->audio_codec : AV_CODEC_ID_PCM_S16LE);
}

static void* fn_open(const char* path, int, int, float, int audio_sample_rate, int audio_channels, const cutil::Prop& props) {
  if(path == nullptr || audio_sample_rate <= 0 || audio_channels <= 0) return nullptr;
  auto h = new(std::nothrow) AudioExportHandle();
  if(h == nullptr) return nullptr;

  if(avformat_alloc_output_context2(&h->fmt_ctx, nullptr, nullptr, path) < 0 || h->fmt_ctx == nullptr) {
    LOG_F(ERROR, "Failed to allocate output context (unsupported extension?): %s", path);
    delete h;
    return nullptr;
  }

  const AVCodec* codec = pick_codec(h->fmt_ctx);
  if(codec == nullptr) {
    LOG_F(ERROR, "No usable audio encoder found for: %s", path);
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

  h->enc_ctx->sample_rate = audio_sample_rate;
  h->enc_ctx->sample_fmt  = codec->sample_fmts ? codec->sample_fmts[0] : AV_SAMPLE_FMT_S16;
  h->enc_ctx->bit_rate    = (int64_t)cutil::get_or<int32_t>(props, "bitrate_kbps", 192) * 1000;
  h->enc_ctx->time_base   = AVRational{1, audio_sample_rate};
  av_channel_layout_default(&h->enc_ctx->ch_layout, audio_channels);
  if(h->fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER) h->enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  if(avcodec_open2(h->enc_ctx, codec, nullptr) < 0) {
    LOG_F(ERROR, "Failed to open audio encoder: %s", path);
    close_and_free(h);
    delete h;
    return nullptr;
  }
  avcodec_parameters_from_context(h->stream->codecpar, h->enc_ctx);
  h->stream->time_base = h->enc_ctx->time_base;

  if(swr_alloc_set_opts2(&h->swr, &h->enc_ctx->ch_layout, h->enc_ctx->sample_fmt, audio_sample_rate, &h->enc_ctx->ch_layout, AV_SAMPLE_FMT_S16, audio_sample_rate, 0, nullptr) < 0 || h->swr == nullptr || swr_init(h->swr) < 0) {
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
  h->frame->format      = h->enc_ctx->sample_fmt;
  h->frame->sample_rate = audio_sample_rate;
  av_channel_layout_copy(&h->frame->ch_layout, &h->enc_ctx->ch_layout);
  // frame_size==0(PCM等、固定フレームサイズを持たないコーデック)は1024サンプル単位で適当に区切って送る
  h->frame->nb_samples = h->enc_ctx->frame_size > 0 ? h->enc_ctx->frame_size : 1024;
  if(av_frame_get_buffer(h->frame, 0) < 0) {
    close_and_free(h);
    delete h;
    return nullptr;
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

  h->channels = audio_channels;
  return h;
}

// PNG連番/動画同様このプラグインは音声専用のため映像フレームは無視する(呼び出し側は全プラグイン共通ループでfn_write_frameを呼ぶ)
static bool fn_write_frame(void*, const Image*, int) { return true; }

static bool fn_write_audio(void* handle, const int16_t* pcm, int n_samples) {
  if(handle == nullptr || pcm == nullptr || n_samples <= 0) return false;
  auto h = (AudioExportHandle*)handle;

  h->accum.insert(h->accum.end(), pcm, pcm + (size_t)n_samples * h->channels);
  const int frame_size = h->frame->nb_samples;
  size_t pos           = 0;
  while(h->accum.size() / h->channels - pos >= (size_t)frame_size) {
    av_frame_make_writable(h->frame);
    const uint8_t* in_planes[1] = {(const uint8_t*)&h->accum[pos * h->channels]};
    swr_convert(h->swr, h->frame->data, frame_size, in_planes, frame_size);
    h->frame->pts = h->next_pts;
    h->next_pts += frame_size;
    if(avcodec_send_frame(h->enc_ctx, h->frame) == 0) drain_packets(h);
    pos += frame_size;
  }
  h->accum.erase(h->accum.begin(), h->accum.begin() + (ptrdiff_t)(pos * h->channels));
  return true;
}

static bool fn_close(void* handle) {
  if(handle == nullptr) return false;
  auto h = (AudioExportHandle*)handle;

  if(!h->accum.empty()) { // 端数を最終フレームとして書き出す
    int n = (int)(h->accum.size() / h->channels);
    av_frame_make_writable(h->frame);
    const uint8_t* in_planes[1] = {(const uint8_t*)h->accum.data()};
    swr_convert(h->swr, h->frame->data, n, in_planes, n);
    h->frame->nb_samples = n;
    h->frame->pts        = h->next_pts;
    if(avcodec_send_frame(h->enc_ctx, h->frame) == 0) drain_packets(h);
  }
  avcodec_send_frame(h->enc_ctx, nullptr); // ドレインモードへ移行
  drain_packets(h);
  av_write_trailer(h->fmt_ctx);
  close_and_free(h);
  delete h;
  return true;
}

OutputPluginTable out_export_audio = {
  0x00000103, // guid
  "Audio (FFmpeg)",
  "FFmpeg(PCM/libmp3lame)を用いた音声書き出しプラグイン(wav/mp3)",
  false, // is_sequence
  {"wav", "mp3", "", "", "", "", "", "", "", ""},
  fn_init,
  fn_exit,
  fn_open,
  fn_write_frame,
  fn_write_audio,
  fn_close,
};

} // namespace mu::detail

namespace {

void plugin_init(mu::ABIContext* abi) { abi->register_output_plugin(&mu::detail::out_export_audio); }
void plugin_exit(mu::ABIContext*) {}

} // namespace

extern "C" void plugin_entry(mu::ABIContext*, mu::PluginTable* table) {
  std::memset(table, 0, sizeof(*table));
  std::strncpy(table->name, "Audio Exporter", sizeof(table->name) - 1);
  std::strncpy(table->description, "FFmpegを用いた音声書き出しプラグイン(wav/mp3)", sizeof(table->description) - 1);
  table->plugin_init = &plugin_init;
  table->plugin_exit = &plugin_exit;
}
