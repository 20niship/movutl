#include <cstring>
#include <movutl/asset/image.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/abi.h>
#include <movutl/plugin/output.hpp>
#include <movutl/plugin/plugin.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
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

// エンコード済みパケットを吐き出し切るまでreceive_packetを回す(fn_write_frame/fn_closeの両方から使う)
static void drain_packets(VideoExportHandle* h) {
  while(avcodec_receive_packet(h->enc_ctx, h->pkt) == 0) {
    av_packet_rescale_ts(h->pkt, h->enc_ctx->time_base, h->stream->time_base);
    h->pkt->stream_index = h->stream->index;
    av_interleaved_write_frame(h->fmt_ctx, h->pkt);
    av_packet_unref(h->pkt);
  }
}

static void close_and_free(VideoExportHandle* h) {
  if(h->sws) sws_freeContext(h->sws);
  if(h->frame) av_frame_free(&h->frame);
  if(h->pkt) av_packet_free(&h->pkt);
  if(h->enc_ctx) avcodec_free_context(&h->enc_ctx);
  if(h->fmt_ctx) {
    if(h->fmt_ctx->pb && !(h->fmt_ctx->oformat->flags & AVFMT_NOFILE)) avio_closep(&h->fmt_ctx->pb);
    avformat_free_context(h->fmt_ctx);
  }
}

static void* fn_open(const char* path, int width, int height, float framerate, const cutil::Prop& props) {
  if(path == nullptr || width <= 0 || height <= 0) return nullptr;
  auto h = new(std::nothrow) VideoExportHandle();
  if(h == nullptr) return nullptr;
  h->width  = width;
  h->height = height;

  if(avformat_alloc_output_context2(&h->fmt_ctx, nullptr, "mp4", path) < 0 || h->fmt_ctx == nullptr) {
    LOG_F(ERROR, "Failed to allocate output context: %s", path);
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

  h->frame         = av_frame_alloc();
  h->pkt           = av_packet_alloc();
  h->frame->format = h->enc_ctx->pix_fmt;
  h->frame->width  = width;
  h->frame->height = height;
  if(h->frame == nullptr || h->pkt == nullptr || av_frame_get_buffer(h->frame, 32) < 0) {
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
  drain_packets(h);
  return true;
}

static bool fn_close(void* handle) {
  if(handle == nullptr) return false;
  auto h = (VideoExportHandle*)handle;
  avcodec_send_frame(h->enc_ctx, nullptr); // ドレインモードへ移行
  drain_packets(h);
  av_write_trailer(h->fmt_ctx);
  close_and_free(h);
  delete h;
  return true;
}

OutputPluginTable out_export_video = {
  0x00000102, // guid
  "MP4 Video (FFmpeg)",
  "FFmpeg(libx264)を用いたmp4動画書き出しプラグイン",
  false, // is_sequence
  {"mp4", "", "", "", "", "", "", "", "", ""},
  fn_init,
  fn_exit,
  fn_open,
  fn_write_frame,
  fn_close,
};

} // namespace mu::detail

namespace {

void plugin_init(mu::ABIContext* abi) { abi->register_output_plugin(&mu::detail::out_export_video); }
void plugin_exit(mu::ABIContext*) {}

} // namespace

extern "C" void plugin_entry(mu::ABIContext*, mu::PluginTable* table) {
  std::memset(table, 0, sizeof(*table));
  std::strncpy(table->name, "MP4 Exporter", sizeof(table->name) - 1);
  std::strncpy(table->description, "FFmpegを用いたmp4動画書き出しプラグイン", sizeof(table->description) - 1);
  table->plugin_init = &plugin_init;
  table->plugin_exit = &plugin_exit;
}
