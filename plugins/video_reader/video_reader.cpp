#include <cstring>
#include <movutl/asset/movie.hpp>
#include <movutl/core/logger.hpp>
#include <movutl/plugin/input.hpp>
#include <mutex>

#ifdef MOVUTL_HAS_FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}
#include <vector>

namespace mu::detail {

// 出力ピクセルフォーマットは BGRA32。
// mu::Image::data_ は Vec4b(B,G,R,A) 順のメモリレイアウトを前提としているため
// (従来の OpenCV 版プラグインの cv::Mat(CV_8UC4, BGR) 直接コピーと互換)。
constexpr AVPixelFormat kOutputPixFmt = AV_PIX_FMT_BGRA;

/// 動画ファイル1つ分のデコードコンテキスト。
/// ハンドル毎に完全に独立しており、複数の動画を同時にオープン・読み込み可能。
struct FFmpegVideoHandle {
  std::mutex mtx;
  AVFormatContext* fmt_ctx = nullptr;
  AVCodecContext* dec_ctx  = nullptr;
  AVFrame* frame           = nullptr; // デコーダからの出力
  AVFrame* bgra            = nullptr; // BGRA 変換バッファ (直近デコード結果のキャッシュ)
  AVFrame* pending         = nullptr; // 受け取ったが未消費の未来フレーム
  AVPacket* pkt            = nullptr;
  SwsContext* sws          = nullptr;
  int stream_index         = -1;
  int width                = 0;
  int height               = 0;
  AVRational fps           = {30, 1};
  int64_t nb_frames        = 0;
  int64_t start_time       = 0;
  int decoded_frame        = -1; ///< bgra に変換済みのフレーム番号 (-1 = 未デコード)
  bool eof                 = false;
  bool has_pending         = false;

  // ---------- audio: fn_open時に全体をPCM16へデコードして保持(ponytail: 長時間音声はメモリ消費大、必要ならvideoと同様のシーク+逐次デコードへ切替) ----------
  int audio_stream_index    = -1;
  int audio_native_rate     = 0;
  int audio_native_channels = 0;
  int64_t audio_total_samples = 0; // 1chあたりのサンプル数
  std::vector<int16_t> audio_pcm;  // interleaved PCM16, audio_total_samples * audio_native_channels 個

  ~FFmpegVideoHandle() { release(); }

  void release() {
    std::lock_guard<std::mutex> lock(mtx);
    if(sws) sws_freeContext(sws);
    if(bgra) av_frame_free(&bgra);
    if(pending) av_frame_free(&pending);
    if(frame) av_frame_free(&frame);
    if(pkt) av_packet_free(&pkt);
    if(dec_ctx) avcodec_free_context(&dec_ctx);
    if(fmt_ctx) avformat_close_input(&fmt_ctx);
    sws           = nullptr;
    dec_ctx       = nullptr;
    fmt_ctx       = nullptr;
    decoded_frame = -1;
    eof           = false;
    has_pending   = false;
  }
};

/// 音声ストリームをh->audio_pcmへ全デコードする。無ければ何もしない(動画のみのファイルもあるためエラー扱いしない)
static void decode_audio_track(FFmpegVideoHandle* h) {
  int idx = av_find_best_stream(h->fmt_ctx, AVMEDIA_TYPE_AUDIO, -1, -1, nullptr, 0);
  if(idx < 0) return;
  auto st  = h->fmt_ctx->streams[idx];
  auto dec = avcodec_find_decoder(st->codecpar->codec_id);
  if(!dec) return;

  AVCodecContext* actx = avcodec_alloc_context3(dec);
  if(!actx || avcodec_parameters_to_context(actx, st->codecpar) < 0 || avcodec_open2(actx, dec, nullptr) < 0) {
    if(actx) avcodec_free_context(&actx);
    return;
  }

  AVChannelLayout out_layout;
  av_channel_layout_copy(&out_layout, &actx->ch_layout);
  SwrContext* swr = nullptr;
  if(swr_alloc_set_opts2(&swr, &out_layout, AV_SAMPLE_FMT_S16, actx->sample_rate, &actx->ch_layout, actx->sample_fmt, actx->sample_rate, 0, nullptr) < 0 || !swr || swr_init(swr) < 0) {
    if(swr) swr_free(&swr);
    av_channel_layout_uninit(&out_layout);
    avcodec_free_context(&actx);
    return;
  }

  AVPacket* pkt   = av_packet_alloc();
  AVFrame* frame  = av_frame_alloc();
  int channels    = out_layout.nb_channels;
  h->audio_native_rate     = actx->sample_rate;
  h->audio_native_channels = channels;
  h->audio_stream_index    = idx;

  auto drain_frame = [&](AVFrame* f) {
    int64_t max_out = swr_get_out_samples(swr, f ? f->nb_samples : 0);
    if(max_out <= 0) max_out = 4096;
    std::vector<int16_t> buf((size_t)max_out * channels);
    uint8_t* out_planes[1] = {(uint8_t*)buf.data()};
    int n = swr_convert(swr, out_planes, (int)max_out, f ? (const uint8_t**)f->data : nullptr, f ? f->nb_samples : 0);
    if(n > 0) h->audio_pcm.insert(h->audio_pcm.end(), buf.data(), buf.data() + (size_t)n * channels);
  };

  while(av_read_frame(h->fmt_ctx, pkt) >= 0) {
    if(pkt->stream_index == idx) {
      if(avcodec_send_packet(actx, pkt) == 0) {
        while(avcodec_receive_frame(actx, frame) == 0) drain_frame(frame);
      }
    }
    av_packet_unref(pkt);
  }
  avcodec_send_packet(actx, nullptr);
  while(avcodec_receive_frame(actx, frame) == 0) drain_frame(frame);
  drain_frame(nullptr); // swr内部に残ったサンプルを吐き出す

  h->audio_total_samples = channels > 0 ? (int64_t)h->audio_pcm.size() / channels : 0;

  av_frame_free(&frame);
  av_packet_free(&pkt);
  swr_free(&swr);
  av_channel_layout_uninit(&out_layout);
  avcodec_free_context(&actx);
}

static bool fn_init() { return true; }
static bool fn_exit() { return true; }

static InputHandle fn_open(const char* file) {
  if(file == nullptr) return nullptr;
  auto h = new(std::nothrow) FFmpegVideoHandle();
  if(!h) return nullptr;

  if(avformat_open_input(&h->fmt_ctx, file, nullptr, nullptr) != 0) {
    LOG_F(ERROR, "Failed to open movie file: %s", file);
    delete h;
    return nullptr;
  }
  if(avformat_find_stream_info(h->fmt_ctx, nullptr) < 0) {
    LOG_F(ERROR, "Failed to find stream info: %s", file);
    delete h;
    return nullptr;
  }

  h->stream_index = av_find_best_stream(h->fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if(h->stream_index < 0) {
    // 映像ストリームが無い場合は音声のみのファイル(mp3/wav等)として扱う
    decode_audio_track(h);
    if(h->audio_total_samples <= 0) {
      LOG_F(ERROR, "No video or audio stream found: %s", file);
      delete h;
      return nullptr;
    }
    return h;
  }
  auto st  = h->fmt_ctx->streams[h->stream_index];
  auto dec = avcodec_find_decoder(st->codecpar->codec_id);
  if(dec == nullptr) {
    LOG_F(ERROR, "Unsupported codec: %s", avcodec_get_name(st->codecpar->codec_id));
    delete h;
    return nullptr;
  }

  h->dec_ctx = avcodec_alloc_context3(dec);
  if(h->dec_ctx == nullptr || avcodec_parameters_to_context(h->dec_ctx, st->codecpar) < 0) {
    LOG_F(ERROR, "Failed to setup decoder: %s", file);
    delete h;
    return nullptr;
  }
  /// AVCodecParametersにtime_baseが含まれないため明示的に設定(無いとAVFrame::ptsが常にAV_NOPTS_VALUEになる)
  h->dec_ctx->pkt_timebase = st->time_base;
  h->dec_ctx->time_base    = st->time_base;
  if(avcodec_open2(h->dec_ctx, dec, nullptr) < 0) {
    LOG_F(ERROR, "Failed to setup decoder: %s", file);
    delete h;
    return nullptr;
  }

  h->width      = h->dec_ctx->width;
  h->height     = h->dec_ctx->height;
  h->fps        = st->avg_frame_rate.num > 0 && st->avg_frame_rate.den > 0 ? st->avg_frame_rate : st->r_frame_rate;
  h->nb_frames  = st->nb_frames;
  h->start_time = st->start_time > 0 ? st->start_time : 0;
  if(h->nb_frames <= 0 && h->fmt_ctx->duration > 0 && h->fps.num > 0) h->nb_frames = (int64_t)((double)h->fmt_ctx->duration / AV_TIME_BASE * av_q2d(h->fps));

  h->frame   = av_frame_alloc();
  h->bgra    = av_frame_alloc();
  h->pending = av_frame_alloc();
  h->pkt     = av_packet_alloc();
  if(h->frame == nullptr || h->bgra == nullptr || h->pending == nullptr || h->pkt == nullptr || h->width <= 0 || h->height <= 0) {
    LOG_F(ERROR, "Failed to allocate frame buffers: %s", file);
    delete h;
    return nullptr;
  }
  h->bgra->format = kOutputPixFmt;
  h->bgra->width  = h->width;
  h->bgra->height = h->height;
  if(av_frame_get_buffer(h->bgra, 32) < 0) {
    LOG_F(ERROR, "Failed to allocate BGRA buffer: %s", file);
    delete h;
    return nullptr;
  }

  decode_audio_track(h);

  return h;
}

static bool fn_close(InputHandle ih) {
  if(ih == nullptr) return false;
  delete(FFmpegVideoHandle*)ih;
  return true;
}

static bool fn_info_get(InputHandle ih, EntityInfo* iip) {
  if(ih == nullptr || iip == nullptr) return false;
  auto h = (FFmpegVideoHandle*)ih;
  std::lock_guard<std::mutex> lock(h->mtx);
  if(h->dec_ctx == nullptr && h->audio_total_samples <= 0) return false;

  iip->flag = (EntityType)((h->dec_ctx ? (int)EntityType_Movie : 0) | (h->audio_total_samples > 0 ? (int)EntityType_Audio : 0));
  iip->framerate        = h->dec_ctx ? av_q2d(h->fps) : 0.0f;
  iip->nframes          = (uint32_t)(h->nb_frames > 0 ? h->nb_frames : 0);
  iip->width             = h->width;
  iip->height            = h->height;
  iip->format            = ImageFormatRGBA; /// メモリレイアウトはBGRA8 (Image::data_ と同一)
  iip->handler           = 0;
  iip->audio_n           = (int32_t)h->audio_total_samples;
  iip->audio_sample_rate = h->audio_native_rate;
  iip->audio_channels    = h->audio_native_channels;
  return true;
}

/// デコーダからフレームを1枚受け取ろうとする
/// 戻り値: 1 = フレーム取得成功 / 0 = 追加入力が必要(再試行) / -1 = EOFまたはエラー
static int decode_next(FFmpegVideoHandle* h) {
  int ret = avcodec_receive_frame(h->dec_ctx, h->frame);
  if(ret == 0) return 1;
  if(ret == AVERROR_EOF) {
    h->eof = true;
    return -1;
  }
  if(ret != AVERROR(EAGAIN)) return -1;

  /// デコーダへの入力が不足しているのでパケットを供給する
  while(true) {
    ret = av_read_frame(h->fmt_ctx, h->pkt);
    if(ret < 0) { /// ストリーム終端
      h->eof = true;
      avcodec_send_packet(h->dec_ctx, nullptr); /// ドレインモードへ移行
      return 0;
    }
    if(h->pkt->stream_index != h->stream_index) {
      av_packet_unref(h->pkt);
      continue;
    }
    av_packet_rescale_ts(h->pkt, h->fmt_ctx->streams[h->stream_index]->time_base, h->dec_ctx->time_base);
    ret = avcodec_send_packet(h->dec_ctx, h->pkt);
    av_packet_unref(h->pkt);
    if(ret < 0 && ret != AVERROR(EAGAIN)) return -1;
    return 0;
  }
}

static void convert_to_bgra(FFmpegVideoHandle* h, AVFrame* src) {
  h->sws = sws_getCachedContext(h->sws, h->width, h->height, (AVPixelFormat)src->format, h->width, h->height, kOutputPixFmt, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if(h->sws == nullptr) return;
  sws_scale(h->sws, src->data, src->linesize, 0, h->height, h->bgra->data, h->bgra->linesize);
}

static void copy_out(FFmpegVideoHandle* h, void* buf) {
  const size_t row_bytes = (size_t)h->width * 4;
  for(int y = 0; y < h->height; y++) memcpy((uint8_t*)buf + y * row_bytes, h->bgra->data[0] + y * h->bgra->linesize[0], row_bytes);
}

static int fn_read_video(InputHandle ih, int frame_no, void* buf) {
  if(ih == nullptr || buf == nullptr || frame_no < 0) return 0;
  auto h = (FFmpegVideoHandle*)ih;
  std::lock_guard<std::mutex> lock(h->mtx);
  if(h->dec_ctx == nullptr) return 0;

  /// キャッシュヒット (同じフレームの再読み込み)
  if(frame_no == h->decoded_frame) {
    copy_out(h, buf);
    return h->width * h->height * 4;
  }

  /// 連続フレームでなければシークしてデコーダを巻き戻す
  if(frame_no != h->decoded_frame + 1) {
    const auto tb        = h->fmt_ctx->streams[h->stream_index]->time_base;
    const int64_t target = h->start_time + av_rescale(frame_no * (int64_t)h->fps.den, tb.den, (int64_t)h->fps.num * tb.num);
    if(av_seek_frame(h->fmt_ctx, h->stream_index, target, AVSEEK_FLAG_BACKWARD) < 0) av_seek_frame(h->fmt_ctx, -1, 0, AVSEEK_FLAG_BACKWARD); /// フォールバック: 先頭へ
    avcodec_flush_buffers(h->dec_ctx);
    h->eof = false;
    if(h->pending) av_frame_unref(h->pending);
    h->has_pending = false;
  }

  const double target_sec = (double)frame_no * h->fps.den / (double)h->fps.num;
  constexpr double EPS    = 0.000001;
  bool got                = false;

  /// 未消費の先行フレームがあれば先に処理する
  if(h->has_pending) {
    h->has_pending = false;
    const double t = h->pending->pts * av_q2d(h->dec_ctx->time_base);
    if(t <= target_sec + EPS) {
      convert_to_bgra(h, h->pending);
      got = true;
    } else {
      h->has_pending = true; /// まだ未来のフレームなので保持し続ける
    }
  }

  while(!h->eof && !h->has_pending) {
    const int r = decode_next(h);
    if(r == 1) {
      const double t = h->frame->pts * av_q2d(h->dec_ctx->time_base);
      if(t <= target_sec + EPS) {
        convert_to_bgra(h, h->frame);
        got = true;
      } else {
        /// 目標を超えるフレーム: 消費せず保持して終了 (連続読み込み時に備える)
        av_frame_unref(h->pending);
        av_frame_ref(h->pending, h->frame);
        h->has_pending = true;
        break;
      }
    } else if(r < 0) {
      break;
    }
  }

  if(!got) return 0;
  h->decoded_frame = frame_no;
  copy_out(h, buf);
  return h->width * h->height * 4;
}

static int fn_read_audio(InputHandle ih, int start, int length, void* buf) {
  if(ih == nullptr || buf == nullptr || length <= 0) return 0;
  auto h = (FFmpegVideoHandle*)ih;
  std::lock_guard<std::mutex> lock(h->mtx);
  int ch = h->audio_native_channels;
  if(ch <= 0 || h->audio_total_samples <= 0) return 0;

  auto* out = (int16_t*)buf;
  int copied = 0;
  for(int i = 0; i < length; i++) {
    int64_t s = (int64_t)start + i;
    if(s < 0 || s >= h->audio_total_samples) {
      for(int c = 0; c < ch; c++) out[(size_t)i * ch + c] = 0;
    } else {
      for(int c = 0; c < ch; c++) out[(size_t)i * ch + c] = h->audio_pcm[(size_t)s * ch + c];
      copied++;
    }
  }
  return copied > 0 ? length : 0;
}

} // namespace mu::detail

#else

/// FFmpeg 無効ビルド用スタブ
namespace mu::detail {
static bool stub_init() {
  LOG_F(WARNING, "FFmpeg support is disabled (built without MOVUTL_HAS_FFMPEG)");
  return true;
}
} // namespace mu::detail

#endif

namespace mu::detail {

mu::InputPluginTable plg_video_reader = {
  0x00000001,                                                                   // guid
  InputPluginFlag_Video | InputPluginFlag_Audio | InputPluginFlag_Concurrent, // flag
  (EntityType)(EntityType_Movie | EntityType_Audio),                           // supports
  "FFmpeg Video Reader",                              // name
  "",                                                 // filepath
  "Read video/audio files via FFmpeg",                // information
  {"avi", "mp4", "mov", "mkv", "webm", "mpg", "mpeg", "m4v", "wav", "mp3", "flac", "aac", "m4a", "ogg", "", ""},
#ifdef MOVUTL_HAS_FFMPEG
  fn_init,       //	DLL開始時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  fn_exit,       //	DLL終了時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
  fn_open,       //	入力ファイルをオープンする関数へのポインタ
  fn_close,      //	入力ファイルをクローズする関数へのポインタ
  fn_info_get,   //	入力ファイルの情報を取得する関数へのポインタ
  fn_read_video, //	画像データを読み込む関数へのポインタ (フレーム指定+呼び出し側バッファ)
  fn_read_audio, //	音声データを読み込む関数へのポインタ
  nullptr,       //	キーフレームか調べる関数へのポインタ (NULLなら全てキーフレーム)
  nullptr,       //	入力設定のダイアログを要求された時に呼ばれる関数へのポインタ (NULLなら呼ばれません)
#else
  stub_init,
  nullptr, // fn_exit
  nullptr, // fn_open
  nullptr, // fn_close
  nullptr, // fn_info_get
  nullptr, // fn_read_video
  nullptr, // fn_read_audio
  nullptr, // fn_is_keyframe
  nullptr, // fn_config_wnd
#endif
};

} // namespace mu::detail
