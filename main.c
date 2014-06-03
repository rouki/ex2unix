#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/avstring.h>
#include <libavutil/mathematics.h>
#include <libavformat/avio.h>

#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>

#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#ifdef __MINGW32__
#undef main /* Prevents SDL from overriding main() */
#endif

#include <stdio.h>
#include <math.h>

#include "ex2.h"

extern int size;
extern URLProtocol e2URLProtocol;

/*
 * merges the resized image (res_image - global variable) and the given destination image
 *
 * returns a YUV merged image.
 *
 * pre-conditions: res_image points to a valid image, otherwise behavior is undefined
 */
AVFrame*
merge_frames (AVFrame* dest_image, VideoState* is1)
{
    AVFrame* pFrameRGB1, *pFrameRGB2;
    pFrameRGB1 = YUV_TO_RGB(dest_image, is1->video_st[is1->videoStream].codec);
    pFrameRGB2 = res_image->small_image;

	if (pFrameRGB1 == NULL || pFrameRGB2 == NULL) {
		return NULL;
	}
    for (int y = 0; y < res_image->height; ++y) {
        uint8_t* s = (pFrameRGB1->data[0] + (y + 42) * pFrameRGB1->linesize[0]);
	    uint8_t* s2 = (pFrameRGB2->data[0] + y * pFrameRGB2->linesize[0]);
        for (int x = 0 ; x < res_image->width * 3 ; x += 3) {
            *(s + x + 42 ) = *(s2 + x );
            *(s + x + 43 ) = *(s2 + x + 1);
		    *(s + x + 44 ) = *(s2 + x + 2);
        }
    }
    return RGB_TO_YUV(pFrameRGB1, is1->video_st[is1->videoStream].codec);
}

/*
 * resizes the given image according to another image, let the 'merging thread' know when it's done.
 *
 * pCodecCtx_image1 - codec context of the image that should be resized
 * pCodecCtx_image2 - codec context of the image it should resize according to
 *
 */
void
resize_image(AVFrame* image1, AVCodecContext* pCodecCtx_image1, AVCodecContext* pCodecCtx_image2)
{
    SDL_LockMutex(res_image->mutex);
    res_image->small_image = avcodec_alloc_frame();
    res_image->width = pCodecCtx_image1->width;
    for (res_image->height = pCodecCtx_image1->height ; res_image->height > pCodecCtx_image2->height / DECR_BY ; res_image->height /= DECR_BY) {
    	res_image->width /= DECR_BY;
    }
    res_image->width /= DECR_BY;
    struct SwsContext* resize = sws_getContext(pCodecCtx_image1->width, pCodecCtx_image1->height,
						 PIX_FMT_YUV420P, res_image->width, res_image->height,
						 PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);

    int num_bytes = avpicture_get_size(PIX_FMT_RGB24, res_image->width, res_image->height);
    uint8_t* frame2_buffer = (uint8_t *)av_malloc(num_bytes * sizeof(uint8_t));
    avpicture_fill((AVPicture*)res_image->small_image, frame2_buffer, PIX_FMT_RGB24, res_image->width, res_image->height);
    sws_scale(resize, image1->data, image1->linesize, 0, pCodecCtx_image1->height, res_image->small_image->data, res_image->small_image->linesize);
    sws_freeContext(resize);
    res_image->flag = 1;
    SDL_CondSignal(res_image->cond);
    SDL_UnlockMutex(res_image->mutex);
}

/*
 * converts the given image to RGB
 *
 * pre-conditions: The given image is formatted as YUV
 */
AVFrame* YUV_TO_RGB(AVFrame* origin, AVCodecContext* pCodecCtx) {
    AVFrame* pFrameRGB;
    uint8_t* buffer;
    pFrameRGB = avcodec_alloc_frame();
	/* wait until we have space for a new pic */
    int numBytes = avpicture_get_size(PIX_FMT_RGB24, pCodecCtx->width,
                                      pCodecCtx->height);
    buffer = (uint8_t *) av_malloc(numBytes * sizeof(uint8_t));

    avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);

    struct SwsContext* sw = sws_getContext(pCodecCtx->width,
                                           pCodecCtx->height, pCodecCtx->pix_fmt,
                                           pCodecCtx->width, pCodecCtx->height,
                                           PIX_FMT_RGB24, SWS_FAST_BILINEAR, NULL, NULL, NULL);

    sws_scale(sw,(const uint8_t * const *) origin->data, origin->linesize, 0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);
    sws_freeContext(sw);

    return pFrameRGB;
}

/*
 * converts the given image to YUV
 *
 *
 * pre-conditions: The given image is formatted as RGB
 */
AVFrame* RGB_TO_YUV(AVFrame* origin, AVCodecContext* pCodecCtx) {
    AVFrame* pYUVFrame;
    uint8_t* buffer;
    pYUVFrame = avcodec_alloc_frame();
    int numBytes = avpicture_get_size(PIX_FMT_YUV420P, pCodecCtx->width, pCodecCtx->height);
    buffer = (uint8_t*) av_malloc(numBytes * sizeof(uint8_t));
    avpicture_fill((AVPicture*)pYUVFrame, buffer, PIX_FMT_YUV420P, pCodecCtx->width, pCodecCtx->height);

    struct SwsContext* sw = sws_getContext(pCodecCtx->width, pCodecCtx->height, PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height, PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);

    sws_scale(sw,(const uint8_t * const *) origin->data, origin->linesize, 0, pCodecCtx->height,pYUVFrame->data, pYUVFrame->linesize);

    return pYUVFrame;
}

void
turn_off_pp()
{
    SDL_DestroyMutex(res_image->mutex);
    SDL_DestroyCond(res_image->cond);
    res_image->flag = 0;
    pp_on = 0;
}

void
turn_on_pp()
{
    res_image->mutex = SDL_CreateMutex();
    res_image->cond = SDL_CreateCond();
    res_image->flag = 0;
    pp_on = 1;
}

void packet_queue_init(PacketQueue *q) {
	memset(q, 0, sizeof(PacketQueue));
	q->mutex = SDL_CreateMutex();
	q->cond = SDL_CreateCond();
}
int packet_queue_put(PacketQueue *q, AVPacket *pkt) {

	AVPacketList *pkt1;
	if (pkt != &flush_pkt && av_dup_packet(pkt) < 0) {
		return -1;
	}
	pkt1 = av_malloc(sizeof(AVPacketList));
	if (!pkt1)
		return -1;
	pkt1->pkt = *pkt;
	pkt1->next = NULL;

	SDL_LockMutex(q->mutex);

	if (!q->last_pkt)
		q->first_pkt = pkt1;
	else
		q->last_pkt->next = pkt1;
	q->last_pkt = pkt1;
	q->nb_packets++;
	q->size += pkt1->pkt.size;
	SDL_CondSignal(q->cond);

	SDL_UnlockMutex(q->mutex);
	return 0;
}
static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block) {
	AVPacketList *pkt1;
	int ret;

	SDL_LockMutex(q->mutex);

	for (;;) {

		if (global_video_state->quit) {
			ret = -1;
			break;
		}

		pkt1 = q->first_pkt;
		if (pkt1) {
			q->first_pkt = pkt1->next;
			if (!q->first_pkt)
				q->last_pkt = NULL;
			q->nb_packets--;
			q->size -= pkt1->pkt.size;
			*pkt = pkt1->pkt;
			av_free(pkt1);
			ret = 1;
			break;
		} else if (!block) {
			ret = 0;
			break;
		} else {
			SDL_CondWait(q->cond, q->mutex);
		}
	}
	SDL_UnlockMutex(q->mutex);
	return ret;
}

static void packet_queue_flush(PacketQueue *q) {
	AVPacketList *pkt, *pkt1;

	SDL_LockMutex(q->mutex);
	for (pkt = q->first_pkt; pkt != NULL; pkt = pkt1) {
		pkt1 = pkt->next;
		av_free_packet(&pkt->pkt);
		av_freep(&pkt);
	}
	q->last_pkt = NULL;
	q->first_pkt = NULL;
	q->nb_packets = 0;
	q->size = 0;
	SDL_UnlockMutex(q->mutex);
}

double get_audio_clock(VideoState *is) {
	double pts;
	int hw_buf_size, bytes_per_sec, n;

	pts = is->audio_clock; /* maintained in the audio thread */
	hw_buf_size = is->audio_buf_size - is->audio_buf_index;
	bytes_per_sec = 0;
	n = is->audio_st->codec->channels * 2;
	if (is->audio_st) {
		bytes_per_sec = is->audio_st->codec->sample_rate * n;
	}
	if (bytes_per_sec) {
		pts -= (double) hw_buf_size / bytes_per_sec;
	}
	return pts;
}

double get_video_clock(VideoState *is) {
	double delta;

	delta = (av_gettime() - is->video_current_pts_time) / 1000000.0;
	return is->video_current_pts + delta;
}
double get_external_clock(VideoState *is) {
	return av_gettime() / 1000000.0;
}

double get_master_clock(VideoState *is) {
	if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
		return get_video_clock(is);
	} else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
		return get_audio_clock(is);
	} else {
		return get_external_clock(is);
	}
}
/* Add or subtract samples to get a better sync, return new
 audio buffer size */
int synchronize_audio(VideoState *is, short *samples, int samples_size,
                      double pts) {
	int n;
	double ref_clock;

	n = 2 * is->audio_st->codec->channels;

	if (is->av_sync_type != AV_SYNC_AUDIO_MASTER) {
		double diff, avg_diff;
		int wanted_size, min_size, max_size;
		//int nb_samples;

		ref_clock = get_master_clock(is);
		diff = get_audio_clock(is) - ref_clock;

		if (diff < AV_NOSYNC_THRESHOLD) {
			// accumulate the diffs
			is->audio_diff_cum = diff
            + is->audio_diff_avg_coef * is->audio_diff_cum;
			if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {
				is->audio_diff_avg_count++;
			} else {
				avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);
				if (fabs(avg_diff) >= is->audio_diff_threshold) {
					wanted_size = samples_size
                    + ((int) (diff * is->audio_st->codec->sample_rate)
                       * n);
					min_size = samples_size
                    * ((100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100);
					max_size = samples_size
                    * ((100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100);
					if (wanted_size < min_size) {
						wanted_size = min_size;
					} else if (wanted_size > max_size) {
						wanted_size = max_size;
					}
					if (wanted_size < samples_size) {
						/* remove samples */
						samples_size = wanted_size;
					} else if (wanted_size > samples_size) {
						uint8_t *samples_end, *q;
						int nb;

						/* add samples by copying final sample*/
						nb = (samples_size - wanted_size);
						samples_end = (uint8_t *) samples + samples_size - n;
						q = samples_end + n;
						while (nb > 0) {
							memcpy(q, samples_end, n);
							q += n;
							nb -= n;
						}
						samples_size = wanted_size;
					}
				}
			}
		} else {
			/* difference is TOO big; reset diff stuff */
			is->audio_diff_avg_count = 0;
			is->audio_diff_cum = 0;
		}
	}
	return samples_size;
}

int audio_decode_frame(VideoState *is, double *pts_ptr) {

	int len1, data_size = 0, n;
	AVPacket *pkt = &is->audio_pkt;
	double pts;

	for (;;) {
		while (is->audio_pkt_size > 0) {
			int got_frame = 0;
			len1 = avcodec_decode_audio4(is->audio_st->codec, &is->audio_frame,
                                         &got_frame, pkt);
			if (len1 < 0) {
				/* if error, skip frame */
				is->audio_pkt_size = 0;
				break;
			}

			if (got_frame) {
				data_size = is->audio_frame.linesize[0];
				/*av_samples_get_buffer_size(NULL,
				 is->audio_st->codec->channels,
				 is->audio_frame.nb_samples,
				 is->audio_st->codec->sample_fmt, 1);*/
				memcpy(is->audio_buf, is->audio_frame.data[0], data_size);
			}

			is->audio_pkt_data += len1;
			is->audio_pkt_size -= len1;
			if (data_size <= 0) {
				/* No data yet, get more frames */
				continue;
			}

			pts = is->audio_clock;
			*pts_ptr = pts;
			n = 2 * is->audio_st->codec->channels;
			is->audio_clock += (double) data_size
            / (double) (n * is->audio_st->codec->sample_rate);

			/* We have data, return it and come back for more later */
			return data_size;
		}
		if (pkt->data)
			av_free_packet(pkt);

		if (is->quit) {
			return -1;
		}
		/* next packet */
		if (packet_queue_get(&is->audioq, pkt, 1) < 0) {
			return -1;
		}

		if (pkt->data == flush_pkt.data) {
			avcodec_flush_buffers(is->audio_st->codec);
			continue;
		}

		is->audio_pkt_data = pkt->data;
		is->audio_pkt_size = pkt->size;

		/* if update, update the audio clock w/pts */
		if (pkt->pts != AV_NOPTS_VALUE) {
			is->audio_clock = (av_q2d(is->audio_st->time_base) * pkt->pts);
		}
	}

	return 0;
}

void audio_callback(void* userdata, Uint8 *stream, int len) {

    VideoState* is;
	if (curr_audio_on == 1) {
        is = global_video_state;
	} else if (curr_audio_on == 2) {
        is = global_video_state2;
    } else {
        is = global_video_state3;
    }
	int len1, audio_size;
	double pts;

	while (len > 0) {
		if (is->audio_buf_index >= is->audio_buf_size) {
			/* We have already sent all our data; get more */
			audio_size = audio_decode_frame(is, &pts);
			if (audio_size < 0) {
				/* If error, output silence */
				is->audio_buf_size = 1024;
				memset(is->audio_buf, 0, is->audio_buf_size);
			} else {
				audio_size = synchronize_audio(is, (int16_t *) is->audio_buf,
                                               audio_size, pts);
				is->audio_buf_size = audio_size;
			}
			is->audio_buf_index = 0;
		}
		len1 = is->audio_buf_size - is->audio_buf_index;
		if (len1 > len)
            len1 = len;
		memcpy(stream, (uint8_t *) is->audio_buf + is->audio_buf_index, len1);
		len -= len1;
		stream += len1;
		is->audio_buf_index += len1;
	}
}

static Uint32 sdl_refresh_timer_cb(Uint32 interval, void *opaque) {
	SDL_Event event;
	event.type = FF_REFRESH_EVENT;
	event.user.data1 = opaque;
	SDL_PushEvent(&event);
	return 0; /* 0 means stop timer */
}

/* schedule a video refresh in 'delay' ms */
static void schedule_refresh(VideoState *is, int delay) {
	SDL_AddTimer(delay, sdl_refresh_timer_cb, is);
}

void video_display(VideoState *is) {

    if (is->id != curr_video_on)
        return;
	SDL_Rect rect;
	VideoPicture *vp;
	float aspect_ratio;
	int w, h, x, y;

	vp = &is->pictq[is->pictq_rindex];
	if (vp->bmp) {
		if (is->video_st->codec->sample_aspect_ratio.num == 0) {
			aspect_ratio = 0;
		} else {
			aspect_ratio = av_q2d(is->video_st->codec->sample_aspect_ratio)
            * is->video_st->codec->width / is->video_st->codec->height;
		}
		if (aspect_ratio <= 0.0) {
			aspect_ratio = (float) is->video_st->codec->width
            / (float) is->video_st->codec->height;
		}
		h = screen->h;
		w = ((int) rint(h * aspect_ratio)) & -3;
		if (w > screen->w) {
			w = screen->w;
			h = ((int) rint(w / aspect_ratio)) & -3;
		}
		x = (screen->w - w) / 2;
		y = (screen->h - h) / 2;

		rect.x = x;
		rect.y = y;
		rect.w = w;
		rect.h = h;
		SDL_DisplayYUVOverlay(vp->bmp, &rect);
	}
}

void video_refresh_timer(void *userdata) {

	VideoState *is = (VideoState *) userdata;
	VideoPicture *vp;
	double actual_delay, delay, sync_threshold, ref_clock, diff;

	if (is->video_st) {
		if (is->pictq_size == 0) {
			schedule_refresh(is, 1);
		} else {

			vp = &is->pictq[is->pictq_rindex];

			is->video_current_pts = vp->pts;
			is->video_current_pts_time = av_gettime();

			delay = vp->pts - is->frame_last_pts; /* the pts from last time */
			if (delay <= 0 || delay >= 1.0) {
				/* if incorrect delay, use previous one */
				delay = is->frame_last_delay;
			}
			/* save for next time */
			is->frame_last_delay = delay;
			is->frame_last_pts = vp->pts;

			/* update delay to sync to audio */
			ref_clock = get_audio_clock(is);
			diff = vp->pts - ref_clock;

			/* update delay to sync to audio if not master source */
			if (is->av_sync_type != AV_SYNC_VIDEO_MASTER) {
				ref_clock = get_master_clock(is);
				diff = vp->pts - ref_clock;

				/* Skip or repeat the frame. Take delay into account
				 FFPlay still doesn't "know if this is the best guess." */
				sync_threshold =
                (delay > AV_SYNC_THRESHOLD) ? delay : AV_SYNC_THRESHOLD;
				if (fabs(diff) < AV_NOSYNC_THRESHOLD) {
					if (diff <= -sync_threshold) {
						delay = 0;
					} else if (diff >= sync_threshold) {
						delay = 2 * delay;
					}
				}
			}
			is->frame_timer += delay;
			/* computer the REAL delay */
			actual_delay = is->frame_timer - (av_gettime() / 1000000.0);
			if (actual_delay < 0.010) {
				/* Really it should skip the picture instead */
				actual_delay = 0.010;
			}
			schedule_refresh(is, (int) (actual_delay * 1000 + 0.5));

			/* show the picture! */
			video_display(is);

			/* update queue for next picture! */
			if (++is->pictq_rindex == VIDEO_PICTURE_QUEUE_SIZE) {
				is->pictq_rindex = 0;
			}
			SDL_LockMutex(is->pictq_mutex);
			is->pictq_size--;
			SDL_CondSignal(is->pictq_cond);
			SDL_UnlockMutex(is->pictq_mutex);
		}
	} else {
		schedule_refresh(is, 100);
	}
}

void alloc_picture(void *userdata) {

	VideoState *is = (VideoState *) userdata;
	VideoPicture *vp;

	vp = &is->pictq[is->pictq_windex];
	if (vp->bmp) {
		// we already have one make another, bigger/smaller
		SDL_FreeYUVOverlay(vp->bmp);
	}
	// Allocate a place to put our YUV image on that screen
	vp->bmp = SDL_CreateYUVOverlay(is->video_st->codec->width,
                                   is->video_st->codec->height, SDL_YV12_OVERLAY, screen);
	vp->width = is->video_st->codec->width;
	vp->height = is->video_st->codec->height;

	SDL_LockMutex(is->pictq_mutex);
	vp->allocated = 1;
	SDL_CondSignal(is->pictq_cond);
	SDL_UnlockMutex(is->pictq_mutex);

}

int
number_of_movies() {
    int result = 0;
    if (global_video_state) result += 1;
    if (global_video_state2) result += 1;
    if (global_video_state3) result += 1;
    return result;
}

/*
 * handles merging between two threads (To support PP)
 *
 */
AVFrame*
handle_merge (AVFrame* origin, VideoState* is)
{

    if (pp_on && ((is->id - 1) == curr_video_on % number_of_movies())) {
        resize_image(origin, is->video_st[is->videoStream].codec,global_video_state->video_st[global_video_state->videoStream].codec);
        return NULL;
    } else if (pp_on && is->id == curr_video_on) {
        while (!res_image->flag) {
            SDL_CondWait(res_image->cond, res_image->mutex);
        }
        SDL_LockMutex(res_image->mutex);
        AVFrame* result = merge_frames(origin, is);
        av_free(res_image->small_image);
        res_image->flag = 0;
        SDL_UnlockMutex(res_image->mutex);
        return result;
    } else {
        return NULL;
    }

}

/*
 * handles color changing (According to the glob. variable 'video_color')
 *
 */
AVFrame*
handle_color (AVFrame* origin, VideoState* is)
{
    AVFrame* pFrameRGB;
    if (video_color == REGULAR) {
        return NULL;
    } else {
        int c_del1 = 0 , c_del2 = 0, bw = 0;
        pFrameRGB = YUV_TO_RGB(origin, is->video_st[is->videoStream].codec);
        switch (video_color) {
        case RED:
            c_del1 = 1; c_del2 = 2;
            break;
        case GREEN:
            c_del1 = 0; c_del2 = 2;
            break;
        case BLUE:
            c_del1 = 0 ; c_del2 = 1;
            break;
        case BW:
            bw = 1;
            break;
        default:
            // UNREACHABLE
            break;

        }
        for (int y = 0; y < is->video_st[is->videoStream].codec->height; ++y) {
            uint8_t* s = (pFrameRGB->data[0] + y * pFrameRGB->linesize[0]);
            for (int x = 0 ; x < is->video_st[is->videoStream].codec->width * 3 ; x += 3) {
                if (bw) {
                    int avg = (*(s + x) + *(s + x + 1) + *(s + x + 2)) / 3;
                    *(s + x) = *(s + x + 1) = *(s + x + 2) = avg;
                } else {
                    *(s + x + c_del1) = 0;
                    *(s + x + c_del2) = 0;
                }
            }
        }
        return RGB_TO_YUV(pFrameRGB, is->video_st[is->videoStream].codec);
    }
}

int
queue_picture(VideoState *is, AVFrame *pFrame, double pts)
{
	VideoPicture *vp;
	//int dst_pix_fmt;
	AVPicture pict;
	AVFrame* frame_copy;

	/* wait until we have space for a new pic */
	SDL_LockMutex(is->pictq_mutex);
	while (is->pictq_size >= VIDEO_PICTURE_QUEUE_SIZE && !is->quit) {
		SDL_CondWait(is->pictq_cond, is->pictq_mutex);
	}
	SDL_UnlockMutex(is->pictq_mutex);

	if (is->quit)
		return -1;

    if ((frame_copy = handle_color(pFrame,is)) != NULL) {
        pFrame = frame_copy;
    }
    if ((frame_copy = handle_merge(pFrame,is)) != NULL) {
        pFrame = frame_copy;
    }
	// windex is set to 0 initially
	vp = &is->pictq[is->pictq_windex];

	/* allocate or resize the buffer! */
	if (!vp->bmp || vp->width != is->video_st->codec->width
        || vp->height != is->video_st->codec->height) {
		SDL_Event event;

		vp->allocated = 0;
		/* we have to do it in the main thread */
		event.type = FF_ALLOC_EVENT;
		event.user.data1 = is;
		SDL_PushEvent(&event);

		/* wait until we have a picture allocated */
		SDL_LockMutex(is->pictq_mutex);
		while (!vp->allocated && !is->quit) {
			SDL_CondWait(is->pictq_cond, is->pictq_mutex);
		}
		SDL_UnlockMutex(is->pictq_mutex);
		if (is->quit) {
			return -1;
		}
	}
	/* We have a place to put our picture on the queue */
	/* If we are skipping a frame, do we set this to null
	 but still return vp->allocated = 1? */

	if (vp->bmp) {

		SDL_LockYUVOverlay(vp->bmp);

		//dst_pix_fmt = PIX_FMT_YUV420P;
		/* point pict at the queue */

		pict.data[0] = vp->bmp->pixels[0];
		pict.data[1] = vp->bmp->pixels[2];
		pict.data[2] = vp->bmp->pixels[1];

		pict.linesize[0] = vp->bmp->pitches[0];
		pict.linesize[1] = vp->bmp->pitches[2];
		pict.linesize[2] = vp->bmp->pitches[1];

		// Convert the image into YUV format that SDL uses
		sws_scale(is->sws_ctx, (const uint8_t * const *) pFrame->data,
                  pFrame->linesize, 0, is->video_st->codec->height, pict.data,
                  pict.linesize);

		SDL_UnlockYUVOverlay(vp->bmp);
		vp->pts = pts;

		/* now we inform our display thread that we have a pic ready */
		if (++is->pictq_windex == VIDEO_PICTURE_QUEUE_SIZE) {
			is->pictq_windex = 0;
		}
		SDL_LockMutex(is->pictq_mutex);
		is->pictq_size++;
		SDL_UnlockMutex(is->pictq_mutex);
	}
	return 0;
}

double
synchronize_video(VideoState *is, AVFrame *src_frame, double pts)
{

	double frame_delay;

	if (pts != 0) {
		/* if we have pts, set video clock to it */
		is->video_clock = pts;
	} else {
		/* if we aren't given a pts, set it to the clock */
		pts = is->video_clock;
	}
	/* update the video clock */
	frame_delay = av_q2d(is->video_st->codec->time_base);
	/* if we are repeating a frame, adjust clock accordingly */
	frame_delay += src_frame->repeat_pict * (frame_delay * 0.5);
	is->video_clock += frame_delay ;
	return pts;
}

/* These are called whenever we allocate a frame
 * buffer. We use this to store the global_pts in
 * a frame at the time it is allocated.
 */
int
our_get_buffer(struct AVCodecContext *c, AVFrame *pic)
{
	int ret = avcodec_default_get_buffer(c, pic);
	uint64_t *pts = av_malloc(sizeof(uint64_t));
	*pts = global_video_pkt_pts;
	pic->opaque = pts;
	return ret;
}

int
our_get_buffer2(struct AVCodecContext *c, AVFrame *pic)
{
	int ret = avcodec_default_get_buffer(c, pic);
	uint64_t *pts = av_malloc(sizeof(uint64_t));
	*pts = global_video_pkt_pts2;
	pic->opaque = pts;
	return ret;
}

int
our_get_buffer3(struct AVCodecContext *c, AVFrame *pic)
{
	int ret = avcodec_default_get_buffer(c, pic);
	uint64_t *pts = av_malloc(sizeof(uint64_t));
	*pts = global_video_pkt_pts3;
	pic->opaque = pts;
	return ret;
}

void
our_release_buffer(struct AVCodecContext *c, AVFrame *pic)
{
	if (pic)
		av_freep(&pic->opaque);
	avcodec_default_release_buffer(c, pic);
}

int
video_thread(void *arg)
{
	VideoState *is = (VideoState *) arg;
	AVPacket pkt1, *packet = &pkt1;
	//int len1;
	int frameFinished;
	AVFrame *pFrame;
	double pts;

	pFrame = avcodec_alloc_frame();
/*    if (ff_on) {
        is->video_st->time_base.den *= 2;
    }
*/
	for (;;) {
		if (packet_queue_get(&is->videoq, packet, 1) < 0) {
			// means we quit getting packets
			break;
		}

		pts = 0;

		// Save global pts to be stored in pFrame in first call
		if (is->id == 1)
			global_video_pkt_pts = packet->pts;
        else if (is->id == 2)
        	global_video_pkt_pts2 = packet->pts;
        else
            global_video_pkt_pts3 = packet->pts;
		// Decode video frame
		//len1 =
		avcodec_decode_video2(is->video_st->codec, pFrame, &frameFinished,
                              packet);

		if (packet->dts == AV_NOPTS_VALUE && pFrame->opaque
            && *(uint64_t*) pFrame->opaque != AV_NOPTS_VALUE) {
			pts = *(uint64_t *) pFrame->opaque;
		} else if (packet->dts != AV_NOPTS_VALUE) {
			pts = packet->dts;
		} else {
			pts = 0;
		}

		pts *= (av_q2d(is->video_st->time_base));

		// Did we get a video frame?
		if (frameFinished) {
                pts = synchronize_video(is, pFrame, pts);
                if (queue_picture(is, pFrame, pts) < 0) {
                    break;
                }
        }
		av_free_packet(packet);
	}
	av_free(pFrame);
	return 0;
}

int
stream_component_open(VideoState* is, int stream_index)
{

	AVFormatContext *pFormatCtx = is->pFormatCtx;
	AVCodecContext *codecCtx;
	AVCodec *codec;
	SDL_AudioSpec wanted_spec, spec;

	if (stream_index < 0 || stream_index >= pFormatCtx->nb_streams) {
		return -1;
	}

	// Get a pointer to the codec context for the video stream
	codecCtx = pFormatCtx->streams[stream_index]->codec;

	if (codecCtx->codec_type == AVMEDIA_TYPE_AUDIO) {
		// Set audio settings from codec info
		init_spec(is, stream_index);

        if (is->id == 1) {
            if (SDL_OpenAudio(&is->spec, &spec) < 0) {
                fprintf(stderr, "SDL_OpenAudio: %s\n", SDL_GetError());
                return -1;
            }
        }
		is->audio_hw_buf_size = spec.size;
	}
	SDL_LockMutex(mutex);
	codec = avcodec_find_decoder(codecCtx->codec_id);
	if (!codec || (avcodec_open2(codecCtx, codec, NULL) < 0)) {
		fprintf(stderr, "Unsupported codec!\n");
		return -1;
	}
	SDL_UnlockMutex(mutex);
	switch (codecCtx->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            is->audioStream = stream_index;
            is->audio_st = pFormatCtx->streams[stream_index];
            is->audio_buf_size = 0;
            is->audio_buf_index = 0;

            /* averaging filter for audio sync */
            is->audio_diff_avg_coef = exp(log(0.01 / AUDIO_DIFF_AVG_NB));
            is->audio_diff_avg_count = 0;
            /* Correct audio only if larger error than this */
            is->audio_diff_threshold = 2.0 * SDL_AUDIO_BUFFER_SIZE
            / codecCtx->sample_rate;

            memset(&is->audio_pkt, 0, sizeof(is->audio_pkt));
            packet_queue_init(&is->audioq);
            SDL_PauseAudio(0);
            break;
        case AVMEDIA_TYPE_VIDEO:
            is->videoStream = stream_index;
            is->video_st = pFormatCtx->streams[stream_index];

            is->sws_ctx = sws_getContext(is->video_st->codec->width,
                                         is->video_st->codec->height, is->video_st->codec->pix_fmt,
                                         is->video_st->codec->width, is->video_st->codec->height,
                                         PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);

            is->frame_timer = (double) av_gettime() / 1000000.0;
            is->frame_last_delay = 40e-3;
            is->video_current_pts_time = av_gettime();

            packet_queue_init(&is->videoq);
            is->video_tid = get_thread(is, VIDEO_THREAD);
            if (is->id == 1)
            	codecCtx->get_buffer = our_get_buffer;
            else if (is->id == 2)
            	codecCtx->get_buffer = our_get_buffer2;
            else
                codecCtx->get_buffer = our_get_buffer3;
            codecCtx->release_buffer = our_release_buffer;
            break;
        default:
            break;
	}

	return 0;
}

int
decode_interrupt_cb(void *opaque)
{
    VideoState* is = (VideoState*) opaque;
    if (is->id == 1)
        return (global_video_state && global_video_state->quit);
    else if (is->id == 2)
        return (global_video_state2 && global_video_state2->quit);
    else if (is->id == 3)
        return (global_video_state3 && global_video_state3->quit);
    else
        return -1;
}

int
decode_thread(void *arg)
{

	VideoState *is = (VideoState *) arg;
	AVFormatContext *pFormatCtx = NULL;
	AVPacket pkt1, *packet = &pkt1;

	int video_index = -1;
	int audio_index = -1;
	int i;
    #define PROT_STR "unixetwo://"
    char prot_ip[strlen(is->filename) + strlen(PROT_STR) + 1];
    sprintf(prot_ip, "%s%s", PROT_STR, is->filename);

	is->videoStream = -1;
	is->audioStream = -1;

	AVIOInterruptCB interupt_cb;

    /* initialize the global video states */
	switch(is->id) {
    case 1:
        global_video_state = is;
        break;
    case 2:
        global_video_state2 = is;
        break;
    case 3:
        global_video_state3 = is;
        break;
	}
	// will interrupt blocking functions if we quit!
	interupt_cb.callback = decode_interrupt_cb;
	interupt_cb.opaque = is;
	SDL_LockMutex(mutex);

	if (av_open_input_file(&pFormatCtx, prot_ip, NULL, 0, NULL) < 0) {
        printf("There was a problem opening the file. \n");
        return -1;
	}

	is->pFormatCtx = pFormatCtx;

	// Retrieve stream information
	if (avformat_find_stream_info(pFormatCtx, NULL) < 0)
		return -1; // Couldn't find stream information

    SDL_UnlockMutex(mutex);

	// Dump information about file onto standard error
	av_dump_format(pFormatCtx, 0, is->filename, 0);

	// Find the first video stream

	for (i = 0; i < pFormatCtx->nb_streams; i++) {
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO
            && video_index < 0) {
			video_index = i;
		}
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO
            && audio_index < 0) {
			audio_index = i;
		}
	}
    if (audio_index >= 0) {
        stream_component_open(is, audio_index);
    }
	if (video_index >= 0) {
		stream_component_open(is, video_index);
	}

	if (is->videoStream < 0) {
		fprintf(stderr, "%s: could not open codecs\n", is->filename);
		goto fail;
	}

	// main decode loop

	for (;;) {
        if(is->seek_req) {
            int stream_index= -1;
            int64_t seek_target = is->seek_pos;

            if     (is->videoStream >= 0) stream_index = is->videoStream;
            else if(is->audioStream >= 0) stream_index = is->audioStream;

            if(stream_index>=0){
                seek_target= av_rescale_q(seek_target, AV_TIME_BASE_Q, pFormatCtx->streams[stream_index]->time_base);
            }
            if(av_seek_frame(is->pFormatCtx, stream_index, seek_target, is->seek_flags) < 0) {
                fprintf(stderr, "%s: error while seeking\n", is->pFormatCtx->filename);
            } else {
                if(is->audioStream >= 0) {
                    packet_queue_flush(&is->audioq);
                    packet_queue_put(&is->audioq, &flush_pkt);
                }
                if(is->videoStream >= 0) {
                    packet_queue_flush(&is->videoq);
                    packet_queue_put(&is->videoq, &flush_pkt);
                }
            }
            is->seek_req = 0;
        }
		if (is->quit) {
			break;
		}
		// seek stuff goes here

		if (is->audioq.size > MAX_AUDIOQ_SIZE
            || is->videoq.size > MAX_VIDEOQ_SIZE) {
                if (is->audioq.size > MAX_AUDIOQ_SIZE && is->id != curr_audio_on) {
                    packet_queue_flush(&is->audioq);
                }

			SDL_Delay(10);
			continue;
		}

		if (av_read_frame(is->pFormatCtx, packet) < 0) {
			if (is->pFormatCtx->pb->error == 0) {
				SDL_Delay(100); /* no error; wait for user input */
				continue;
			} else {
				break;
			}
		}

		// Is this a packet from the video stream?
		if (packet->stream_index == is->videoStream) {
			packet_queue_put(&is->videoq, packet);
		} else if (packet->stream_index == is->audioStream) {
			packet_queue_put(&is->audioq, packet);
		} else {
			av_free_packet(packet);
		}
	}

	/* all done - wait for it */
	while (!is->quit) {
		SDL_Delay(100);
	}

fail: {
    SDL_Event event;
    event.type = FF_QUIT_EVENT;
    event.user.data1 = is;
    SDL_PushEvent(&event);
}
	return 0;
}

void
stream_seek(VideoState *is, int64_t pos, int rel)
{

	if (!is->seek_req) {
		is->seek_pos = pos;
		is->seek_flags = rel < 0 ? AVSEEK_FLAG_BACKWARD : 0;
		is->seek_req = 1;
	}
}

void
init_spec(VideoState* is, int streamIndex)
{
    AVFormatContext *pFormatCtx = is->pFormatCtx;
    AVCodecContext* codecCtx;
    codecCtx = pFormatCtx->streams[streamIndex]->codec;
    (is->spec).freq = codecCtx->sample_rate;
    (is->spec).format = AUDIO_S16SYS;
    (is->spec).channels = codecCtx->channels;
    (is->spec).silence = 0;
    (is->spec).samples = SDL_AUDIO_BUFFER_SIZE;
    (is->spec).callback = audio_callback;
}

/*
 * reopens the audio device with the given VideoState spec.
 *
 * (Used when there's a need to change between two movies with different spec)
 */
void
reopen_audio(VideoState* is)
{
    SDL_AudioSpec spec;
    SDL_CloseAudio();
    init_spec(is,is->audioStream);
    SDL_OpenAudio(&is->spec,&spec);
    SDL_Delay(10);
    SDL_PauseAudio(0);
}

/*
 *
 * returns the video position of the last played video
 *
 */
double
calculate_pos()
{

    if (last_played_video == 1) {
        return get_master_clock(global_video_state);
    } else if (last_played_video == 2) {
        return get_master_clock(global_video_state2);
    } else {
        return get_master_clock(global_video_state3);
    }
}

/*
 * Used when switching between two videos (If the switched movie's dimensions are bigger than the new one's)
 *
 */
void
turn_screen_black()
{
    SDL_Rect r;
    Uint32 clearColor;
    r.x = r.y = 0;
    r.w = screen->w;
    r.h = screen->h;
    clearColor = SDL_MapRGB(screen->format, 0, 0, 0);
    SDL_FillRect(screen, &r, clearColor);
    SDL_Flip(screen);
}

/*
 *
 * initializes the given VideoState and launches its decode thread
 *
 */
int
init_run_video(VideoState* is, const char* name, int id)
{
    av_strlcpy(is->filename, name, sizeof(is->filename));
	is->id = id;
	is->pictq_mutex = SDL_CreateMutex();
	is->pictq_cond = SDL_CreateCond();
	is->av_sync_type = DEFAULT_AV_SYNC_TYPE;
	is->parse_tid = get_thread(is, DECODE_THREAD);
	schedule_refresh(is, 40);
	return 0;
}

/*
 *
 * given a video state and thread type, launches the desired thread.
 * each thread (decode and video thread) can be launched only ONCE for each VideoState.
 */
static SDL_Thread*
get_thread (VideoState* video, THREAD_TYPE type)
{
    static SDL_Thread* dec_thread[3];
    static SDL_Thread* vid_thread[3];

    switch (video->id) {
        case 1: {
                if (type == DECODE_THREAD) {
                    if (dec_thread[0] == NULL) {
                        dec_thread[0] = SDL_CreateThread(decode_thread, video);
                    }
                    return dec_thread[0];
                }
                else if (type == VIDEO_THREAD) {
                    if (vid_thread[0] == NULL) {
                        vid_thread[0] = SDL_CreateThread(video_thread, video);
                    }
                    return vid_thread[0];
                }
            }
            break;
        case 2: {
                if (type == DECODE_THREAD) {
                    if (dec_thread[1] == NULL) {
                        dec_thread[1] = SDL_CreateThread(decode_thread, video);
                    }
                    return dec_thread[1];
                }
                else if (type == VIDEO_THREAD) {
                    if (vid_thread[1] == NULL) {
                        vid_thread[1] = SDL_CreateThread(video_thread, video);
                    }
                    return vid_thread[1];
                }
            }
            break;
        case 3: {
               if (type == DECODE_THREAD) {
                    if (dec_thread[2] == NULL) {
                        dec_thread[2] = SDL_CreateThread(decode_thread, video);
                    }
                    return dec_thread[2];
                }
                else if (type == VIDEO_THREAD) {
                    if (vid_thread[2] == NULL) {
                        vid_thread[2] = SDL_CreateThread(video_thread, video);
                    }
                    return vid_thread[2];
                }
            }
            break;

    }
    return NULL;
}

int connect_server() {
    int s, t, len;
    struct sockaddr_un remote;
    char str[100];

    if ((s = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        return -1;
    }

    remote.sun_family = AF_UNIX;
    strcpy(remote.sun_path, "unixetwo");
    len = strlen(remote.sun_path) + sizeof(remote.sun_family);
    if (connect(s, (struct sockaddr *)&remote, len) == -1) {
        perror("connect");
        return -1;
    }
}

int
main(int argc, char *argv[])
{
	SDL_Event event;
	//double pts;
	VideoState* is = NULL;
    VideoState* is2 = NULL;
    VideoState* is3 = NULL;
    VideoState* tempIs;

    res_image = (Resized*) malloc(sizeof(Resized));
    memset(res_image,0,sizeof(Resized));

    mutex = SDL_CreateMutex();
	// Register all formats and codecs
	av_register_all();

//	av_register_protocol(NULL);
    if (av_register_protocol2(&e2URLProtocol, size) < 0) {
        printf("Error registering the protocol. \n");
        return -1;
    }

  /*  if (connect_server() < 0) {
        printf("error connecting. \n");
        return -1;
    }
    */

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
		fprintf(stderr, "Could not initialize SDL - %s\n", SDL_GetError());
		exit(1);
	}

	// Make a screen to put our video
#ifndef __DARWIN__
	screen = SDL_SetVideoMode(640, 480, 0, 0);
#else
	screen = SDL_SetVideoMode(640, 480, 24, 0);
#endif
	if (!screen) {
		fprintf(stderr, "SDL: could not set video mode - exiting\n");
		exit(1);
	}

    if (argc < 2) {
		fprintf(stderr, "Usage: test <file>\n");
		exit(1);
	}
	is = av_mallocz(sizeof(VideoState));
	if (init_run_video(is,argv[1],1) < 0) {
        printf("Failed to init video : %s \n", argv[1]);
        return -1;
	}
	if (argc >= 3) {
        is2 = av_mallocz(sizeof(VideoState));
        if (init_run_video(is2,argv[2],2) < 0) {
            printf("Failed to init video : %s \n", argv[2]);
            return -1;
        }
        if (argc == 4) {
            is3 = av_mallocz(sizeof(VideoState));
            if (init_run_video(is3,argv[3],3) < 0) {
                printf("Failed to init video : %s \n", argv[3]);
                return -1;
            }
        }
	}
	av_init_packet(&flush_pkt);
	flush_pkt.data = (unsigned char *) "FLUSH";
	for (;;) {
		double incr, pos;
		SDL_WaitEvent(&event);
		switch (event.type) {
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_1:
                        if (is != NULL && !is->quit && (curr_video_on != 1 || curr_audio_on != 1)) {
                            if (pp_on) turn_off_pp();
                            turn_screen_black();
                            curr_video_on = 1;
                            curr_audio_on = 1;
                            reopen_audio(is);
                            goto do_seek;
                        }
                        break;
                    case SDLK_2:
                        if (is2 != NULL && !is2->quit && (curr_audio_on != 2 || curr_video_on != 2)) {
                            if (pp_on) turn_off_pp();
                            turn_screen_black();
                            curr_video_on = 2;
                            curr_audio_on = 2;
                            reopen_audio(is2);
                            goto do_seek;
                        }
                        break;
                    case SDLK_3:
                        if (is3 != NULL && !is3->quit && (curr_audio_on != 3 || curr_video_on != 3)) {
                            if (pp_on) turn_off_pp();
                            turn_screen_black();
                            curr_video_on = 3;
                            curr_audio_on = 3;
                            reopen_audio(is3);
                            goto do_seek;
                        }
                        break;
                    case SDLK_4:
                        if (is != NULL && !is->quit && curr_video_on != 1) {
                            if (pp_on) turn_off_pp();
                            curr_video_on = 1;
                            goto do_seek;
                        }
                        break;
                    case SDLK_5:
                        if (is2 != NULL && !is2->quit && curr_video_on != 2) {
                            if (pp_on) turn_off_pp();
                            curr_video_on = 2;
                            goto do_seek;
                        }
                        break;
                    case SDLK_6:
                        if (is3 != NULL && !is3->quit &&  curr_video_on != 3) {
                            if (pp_on) turn_off_pp();
                            curr_video_on = 3;
                            goto do_seek;
                        }
                        break;
                    case SDLK_7:
                        if (is != NULL && !is->quit && curr_audio_on != 1) {
                            if (pp_on) turn_off_pp();
                            curr_audio_on = 1;
                            reopen_audio(is);
                            goto do_seek;
                        }
                        break;
                    case SDLK_8:
                        if (is2 != NULL && !is2->quit && curr_audio_on != 2) {
                            if (pp_on) turn_off_pp();
                            curr_audio_on = 2;
                            reopen_audio(is2);
                            goto do_seek;
                        }
                        break;
                    case SDLK_9:
                        if (is3 != NULL && !is3->quit && curr_audio_on != 3) {
                            if (pp_on) turn_off_pp();
                            curr_audio_on = 3;
                            reopen_audio(is3);
                            goto do_seek;
                        }
                        break;
                    case SDLK_r:
                        video_color = RED;
                        break;
                    case SDLK_g:
                        video_color = GREEN;
                        break;
                    case SDLK_b:
                        video_color = BLUE;
                        break;
                    case SDLK_w:
                        video_color = BW;
                        break;
                    case SDLK_p:
                        if (pp_on) {
                            turn_off_pp();
                        }
                        else {
                            turn_on_pp();
                        }
                        break;
                    do_seek:
                    pos = calculate_pos();
                    if (curr_video_on == 1 || curr_audio_on == 1) {
                        stream_seek(global_video_state,
                                    (int64_t) (pos * AV_TIME_BASE), 1);
                        last_played_video = 1;
                    }
                    if (curr_video_on == 2 || curr_audio_on == 2) {
                        stream_seek(global_video_state2,
                                    (int64_t) (pos * AV_TIME_BASE), 1);
                        last_played_video = 2;
                    }
                    if (curr_video_on == 3 || curr_audio_on == 3) {
                        stream_seek(global_video_state3,
                                    (int64_t) (pos * AV_TIME_BASE), 1);
                        last_played_video = 3;
                    }
                        break;
                    default:
                        break;
                }
                break;
            case FF_QUIT_EVENT:
            case SDL_QUIT:
                SDL_CondSignal(is->audioq.cond);
                SDL_CondSignal(is->videoq.cond);
                is->quit = 1;
                SDL_Quit();
                exit(0);
                break;
            case FF_ALLOC_EVENT:
                alloc_picture(event.user.data1);
                break;
            case FF_REFRESH_EVENT:
                video_refresh_timer(event.user.data1);
                break;
            default:
                break;
		}
	}
	return 0;

}
