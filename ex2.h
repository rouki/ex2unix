#define SDL_AUDIO_BUFFER_SIZE 1024

#define MAX_AUDIOQ_SIZE (5 * 16 * 1024)
#define MAX_VIDEOQ_SIZE (5 * 256 * 1024)

#define AV_SYNC_THRESHOLD 0.01
#define AV_NOSYNC_THRESHOLD 10.0

#define SAMPLE_CORRECTION_PERCENT_MAX 10
#define AUDIO_DIFF_AVG_NB 20
#define AVCODEC_MAX_AUDIO_FRAME_SIZE 192000

#define FF_ALLOC_EVENT   (SDL_USEREVENT)
#define FF_REFRESH_EVENT (SDL_USEREVENT + 1)
#define FF_QUIT_EVENT (SDL_USEREVENT + 2)

#define VIDEO_PICTURE_QUEUE_SIZE 1

#define DEFAULT_AV_SYNC_TYPE AV_SYNC_VIDEO_MASTER

#define SERVER_IP "127.0.0.1"
#define DECR_BY 2

typedef struct Resized_Image {
    int flag;
    int width, height; /* resized image heigh & width */
    AVFrame* small_image; /* the resized image */
    SDL_mutex* mutex;
    SDL_cond*  cond;
}Resized;

typedef struct PacketQueue {
	AVPacketList *first_pkt, *last_pkt;
	int nb_packets;
	int size;
	SDL_mutex *mutex;
	SDL_cond *cond;
} PacketQueue;


typedef struct VideoPicture {
	SDL_Overlay *bmp;
	int width, height; /* source height & width */
	int allocated;
	double pts;
} VideoPicture;

enum {
	AV_SYNC_AUDIO_MASTER, AV_SYNC_VIDEO_MASTER, AV_SYNC_EXTERNAL_MASTER,
};

typedef struct VideoState {

	AVFormatContext *pFormatCtx;
	int videoStream, audioStream;

	int av_sync_type;
	double external_clock; /* external clock base */
	int64_t external_clock_time;
	int p2;

	int seek_req;
	int seek_flags;
	int64_t seek_pos;

	double audio_clock;
	AVStream *audio_st;
	PacketQueue audioq;
	uint8_t audio_buf[(AVCODEC_MAX_AUDIO_FRAME_SIZE * 3) / 2];
	unsigned int audio_buf_size;
	unsigned int audio_buf_index;
	AVPacket audio_pkt;
	uint8_t *audio_pkt_data;
	int audio_pkt_size;
	AVFrame audio_frame;
	AVStream *video_st;
	PacketQueue videoq;
	int audio_hw_buf_size;
	double audio_diff_cum; /* used for AV difference average computation */
	double audio_diff_avg_coef;
	double audio_diff_threshold;
	int audio_diff_avg_count;
	double frame_timer;
	double frame_last_pts;
	double frame_last_delay;

	double video_current_pts; ///<current displayed pts (different from video_clock if frame fifos are used)
	int64_t video_current_pts_time; ///<time (av_gettime) at which we updated video_current_pts - used to have running video pts

	double video_clock; ///<pts of last decoded frame / predicted pts of next decoded frame

	VideoPicture pictq[VIDEO_PICTURE_QUEUE_SIZE];
	int pictq_size, pictq_rindex, pictq_windex;
	SDL_mutex *pictq_mutex;
	SDL_cond *pictq_cond;

	SDL_Thread *parse_tid;
	SDL_Thread *video_tid;

	SDL_AudioSpec spec;

	AVIOContext *io_ctx;
	struct SwsContext *sws_ctx;

    int id;
	char filename[1024];
	int quit;
} VideoState;

typedef enum {REGULAR, RED, GREEN, BLUE, BW} VIDEO_COLOR ;

typedef enum {VIDEO_THREAD, DECODE_THREAD} THREAD_TYPE;

int curr_video_on = 1;
int curr_audio_on = 1;
int ff_on = 1;
int last_played_video = 1;
int pp_on = 0;

Resized* res_image;
SDL_mutex* mutex;
//SDL_mutex* pp_mutex;
//SDL_cond* pp_cond;

VIDEO_COLOR video_color = REGULAR;

SDL_Surface *screen;

uint64_t global_video_pkt_pts = AV_NOPTS_VALUE;
uint64_t global_video_pkt_pts2 = AV_NOPTS_VALUE;
uint64_t global_video_pkt_pts3 = AV_NOPTS_VALUE;

/* Since we only have one decoding thread, the Big Struct
 can be global in case we need it. */
VideoState* global_video_state;
VideoState* global_video_state2;
VideoState* global_video_state3;

AVPacket flush_pkt;

static SDL_Thread*  get_thread (VideoState* video, THREAD_TYPE type);
AVFrame*            YUV_TO_RGB(AVFrame* origin, AVCodecContext* pCodecCtx);
AVFrame*            RGB_TO_YUV(AVFrame* origin, AVCodecContext* pCodecCtx);
AVFrame*            merge_frames (AVFrame* dest_image, VideoState* is1);
void                resize_image(AVFrame* image1, AVCodecContext* pCodecCtx_image1, AVCodecContext* pCodecCtx_image2);
void                turn_off_pp();
void                turn_on_pp();
int                 number_of_movies();
AVFrame*            handle_merge (AVFrame* origin, VideoState* is);
AVFrame*            handle_color (AVFrame* origin, VideoState* is);
void                init_spec(VideoState* is, int streamIndex);
void                reopen_audio(VideoState* is);
double              calculate_pos();
void                turn_screen_black();
int                 init_run_video(VideoState* is, const char* name, int id);
