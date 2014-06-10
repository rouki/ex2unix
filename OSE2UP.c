#include <libavformat/avio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <SDL/SDL.h>

#define BUFSIZE 256

int e2url_open(URLContext *h, const char *filename, int flags);
int e2url_read(URLContext *h, unsigned char *buf, int size);
int e2url_write(URLContext *h, unsigned char *buf, int size);
int64_t e2url_seek(URLContext *h, int64_t pos, int whence);
int e2url_close(URLContext *h);

URLProtocol e2URLProtocol = {
	.name = "unixetwo",			//name must be alphabetic.
	.url_open = e2url_open,
	.url_read = e2url_read,
	.url_write = e2url_write,
	.url_seek = e2url_seek,
	.url_close = e2url_close,

};

int prot_size = sizeof(e2URLProtocol);

int s = -1;
int s2 = -1;

struct sockaddr_un remote;
SDL_mutex* socket_mutex;

int
e2url_open(URLContext *h, const char *filename, int flags)
{
    SDL_LockMutex(socket_mutex);
	int t, len;
    char str[BUFSIZE];
    if (s == -1)
        h->priv_data = &s;
    else if (s2 == -1)
        h->priv_data = &s2;
    int* sock = (int*)h->priv_data;
    if ((*sock = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        return -1;
    }
    remote.sun_family = AF_UNIX;
    strcpy(remote.sun_path, "unixetwo");
    len = strlen(remote.sun_path) + sizeof(remote.sun_family);
    if (connect(*sock, (struct sockaddr *)&remote, len) == -1) {
        perror("connect");
        return -1;
    }
    str[0] = 'r';
    strcpy(str + 1, filename);
    if (send(*sock, str, strlen(str), 0) == -1) {
            perror("send");
            return -1;
    }
    if ((t = recv(*sock, str, BUFSIZE, 0)) > 0) {
            str[t] = '\0';
    }
    printf("%c", str[0]);
    SDL_UnlockMutex(socket_mutex);
    return str[0] == 'a' ? 0 : -1;
}

int
e2url_read(URLContext *h, unsigned char *buf, int size)
{
    printf("Size = %d\n",size);
    char str[BUFSIZE];
    int n = 0;
    int copy_to = 0;
    str[0] = 'f';
    int sock = *(int*)h->priv_data;

    memcpy(str + 1, &size, sizeof(int));
    if (send(sock, str, BUFSIZE, 0) < 0) {
        perror("send");
        return 0;
    }
    while (size > 0) {
        n = recv(sock, str, BUFSIZE, 0);
        ssize_t data_size;
        memcpy(&data_size, str, sizeof(ssize_t));
        memcpy(buf + copy_to, str + 4, data_size);
        copy_to += data_size;
        size -= data_size;
    }
    printf("copy_to = %d \n", copy_to);
    return copy_to;
}

int
e2url_write(URLContext *h, unsigned char *buf, int size)
{
	//Not implemented.
	return -1;
}

int64_t
e2url_seek(URLContext *h, int64_t pos, int whence)
{
    return -1;
    char str[BUFSIZE];
    int n = 0;
    static int count = 0;
    int sock = *(int*)h->priv_data;
    int64_t result = 0;
    str[0] = 's';
    memcpy(str + 1, &pos, sizeof(int64_t));
    memcpy(str + 1 + sizeof(int64_t), &whence, sizeof(int));
    if (send(sock, str, BUFSIZE, 0) < 0) {
        perror("send");
        return -1;
    }
    n = recv(sock, str, BUFSIZE, 0);
    memcpy(&result, str, sizeof(int64_t));
    printf("result = %lld \n", result);
    return result;
}

int e2url_close(URLContext *h)
{
    fclose(s);
}
