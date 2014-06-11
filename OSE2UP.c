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
SDL_mutex* socket_mutex;

int
e2url_open(URLContext *h, const char *filename, int flags)
{
	int t, len;
	char result;
	char str[BUFSIZE];
	struct sockaddr_un remote;
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
    int name_len = strlen(filename);
    memcpy(str + 1, &name_len, sizeof(int));
    strcpy(str + sizeof(int) + 1, filename);
    if (send(*sock, str, strlen(filename) + sizeof(int) + 1, 0) == -1) {
            perror("send");
            return -1;
    }
    if (recv(*sock, &result, 1, 0) == -1) {
        perror("recv");
        return -1;
    }
    printf("%c \n", result);
    return result == 'a' ? 0 : -1;
}

int
e2url_read(URLContext *h, unsigned char *buf, int size)
{
    int sock = *(int*)h->priv_data;
    int num_of_bytes;
    char command = 'f';
    if (send(sock, &command, 1, 0) < 0) {
        perror("send");
    }
    if (send(sock, &size, sizeof(int), 0) < 0) {
        perror("send");
    }
    if (recv(sock, &num_of_bytes, sizeof(int), 0) == -1) {
        perror("recv");
    }
    if (recv(sock, buf, num_of_bytes, 0) == -1) {
        perror("recv");
    }
    printf("num of bytes = %d \n", num_of_bytes);
    return num_of_bytes;
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
    int sock = *(int*)h->priv_data;
    int64_t result;
    char command = 's';
    if (send(sock, &command, 1, 0) < 0) {
        perror("send");
    }
    if (send(sock, &pos, sizeof(int64_t), 0) < 0) {
        perror("send");
    }
    if (send(sock, &whence, sizeof(int) , 0) < 0) {
        perror("send");
    }
    if (recv(sock, &result, sizeof(int64_t), 0) == -1) {
        perror("recv");
    }
    return result;
}

int e2url_close(URLContext *h)
{
    int sock = *(int*)h->priv_data;
    char command = 'c';
    if (send(sock, &command, 1, 0) < 0) {
        perror("send");
    }
    close(sock);
}
