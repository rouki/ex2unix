#include <libavformat/avio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

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

int size = sizeof(e2URLProtocol);
int s;
struct sockaddr_un remote;

int e2url_open(URLContext *h, const char *filename, int flags)
{
	int t, len;
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
    str[0] = 'r';
    strcpy(str + 1, filename);
    if (send(s, str, strlen(str), 0) == -1) {
            perror("send");
            return -1;
    }
    if ((t = recv(s, str, 100, 0)) > 0) {
            str[t] = '\0';
    }
    printf("%c", str[0]);
    return str[0] == 'a' ? 0 : -1;
}

int e2url_read(URLContext *h, unsigned char *buf, int size)
{
    char str[BUFSIZE];
    int n = 0;
    int copy_to = 0;
    char term = '\0';
    str[0] = 'f';
    sprintf(str + 1, "%d%c", size, term);
    if (send(s, str, sizeof(int) + 2, 0) < 0) {
        perror("send");
        return 0;
    }
    while (size > 0) {
        n = recv(s, str,sizeof(str), 0);
        int data_size = atoi(str);
        printf("DATA SIZE = %d\n",data_size);
        if (data_size < BUFSIZE - 5) {
            size = -1;
        } else {
            size -= data_size;
        }
        memcpy(buf + copy_to, str + 5, data_size);
        copy_to += data_size;
    }
    printf("copy_to = %d \n", copy_to);
    return copy_to;
}

int e2url_write(URLContext *h, unsigned char *buf, int size)
{
	//Not implemented.
	return -1;
}

int64_t e2url_seek(URLContext *h, int64_t pos, int whence)
{
    /*
    char str[BUFSIZE];
    int n = 0;
    int copy_to = 0;
    char term = '\0';
    str[0] = 's';
    sprintf(str + 1, "%lld%c%d%c", pos, term, whence, pos);
    if (send(s, str, sizeof(int64_t) + sizeof(int) + 2, 0) < 0) {
        perror("send");
        return 0;
    }
    n = recv(s, str,sizeof(str), 0);
    uint64_t result_pos = atoll(str);
    return result_pos;
    */
}

int e2url_close(URLContext *h)
{
    fclose(s);
}
