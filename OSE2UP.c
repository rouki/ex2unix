#include <libavformat/avio.h>

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

int e2url_open(URLContext *h, const char *filename, int flags)
{
	//Open the file on the server side and prepare it for reading.
	return -1;
}

int e2url_read(URLContext *h, unsigned char *buf, int size)
{
	//Read the file on server side and tell it to send the buffer here.
	return -1;
}

int e2url_write(URLContext *h, unsigned char *buf, int size)
{
	//Not implemented.
	return -1;
}

int64_t e2url_seek(URLContext *h, int64_t pos, int whence)
{
	//Tell the server to do a seek on the file.
	return -1;
}

int e2url_close(URLContext *h)
{
	//Tell the server to close the file.
	return -1;
}
