#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/un.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>

#define SOCK_PATH "unixetwo"
#define BUFSIZE 256

struct on_movies
{
    int sd;
    int fd;
    char name[40];
};

struct on_movies* get_movie(struct on_movies* movies, int length, int sd)
{
    int i;
    for (i = 0 ; i < length; ++i) {
        if (movies[i].sd == sd)
            return &movies[i];
    }
    return NULL;
}

int main(void)
{
    int s, t, len, fdmax, i, newfd;
    struct sockaddr_un local, remote;

    char choice;
    struct on_movies movies[2];
    int movie_arr_index = 0;
    fd_set master, read_fds;
    FD_ZERO(&master);
    FD_ZERO(&read_fds);
    t = sizeof(struct sockaddr_un);
    if ((s = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        exit(1);
    }

    local.sun_family = AF_UNIX;
    strcpy(local.sun_path, SOCK_PATH);
    unlink(local.sun_path);
    len = strlen(local.sun_path) + sizeof(local.sun_family);
    if (bind(s, (struct sockaddr *)&local, len) == -1)
    {
        perror("bind");
        exit(1);
    }

    if (listen(s, 4) == -1)
    {
        perror("listen");
        exit(1);
    }
    FD_SET(s, &master);
    fdmax = s;
    for(;;)
    {
        int  n;
        read_fds = master;
        if (select(fdmax + 1, &read_fds, NULL, NULL, NULL) == -1)
        {
            perror("select");
            exit(4);
        }
        for (i = 0; i <= fdmax; ++i)
        {
            if (FD_ISSET(i, &read_fds))
            {
                if (i == s)   // listener
                {
                    newfd = accept(s, (struct sockaddr*)&remote, &t);
                    printf("\n\n %d \n \n",newfd);
                    movies[movie_arr_index++].sd = newfd;
                    if (newfd == -1)
                    {
                        perror("accept");
                    }
                    else
                    {
			printf("Success! connected. \n");
                        FD_SET(newfd, &master);
                        if (newfd > fdmax)
                        {
                            fdmax = newfd;
                        }
                    }
                }
                else
                {
                    struct on_movies* the_movie = get_movie(movies, 2, i);
			if (recv(i, &choice, 1, 0) == -1) {
				perror("recv");			
			}
                        switch (choice)
                        {
                        case 'r':
                        {
				char file_name[30];
				int name_len;
				if (recv(i, &name_len, sizeof(int), 0) == -1) {
					perror("recv");
				}
				printf("%d \n", name_len);
				if (recv(i, file_name, name_len, 0) == -1) {
					perror("recv");				
				}
				file_name[name_len] ='\0';
				printf("%s \n", file_name);
				strcpy(the_movie->name, file_name + 11);
				printf("%s \n", the_movie->name);
				the_movie->fd = open(the_movie->name, O_RDWR);
				if (the_movie->fd >= 0) {
					char result = 'a';
					if (send(i, &result, 1, 0) < 0) {
						perror("send");					
					}
				}
				
                        }
                        break;
                        case 'f':
                        {
				int buffer_size;
				if (recv(i, &buffer_size, sizeof(int), 0) == -1) {
					perror("recv");				
				}
				char* buf = (char*) malloc(buffer_size);
				int nbytes = read(the_movie->fd, buf, buffer_size);
				if (send(i, &nbytes, sizeof(int), 0) < 0) {
					perror("send");			
				}
				if (send(i, buf, nbytes, 0) < 0) {
					perror("send");				
				}
				free(buf);
                        }
                        break;
                        case 's':
                        {
				int64_t pos; int whence;
				if (recv(i, &pos, sizeof(int64_t), 0) == -1) {
					perror("recv");				
				}	
				if (recv(i, &whence, sizeof(int), 0) == -1) {
					perror("recv");
				}
				int64_t result = lseek(the_movie->fd, pos, whence);
				if (send(i, &result, sizeof(int64_t), 0) == -1) {
					perror("send");				
				}
                        }
                        break;
			case 'c':
				close(the_movie->fd);
				close(the_movie->sd);
				FD_CLR(i, &master);
			break;
                        }
                    }
               
            }
        }
    }
}
