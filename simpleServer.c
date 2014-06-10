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
    int movie_arr_index = 0;
    struct sockaddr_un local, remote;
    char str[BUFSIZE];
    char temp[BUFSIZE];
    struct on_movies movies[2];
    fd_set master, read_fds;
    FD_ZERO(&master);
    FD_ZERO(&read_fds);
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

    if (listen(s, 2) == -1)
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
                    //  printf("\n\n %d \n \n",newfd);
                    movies[movie_arr_index++].sd = newfd;
                    if (newfd == -1)
                    {
                        perror("accept");
                    }
                    else
                    {
                        FD_SET(newfd, &master);
                        if (newfd > fdmax)
                        {
                            fdmax = newfd;
                        }
                    }
                }
                else
                {
                    //        printf("Request from : %d \n", i);
                    struct on_movies* the_movie = get_movie(movies, 2, i);
                    //         printf("MOVIE : %d \n", the_movie->sd);
                    n = recv(i, str, sizeof(str), 0);
                    if (n <= 0)
                    {
                        if (n == 0)
                        {
                       //     close(i);
                       //     FD_CLR(i, &master);
                       //     close(the_movie->fd);
                        }
                    }
                    else
                    {
                        switch (str[0])
                        {
                        case 'r':
                        {
                            str[n] = '\0';
                            char* file_name = str + 12;
                            strcpy(the_movie->name, file_name);
                            //         printf("%s \n", file_name);
                            if ( (the_movie->fd = open(the_movie->name, O_RDWR)) == -1)
                            {
                                printf("Failed. \n");
                                str[0] = 'n';
                            }
                            else
                            {
                                str[0] = 'a';
                            }
                            if (send(i, str, BUFSIZE, 0) < 0)
                            {
                                perror("send");
                            }
                        }
                        break;
                        case 'f':
                        {

                            //         printf("Read request! socket : %d \n", i);
                            int size = 0;
                            memcpy(&size, str + 1, sizeof(int));
                            ssize_t nbytes_read = 0;
                            //       printf("movie_name : %s \n", the_movie->name);
                            //         printf("socket number : %d \n", the_movie->sd);
                            //         printf("i : %d \n", i);
               //             printf("Data size given : %d \n", size);
                            while (size > 0)
                            {
                                if (size >= BUFSIZE - 4)
                                    nbytes_read = read(the_movie->fd, temp, BUFSIZE - 4);
                                else
                                    nbytes_read = read(the_movie->fd, temp, size);

                                if (0 == strcmp(the_movie->name, "boy_named_sue.avi")) {
                                          printf("bytes read = %d \n", nbytes_read);
                                }       
                                //      printf("Bytes read : %d \n", nbytes_read);
                                memcpy(str, &nbytes_read, sizeof(ssize_t));
                                memcpy(str + sizeof(int), temp, nbytes_read);
                 //               if (0 == strcmp(the_movie->name, "boy_named_sue.avi")) {
                 //                       printf("Sent to bns.avi : socket - %d , fd - %d \n", the_movie->sd, the_movie->fd);
                 //               } else {
                //                        printf("Sent to francegall.avi : socket - %d , fd - %d \n", the_movie->sd, the_movie->fd);
                  //              }
                                if (send(i, str, BUFSIZE, 0) < 0)
                                {
                                    perror("send");
                                }
                                size -= nbytes_read;
                            }
                        }
                        break;
                        case 's':
                        {
                                int64_t pos; int whence;
                                memcpy(&pos, str + 1, sizeof(int64_t));
                                memcpy(&whence, str + sizeof(int64_t) + 1, sizeof(int));
                                int64_t result = lseek(the_movie->fd, pos, whence);
                                memcpy(str, &result, sizeof(int64_t));
                                if (send(i, str, BUFSIZE, 0) < 0) {
                                        perror("send");
                                }
                        }
                        break;
                        }
                    }
                }
            }
        }
    }
}
