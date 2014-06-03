#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <inttypes.h>

#define SOCK_PATH "unixetwo"
#define BUFSIZE 256

int main(void)
{
    int s, s2, t, len;
    struct sockaddr_un local, remote;
    char str[BUFSIZE];
    char temp[BUFSIZE];
    FILE* f1, *f2, *f3;
    if ((s = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }

    local.sun_family = AF_UNIX;
    strcpy(local.sun_path, SOCK_PATH);
    unlink(local.sun_path);
    len = strlen(local.sun_path) + sizeof(local.sun_family);
    if (bind(s, (struct sockaddr *)&local, len) == -1) {
        perror("bind");
        exit(1);
    }

    if (listen(s, 5) == -1) {
        perror("listen");
        exit(1);
    }

    for(;;) {
        int done, n;
        printf("Waiting for a connection...\n");
        t = sizeof(remote);
        if ((s2 = accept(s, (struct sockaddr *)&remote, &t)) == -1) {
            perror("accept");
            exit(1);
        }

        printf("Connected.\n");

        done = 0;
        do {
            n = recv(s2, str, sizeof(str), 0);
            if (n <= 0) {
                if (n < 0) perror("recv");
                done = 1;
            } else {
                    switch (str[0]) {
                    case 'r':
                            {
                                str[n] = '\0';
                                char* file_name = str + 12;
                                printf("%s \n", file_name);
                                if ( (f1 = fopen(file_name,"r")) == NULL) {
                                    str[0] = 'n';
                                } else {
                                    str[0] = 'a';
                                }
                                if (send(s2, str, 1, 0) < 0) {
                                        perror("send");
                                }
                            }
                        break;
                        case 'f':
                        {
			    int size = 0;
			    str[n] = '\0';
			    char term = '\0';
			    size = atoi(str + 1);
			    int nbytes_read = 0;
                            while (size > 0) {
				if (size < BUFSIZE - 5) {
					nbytes_read = fread(temp, 1, size , f1);	
					size = -1;
				} else {
                                	nbytes_read = fread(temp, 1 ,BUFSIZE - (sizeof(int) + 1) , f1);
				}
                                sprintf(str,"%d%c", nbytes_read, term);
                                memcpy(str + sizeof(int) + 1, temp, nbytes_read);
                                if (send(s2, str, sizeof(str), 0) < 0) {
                                    perror("send");
                               	}
				if (size == -1)
					break;
                                size -= nbytes_read;
                            }
				printf("Done. sent all \n");
                        }
                    break;
		      case 's':
			{
				char term = '\0';
				uint64_t pos = atoll(str + 1);
				printf("%lld \n", pos);
				int whence = atoi(str + 9);
				printf("%d \n", whence);
				uint64_t seek = fseek(f1, pos, whence);
				sprintf(str, "%lld%c", seek, term);
				if (send(s2, str, sizeof(uint64_t) + 1, 0) < 0) {
					perror("send");				
				}
			}
			break;
                }
            }
        } while (!done);

        close(s2);
    }

    return 0;
}

