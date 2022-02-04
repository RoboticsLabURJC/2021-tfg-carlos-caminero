#ifndef PROXY_H
#define PROXY_H

#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <err.h>

int create_socket(void);
void close_socket(int fd);
void set_ip_port(struct sockaddr_in & dest_addr, const char * ip, int port);
int send_message(int fd, struct sockaddr_in & dest_addr, void * buf, size_t len);

#endif // PROXY_H
