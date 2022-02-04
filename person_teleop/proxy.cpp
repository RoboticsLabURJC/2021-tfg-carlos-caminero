#include "proxy.h"

int create_socket(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1) {
        err(EXIT_FAILURE, "socket failed");
    }
    return fd;
}

void close_socket(int fd)
{
    if (close(fd) == -1) {
        err(EXIT_FAILURE, "close failed");
    }
}

void set_ip_port(struct sockaddr_in & dest_addr, const char * ip, int port)
{
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(ip);
    dest_addr.sin_port = htons(port);
}

int send_message(int fd, struct sockaddr_in & dest_addr, void * buf, size_t len)
{
    if (sendto(fd, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof (struct sockaddr)) == -1) {
        warn("sendto failed");
        return 1;
    }
    return 0;
}


