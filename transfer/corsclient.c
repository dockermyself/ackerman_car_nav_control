
#include "corsclient.h"

#define NTRIPCLIENT_AGENTSTRING    "NTRIP NtripClientPOSIX"/* The string, which is send as agent in HTTP request */
//定于全局变量
static const char *ntripclient_revisionstr = "$Revision: 1.51 $";
static const char *nmeahead = "$GPGGA";
static const char encodingTable[64] = {
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
        'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
        'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
        'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/'
};
#define INVALID_SOCKET  (-1)
static SOCKETTYPE socketfd = INVALID_SOCKET;
int connect_status = cors_disconnected;

//功能函数
static int encode(char *buf, int size, const char *user, const char *pwd) {
    unsigned char inbuf[3];
    char *out = buf;
    int i, sep = 0, fill = 0, bytes = 0;

    while (*user || *pwd)
    {
        i = 0;
        while (i < 3 && *user) inbuf[i++] = *(user++);
        if (i < 3 && !sep)
        {
            inbuf[i++] = ':';
            ++sep;
        }
        while (i < 3 && *pwd) inbuf[i++] = *(pwd++);
        while (i < 3)
        {
            inbuf[i++] = 0;
            ++fill;
        }
        if (out - buf < size - 1)
            *(out++) = encodingTable[(inbuf[0] & 0xFC) >> 2];
        if (out - buf < size - 1)
            *(out++) = encodingTable[((inbuf[0] & 0x03) << 4)
                                     | ((inbuf[1] & 0xF0) >> 4)];
        if (out - buf < size - 1)
        {
            if (fill == 2)
                *(out++) = '=';
            else
                *(out++) = encodingTable[((inbuf[1] & 0x0F) << 2)
                                         | ((inbuf[2] & 0xC0) >> 6)];
        }
        if (out - buf < size - 1)
        {
            if (fill >= 1)
                *(out++) = '=';
            else
                *(out++) = encodingTable[inbuf[2] & 0x3F];
        }
        bytes += 4;
    }
    if (out - buf < size)
        *out = 0;
    return bytes;
}

int cors_connect(cors_client_t *cors_cli) {
    const int buffer_size = 256;
    struct sockaddr_in their_addr; /* connector's address information */
    struct hostent *he;
    char buff[buffer_size];
    memset(buff, 0, buffer_size);
    memset(&their_addr, 0, sizeof(struct sockaddr_in));
    their_addr.sin_port = htons(cors_cli->dst_port);
    if (!(he = gethostbyname(cors_cli->dst_host)))
    {
        printf("Server name lookup failed for %s (%s)\n", cors_cli->dst_host, strerror(errno));
        return -1;
    }
    if ((socketfd = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
    {
        printf("Create socket error (%s)\n", strerror(errno));
        return -1;
    }
    else
    {
        their_addr.sin_family = AF_INET;
        their_addr.sin_addr = *((struct in_addr *) he->h_addr);
    }

    printf("client connecting to cors ...\n");
    if (connect(socketfd, (struct sockaddr *) &their_addr, sizeof(struct sockaddr)) == -1)
    {
        printf("Connect error (%s)\n", strerror(errno));
        return -1;
    }

    int i = snprintf(buff, buffer_size - 40, /* leave some space for login */
                     "GET /%s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "User-Agent: %s/%s\r\n"
                     "%s%s%s"
                     "Connection: close%s",
                     cors_cli->dst_mount,
                     cors_cli->dst_host,
                     NTRIPCLIENT_AGENTSTRING, ntripclient_revisionstr,
                     nmeahead ? "Ntrip-GGA: " : "", nmeahead ? nmeahead : "", nmeahead ? "\r\n" : "",
                     (*(cors_cli->dst_user) || *(cors_cli->dst_password)) ? "\r\nAuthorization: Basic " : "");

    if (i > buffer_size - 40 || i < 0)
    {
        printf("Requested data too long\n");
        return -1;
    }
    else
    {
        i += encode(buff + i, buffer_size - i - 4, cors_cli->dst_user, cors_cli->dst_password);
        if (i > buffer_size - 4)
        {
            printf("Username and/or password too long\n");
            return -1;
        }
        else
        {
            buff[i++] = '\r';
            buff[i++] = '\n';
            buff[i++] = '\r';
            buff[i++] = '\n';
        }
        //here add nmea
    }

    printf("Ntrip client head:\n%s\n", buff);
    if (send(socketfd, buff, (size_t) i, 0) < i)
    {
        printf("Send error (%s)\n", strerror(errno));
        return -1;
    }
    else
    {
        memset(buff, 0, buffer_size);
        ssize_t nbytes = recv(socketfd, buff, buffer_size - 1, 0);
        buff[nbytes] = '\0'; /* latest end mark for strstr */
        if (!strstr(buff, "ICY 200 OK"))
        {
            printf("Could not get the requested data");
            return -1;
        }
        printf("Destination caster response: %s", buff);

    }
    printf("connected\n");

    //设置ntrip连接状态
    set_connect_status(cors_connected);
    //设置非阻塞IO
    int flags;
    if ((flags = fcntl(socketfd, F_GETFL, NULL)) < 0)
    {
        return -1;
    }
    if (fcntl(socketfd, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        return -1;
    }

    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGPIPE);
    sigprocmask(SIG_BLOCK, &set, NULL);
    return 0;
}

int cors_recv(char *recv_buf, int recv_size) {//接收
    if (recv_buf == NULL || recv_size == 0)
    {
        printf("Input parameter(s) invalid!\n");
        return -1;
    }
    memset(recv_buf, 0, recv_size);
    return (int) recv(socketfd, recv_buf, (int) recv_size, MSG_DONTWAIT);
}

int cors_send(const char *send_buf, int send_size) {//发送
    if (send_buf == NULL || send_size == 0)
    {
        printf("Input parameter(s) invalid!\n");
        return -1;
    }
    return (int) send(socketfd, send_buf, (size_t) send_size, MSG_DONTWAIT);
}


void cors_close(void) {
    if (socketfd != INVALID_SOCKET)
    {
        close(socketfd);
        socketfd = INVALID_SOCKET;
        printf("cors client closed!\n");
    }
}
/*
    1.IP地址：120.253.226.97(备用IP： 117.135.142.201)
    2.端口：8001（CGCS2000坐标系)  8002（WGS84坐标系）  8003（ITRF2008坐标框架）
    3.源节点 ：(四星)源节点：  RTCM33_GRCE   (三星)源节点：RTCM33_GRC
 */

cors_client_t *cors_client_init(const char *host, short port,
                                  const char *mount, const char *user, const char *password) {
    printf("host:%s,port:%d,mount:%s,user:%s,password:%s\n",host,port,mount,user,password);
    cors_client_t *cors_cli = malloc(sizeof(cors_client_t));
    memset(cors_cli, 0, sizeof(cors_client_t));
    memcpy(cors_cli->dst_host, host, strlen(host) + 1);
    cors_cli->dst_port = port;
    memcpy(cors_cli->dst_mount, mount, strlen(mount) + 1);
    memcpy(cors_cli->dst_user, user, strlen(user) + 1);
    memcpy(cors_cli->dst_password, password, strlen(password) + 1);
    return cors_cli;

}

void cors_client_destroy(cors_client_t *cors_cli) {
    if (cors_cli != NULL)
    {
        free(cors_cli);
    }
}

int cors_socketfd() {
    return socketfd;
}

int cors_keep_connected() {
    if (connect_status == cors_connected)
        return 1;
    return 0;
}

void set_connect_status(enum NTRIP_STATUS status) {
    connect_status = status;
}

