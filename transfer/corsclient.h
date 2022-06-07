#ifndef NTRIPCLIENT_H
#define NTRIPCLIENT_H


#include "include/header.h"
typedef int SOCKETTYPE;
enum NTRIP_STATUS{
    cors_disconnected = 0,
    cors_connected = 1
};
#pragma pack(1)
typedef struct {
    char dst_host[20];
    uint16_t dst_port;
    char dst_mount[20];
    char dst_user[20];
    char dst_password[20];
} cors_client_t;
#pragma pack()
int cors_socketfd();
int cors_keep_connected();
void set_connect_status(enum NTRIP_STATUS status);
int cors_connect(cors_client_t *cors_cli);
int cors_recv( char *recv_buf, int recv_size);
int cors_send(const char *send_buf, int send_size);
void cors_close(void);
cors_client_t* cors_client_init(const char *host, short port,const char *mount, const char *user, const char *password);
void cors_client_destroy(cors_client_t *cors_cli);

#endif
