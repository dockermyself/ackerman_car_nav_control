//
// Created by server on 2022/4/29.
//

#ifndef RTK_NAV_UDP_PIECE_H
#define RTK_NAV_UDP_PIECE_H
#include "include/header.h"
#pragma pack(1)

struct UDP_PIECE {
    unsigned char code[2];  //标识该 piece 0xFF,0xAA
    int num;                //分片序号;
    int size;               //piece len
    char data[];            // 保存数据
};

struct PIECE_LIST {//分片列表
    struct PIECE_LIST *next;
    struct UDP_PIECE piece;
};

//首先接收到PIECE HEAD直到接下来接收数据情况,并应答握手
struct PIECE_HEAD {//分片头
    unsigned char piece_head[2];  //标识 0xFF,0x00
    unsigned char piece_code[2]; //标识 piece 0xFF,0xAA
    int total_piece;             // 分片总数量
    int total_size;              // 总size
};

struct PIECE_DATA{//分片数据
    const char *data;
    int data_len;
};
struct MERGE_PIECE_POOL {//合并分片池
    unsigned char piece_code[2];
    int total_piece;
    int total_size;
    int recv_piece;
    int recv_size;
    char *piece_buffer;
    struct PIECE_DATA piece_map[];//标识已经收到的piece

};
#pragma pack()

struct MERGE_PIECE_POOL *init_piece_pool(struct PIECE_HEAD *piece_head);//初始化合并分片池

void destroy_piece_pool(struct MERGE_PIECE_POOL *merge_piece);//销毁合并分片池

int merge_piece(const char *buffer, struct MERGE_PIECE_POOL *piece_pool);//合并分片

void order_and_swap(struct MERGE_PIECE_POOL *piece_pool);//排序并交换

struct PIECE_LIST *init_piece(struct PIECE_HEAD *piece_head, const char *msg, int msg_len, int piece_fix_size);//初始化分片

void destroy_piece(struct PIECE_LIST *piece);//销毁分片

#endif //RTK_NAV_UDP_PIECE_H
