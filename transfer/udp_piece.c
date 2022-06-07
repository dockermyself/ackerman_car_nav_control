//
// Created by server on 2022/4/29.
//
#include "udp_piece.h"


void destroy_piece(struct PIECE_LIST *piece) {
    while (piece)
    {
        struct PIECE_LIST *list = piece;
        piece = piece->next;
        free(list);
    }
}


struct PIECE_LIST *init_piece(struct PIECE_HEAD *piece_head, const char *msg, int msg_len, int piece_fix_size) {
    if (msg_len < sizeof(struct PIECE_HEAD))
    {
        return NULL;
    }
    piece_head->piece_head[0] = 0xFF;
    piece_head->piece_head[1] = 0x00;
    piece_head->piece_code[0] = 0xAA;
    piece_head->piece_code[1] = 0xBB;
    struct PIECE_LIST *piece_list_head = NULL;//init head
    struct PIECE_LIST *piece_list_tail = NULL;//init tail
    int piece_rest = msg_len;
    const char *piece_data = msg;
    int i = 0;
    while (piece_rest > 0)
    {
        int piece_size = piece_rest > piece_fix_size ? piece_fix_size : piece_rest;
        size_t mem_size = sizeof(struct PIECE_LIST) + piece_size;
        struct PIECE_LIST *piece_node = malloc(mem_size);
        memset(piece_node, 0, mem_size);

        if (piece_node == NULL)
        {
            destroy_piece(piece_list_head);
            return NULL;
        }
        piece_node->next = NULL;
        memcpy(piece_node->piece.data, piece_data, piece_size);
        piece_node->piece.size = piece_size;
        piece_node->piece.num = i;
        piece_node->piece.code[0] = 0xAA;
        piece_node->piece.code[1] = 0xBB;
        piece_rest = piece_rest - piece_size;
        piece_data = piece_data + piece_size;
        if (piece_list_head == NULL)
        {
            piece_list_head = piece_node;//head node
            piece_list_tail = piece_node;//tail node

        }
        else
        {
            piece_list_tail->next = piece_node;//insert to tail
            piece_list_tail = piece_node;//update tail
        }
        piece_head->total_piece++;
        piece_head->total_size += piece_size;
        i++;
    }

    return piece_list_head;//return head node
}


//当解析到head，创建该PIECE_POOL
struct MERGE_PIECE_POOL *init_piece_pool(struct PIECE_HEAD *piece_head) {
    if (NULL == piece_head)
    {
        return NULL;
    }
    size_t mem_size = sizeof(struct MERGE_PIECE_POOL) + piece_head->total_piece * sizeof(struct PIECE_DATA);
    struct MERGE_PIECE_POOL *piece_pool = malloc(mem_size);
    memset(piece_pool, 0, mem_size);
    memcpy(piece_pool->piece_code, piece_head->piece_code, sizeof(piece_head->piece_code));
    piece_pool->total_piece = piece_head->total_piece;
    piece_pool->total_size = piece_head->total_size;
    piece_pool->piece_buffer = malloc(piece_head->total_size);



    if (!piece_pool->piece_buffer)
    {
        destroy_piece_pool(piece_pool);
        return NULL;
    }

    return piece_pool;
}

void destroy_piece_pool(struct MERGE_PIECE_POOL *merge_piece) {
    if (NULL == merge_piece)
    {
        return;
    }
    if (merge_piece->piece_buffer)
    {
        free(merge_piece->piece_buffer);
        merge_piece->piece_buffer = NULL;
    }
    free(merge_piece);
}


void order_and_swap(struct MERGE_PIECE_POOL *piece_pool) {
    char *buffer = malloc(piece_pool->total_size );
    memset(buffer, 0, piece_pool->total_size);
    int mem_offset = 0;
    for (int i = 0; i < piece_pool->total_piece; i++)
    {
        memcpy(buffer + mem_offset, piece_pool->piece_map[i].data, piece_pool->piece_map[i].data_len);
        mem_offset += piece_pool->piece_map[i].data_len;
    }
    free(piece_pool->piece_buffer);
    piece_pool->piece_buffer = buffer;
}

//检测piece完整性
int check_piece_complete(struct MERGE_PIECE_POOL *piece_pool) {
    if (NULL == piece_pool)
    {
        return 0;
    }
    for (int i = 0; i < piece_pool->total_piece; i++)
    {
        if (piece_pool->piece_map[i].data_len == 0 && piece_pool->piece_map[i].data == NULL)
        {
            return 0;
        }
    }
    return 1;
}


//return 1数据完整，0数据为完整
int merge_piece(const char *buffer, struct MERGE_PIECE_POOL *piece_pool) {
    struct UDP_PIECE *piece = (struct UDP_PIECE *) buffer;
    if (piece->code[0] != piece_pool->piece_code[0] || piece->code[1] != piece_pool->piece_code[1])
    {
        printf("piece code not match\n");
        return 0;
    }
    int i = piece->num;
    //将buffer拷贝到piece_pool->piece_buffer
    char *dest = piece_pool->piece_buffer + piece_pool->recv_size;
    memcpy(dest, piece->data, piece->size);
    piece_pool->piece_map[i].data_len = piece->size;
    piece_pool->piece_map[i].data = dest;
    piece_pool->recv_piece += 1;
    piece_pool->recv_size += piece->size;

    if (piece_pool->recv_size == piece_pool->total_size &&
        piece_pool->recv_piece == piece_pool->total_piece &&
        check_piece_complete(piece_pool))
    {
        order_and_swap(piece_pool);//piece pool数据完成
        return 1;
    }
    return 0;
}







