//
// Created by server on 2022-05-18.
//

#include "log.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
static struct LogNode *log_head = NULL;
static struct LogNode *log_tail = NULL;
static char LOG_PATH[256] = {0};
const int MAX_LOG_SIZE = 256;
//insert logger tail
static void insert_log(const char* path, FILE * logger){
    struct LogNode *node = (struct LogNode *)malloc(sizeof(struct LogNode));
    stpcpy(node->path, path);//copy with \0
    node->logger = logger;
    if(log_head == NULL){
        log_head = node;
        log_tail = node;
    }else{
        log_tail->next = node;
        log_tail = node;
    }
    log_tail->next = NULL;
}

static void delete_log(const char * path){
    struct LogNode *node = log_head;
    struct LogNode *pre = NULL;
    while(node != NULL){
        if(strcmp(node->path, path) == 0){
            if(pre == NULL){
                log_head = node->next;//delete head
            }else{
                pre->next = node->next;//delete middle
            }
            if(node == log_tail){//delete tail
                log_tail = pre;
            }
            free(node);
            return;
        }
        pre = node;
        node = node->next;
    }
}

struct LogNode * get_log(const char * path){
    struct LogNode *node = log_head;
    while(node != NULL){
        if(strcmp(node->path, path) == 0){
            return node;
        }
        node = node->next;
    }
    return NULL;
}


FILE* open_logger(const char *path) {
    if (path == NULL) {
        return NULL;
    }
    if(strlen(LOG_PATH) == 0){
        //get current time
        time_t t = time(NULL);
        struct tm *tm = localtime(&t);
        strftime(LOG_PATH, sizeof(LOG_PATH), "./log-%Y-%m-%d-%H-%M-%S/", tm);
        mkdir(LOG_PATH,ACCESSPERMS);
    }


    FILE *logger = NULL;
    //check if the file is already opened
    struct LogNode *node = get_log(path);
    if (node != NULL) {
        logger = node->logger;
    } else {
        char file_path[256] = {0};
        strcpy(file_path, LOG_PATH);
        logger = fopen(strcat(file_path,path), "w+");
        if (logger == NULL) {
            return NULL;
        }
        insert_log(path, logger);
    }
    return logger;
}

void close_logger(const char *path) {
    if (path == NULL) {
        return;
    }
    struct LogNode *node = get_log(path);
    if (node != NULL) {
        fclose(node->logger);
        delete_log(path);
    }
}

void logger_write_message(const char *path,const char *message) {
    if (path == NULL || message == NULL) {
        return;
    }

    struct LogNode *node = get_log(path);
    if (node == NULL || node->logger == NULL) {
        printf("logger is not opened\n");
        return;
    }
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = localtime(&tv.tv_sec);
    char time_str[32] = {0};
    strftime(time_str, 32, "%Y-%m-%d %H:%M:%S", tm);
    fprintf(node->logger, "[%s.%06ld] %s\n", time_str, tv.tv_usec, message);
    fflush(node->logger);
}




void logger_write_bytes(const char *file_name, const uint8_t *buffer, int n) {
    if (file_name == NULL || buffer == NULL) {
        return;
    }
    struct LogNode *node = get_log(file_name);
    if (node == NULL || node->logger == NULL) {
        printf("logger is not opened\n");
        return;
    }
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = localtime(&tv.tv_sec);
    char time_str[32] = {0};
    strftime(time_str, 32, "%Y-%m-%d %H:%M:%S", tm);

    char* bytes = malloc(3*n + 1);
    for (int i = 0; i < n; ++i)
    {
        if (i == n - 1)
        {
            snprintf(bytes+3*i,4,"%02X\n",(uint8_t) buffer[i]);
        }
        else
        {
            snprintf(bytes+3*i, 4,"%02X ", (uint8_t) buffer[i]);
        }
    }
    fprintf(node->logger, "[%s.%06ld]:\n%s\n", time_str, tv.tv_usec, bytes);
    fflush(node->logger);
    free(bytes);

}

void logger_print_bytes( const uint8_t *buffer, int n){
    char* bytes = malloc(3*n + 1);
    for (int i = 0; i < n; ++i)
    {
        if (i == n - 1)
        {
            snprintf(bytes+3*i,4,"%02X\n",(uint8_t) buffer[i]);
        }
        else
        {
            snprintf(bytes+3*i, 4,"%02X ", (uint8_t) buffer[i]);
        }
    }
    bytes[3*n] = '\0';
    printf("%s",bytes);
    free(bytes);

}

void logger_info(const char *__restrict _format, ...){
    if(_format == NULL){
        return;
    }
    char buffer[MAX_LOG_SIZE];
    va_list args;
    va_start(args, _format);//初始化变长参数
    int n = vsnprintf(buffer, MAX_LOG_SIZE, _format, args);
    va_end(args);
    if(n < 0){
        return;
    }
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = localtime(&tv.tv_sec);
    char time_str[32] = {0};
    strftime(time_str, 32, "%Y-%m-%d %H:%M:%S", tm);
    printf("[%s] %s", time_str, buffer);
}


void logger_warn(const char *__restrict _format, ...) {
    if(_format == NULL){
        return;
    }
    char buffer[MAX_LOG_SIZE];
    va_list args;
    va_start(args, _format);//初始化变长参数
    int n = vsnprintf(buffer, MAX_LOG_SIZE, _format, args);
    va_end(args);
    if(n < 0){
        return;
    }
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = localtime(&tv.tv_sec);
    char time_str[32] = {0};
    strftime(time_str, 32, "%Y-%m-%d %H:%M:%S", tm);
    //set yellow color
    printf("\033[1;33m[%s] %s\033[0m", time_str, buffer);
}


void logger_error(const char *__restrict _format, ...) {
    if(_format == NULL){
        return;
    }
    char buffer[MAX_LOG_SIZE];
    va_list args;
    va_start(args, _format);//初始化变长参数
    int n = vsnprintf(buffer, MAX_LOG_SIZE, _format, args);
    va_end(args);
    if(n < 0){
        return;
    }
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = localtime(&tv.tv_sec);
    char time_str[32] = {0};
    strftime(time_str, 32, "%Y-%m-%d %H:%M:%S", tm);
    //set red color
    printf("\033[1;31m[%s] %s\033[0m", time_str, buffer);
}


void logger_important(const char *__restrict _format, ...) {
    if(_format == NULL){
        return;
    }
    char buffer[MAX_LOG_SIZE];
    va_list args;
    va_start(args, _format);//初始化变长参数
    int n = vsnprintf(buffer, MAX_LOG_SIZE, _format, args);
    va_end(args);
    if(n < 0){
        return;
    }
    // Get current time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm = localtime(&tv.tv_sec);
    char time_str[32] = {0};
    strftime(time_str, 32, "%Y-%m-%d %H:%M:%S", tm);
    //set green color and bold
    printf("\033[1;32m[%s] %s\033[0m", time_str, buffer);



}
