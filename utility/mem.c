//
// Created by server on 2022-05-20.
//

#include "include/header.h"
#include "mem.h"
#include "log.h"


void *mem_alloc(size_t size, const char *file, int line) {
#undef malloc
    void *ptr = malloc(size);
    if (ptr == NULL) {
        logger_write_message(MEMORY_LOGGER_FILE, "mem_alloc error");
        return NULL;
    }
    char malloc_info[256] = {0};
    snprintf(malloc_info, sizeof(malloc_info),
             "[mem_alloc point %p] %zu bytes at %s:%d", ptr, size, file, line);
    logger_write_message(MEMORY_LOGGER_FILE, malloc_info);
    return ptr;
#define malloc(size) mem_alloc(size, __FILE__, __LINE__)

}

void mem_free(void *ptr, const char *file, int line) {
#undef free
    char free_info[256] = {0};
    snprintf(free_info, sizeof(free_info),
             "[mem_free point %p] at %s:%d", ptr, file, line);
    logger_write_message(MEMORY_LOGGER_FILE, free_info);
    free(ptr);
#define free(ptr) mem_free(ptr, __FILE__, __LINE__)
}

