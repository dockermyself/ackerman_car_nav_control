//
// Created by server on 2022-05-20.
//

#ifndef RTK_NAV_MEM_H
#define RTK_NAV_MEM_H

#define malloc(size) mem_alloc(size, __FILE__, __LINE__)
#define free(ptr) mem_free(ptr, __FILE__, __LINE__)
void *mem_alloc(size_t size, const char *file, int line);
void mem_free(void *ptr, const char *file, int line);

#endif //RTK_NAV_MEM_H
