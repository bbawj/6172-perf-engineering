#include "memlib.h"
#include <assert.h>
#include <stdlib.h>
#include <sys/mman.h>

#ifndef BLOCK_SIZE
#define BLOCK_SIZE 1024
#endif

void *head = NULL;

int my_init() {
  mem_init();
  head = mem_heap_lo();
  free_list.head = NULL;
  free_list.tail = NULL;
  return 0;
}

void *my_malloc(size_t size) {
  assert(size <= BLOCK_SIZE);
  if (free_list.head != NULL) {
    void *ret = free_list.head;
    if (free_list.tail == free_list.head) {
      free_list.tail = NULL;
    }
    free_list.head = free_list.head->next;
    return ret;
  }
  if (BLOCK_SIZE + head > mem_heap_hi()) {
    mem_sbrk(mem_pagesize());
  }
  void *ret = head;
  head += BLOCK_SIZE;
  return ret;
}

void *my_realloc(void *ptr, size_t size) { return ptr; }

void my_free(void *ptr) {
  if (free_list.head == NULL) {
    free_list.head = ptr;
    free_list.tail = ptr;
  } else {
    free_list.tail->next = ptr;
  }
}

void my_check() { return; }
