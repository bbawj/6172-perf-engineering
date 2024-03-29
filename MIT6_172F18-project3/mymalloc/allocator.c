/**
 * Copyright (c) 2015 MIT License by 6.172 Staff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 **/

#include "./allocator_interface.h"
#include "./memlib.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Don't call libc malloc!
#define malloc(...) (USE_MY_MALLOC)
#define free(...) (USE_MY_FREE)
#define realloc(...) (USE_MY_REALLOC)

// All blocks must have a specified minimum alignment.
// The alignment requirement (from config.h) is >= 8 bytes.
#ifndef ALIGNMENT
#define ALIGNMENT 8
#endif

// Rounds up to the nearest multiple of ALIGNMENT.
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1))

typedef struct Node {
  struct Node *next;
} Node;

// The smallest aligned size that will hold a size_t value.
#define SIZE_T_SIZE (ALIGN(sizeof(size_t)))
#define NODE_SIZE (ALIGN(sizeof(Node)))
#define MIN_SIZE (ALIGN(sizeof(size_t) + sizeof(Node)))
#define SIZE_FROM_DATA(x) *(size_t *)((char *)(x)-NODE_SIZE - SIZE_T_SIZE)
#define SIZE_FROM_NODE(x) *(size_t *)((char *)(x)-SIZE_T_SIZE)
#define NODE(x) *((Node *)((char *)(x)-NODE_SIZE))
#define DATA(x) (void *)((char *)(x) + SIZE_T_SIZE + NODE_SIZE)
#define DATA_FROM_NODE(x) (void *)((char *)(x) + NODE_SIZE)

struct {
  Node *head;
} free_list;

char *head = NULL;

// check - This checks our invariant that the size_t header before every
// block points to either the beginning of the next block, or the end of the
// heap.
int my_check() { return 0; }

// init - Initialize the malloc package.  Called once before any other
// calls are made.  Since this is a very simple implementation, we just
// return success.
int my_init() {
  unsigned int req = ALIGN(1024);
  head = mem_sbrk(req);
  free_list.head = NULL;
  return 0;
}

//  Always allocate a block whose size is a multiple of the alignment.
void *my_malloc(size_t size) {
  unsigned int aligned_size = ALIGN(size + SIZE_T_SIZE + NODE_SIZE);

  if ((char *)head + aligned_size <= (char *)mem_heap_hi()) {
    void *ret = DATA(head);
    head = (char *)head + aligned_size;
    SIZE_FROM_DATA(ret) = aligned_size;
    NODE(ret) = (Node){0};
    return ret;
  }

  Node *prev = NULL;
  Node *cur = free_list.head;
  while (cur != NULL) {
    size_t cur_size = SIZE_FROM_NODE(cur);
    if (cur_size == aligned_size) {
      if (prev == NULL) {
        // Allocating from head of list
        free_list.head = cur->next;
      } else {
        prev->next = cur->next;
      }
      cur->next = NULL;
      return DATA_FROM_NODE(cur);
    }

    // Coalesce the blocks if they are next to each other
    if (prev != NULL &&
        &SIZE_FROM_NODE(prev) + SIZE_FROM_NODE(prev) == &SIZE_FROM_NODE(cur)) {
      prev->next = cur->next;
      SIZE_FROM_NODE(prev) = SIZE_FROM_NODE(prev) + cur_size;
      prev = cur;
      cur = cur->next;
      cur_size = SIZE_FROM_NODE(cur);
      if (cur->next != NULL) {
        continue;
      }
    }

    if (cur_size > aligned_size) {
      void *ret = DATA_FROM_NODE(cur);
      SIZE_FROM_DATA(ret) = aligned_size;

      // Allocating from middle of free list
      if (prev != NULL) {
        // Not enough space to split
        if (cur_size - aligned_size < MIN_SIZE) {
          prev->next = cur->next;
          NODE(ret) = (Node){.next = NULL};
          return ret;
        }
        // Split the current free node into 2 free nodes
        prev->next =
            (Node *)((char *)&SIZE_FROM_DATA(ret) + aligned_size + SIZE_T_SIZE);
        prev->next->next = cur->next;
        SIZE_FROM_NODE(prev->next) = cur_size - aligned_size;
        NODE(ret) = (Node){.next = NULL};
      } else {
        // Allocating from first free node in the list
        free_list.head = &NODE(ret)->next;
        NODE(ret) = (Node){.next = NULL};
      }
      return ret;
    }
    prev = cur;
    cur = cur->next;
  }

  if ((char *)head + aligned_size > (char *)mem_heap_hi()) {
    // Not enough memory, double the heap size
    unsigned int req = ALIGN(1024 + aligned_size);
    if (mem_sbrk(req) == (void *)-1) {
      if (mem_sbrk(aligned_size) == (void *)-1) {
        return NULL;
      }
    }
  }
  void *ret = DATA(head);
  SIZE_FROM_DATA(ret) = aligned_size;
  NODE(ret) = (Node){0};
  head = (char *)head + aligned_size;
  return ret;
}

void my_free(void *ptr) {
  Node *n = &NODE(ptr);
  n->next = NULL;
  if (free_list.head == NULL) {
    free_list.head = n;
    return;
  }

  n->next = free_list.head;
  free_list.head = n;
}

// realloc - Implemented simply in terms of malloc and free
void *my_realloc(void *ptr, size_t size) {
  void *newptr;
  size_t copy_size;

  // Allocate a new chunk of memory, and fail if that allocation fails.
  newptr = my_malloc(size);
  if (NULL == newptr) {
    return NULL;
  }

  copy_size = SIZE_FROM_DATA(ptr) - SIZE_T_SIZE - NODE_SIZE;

  // If the new block is smaller than the old one, we have to stop copying
  // early so that we don't write off the end of the new block of memory.
  if (size < copy_size) {
    copy_size = size;
  }

  memcpy(newptr, ptr, copy_size);

  // Release the old block.
  my_free(ptr);

  // Return a pointer to the new block.
  return newptr;
}
