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
  uint8_t id;
  struct Node *next;
} Node;

// The smallest aligned size that will hold a size_t value.
#define NODE_SIZE (ALIGN(sizeof(Node)))
#define DATA_PTR(ptr) ((char *)(ptr) + NODE_SIZE)
#define BLOCK_SIZE(id) (ALIGN(8 << (id)) + NODE_SIZE)

// The maximum number of free lists is ceil(log(MAX_HEAP)) = 26
// but we can ignore the first 3 because they are less than ALIGN
#define NUM_LISTS 23
typedef Node *bfl[NUM_LISTS];

bfl BFL = {0};

static inline uint8_t compute_id(uint32_t size) {
  assert(size >= ALIGNMENT);
  int id = 32 - __builtin_clz(size - 1) - 3;
  assert(id >= 0 && id < NUM_LISTS);
  return id;
}

// check - This checks our invariant that the size_t header before every
// block points to either the beginning of the next block, or the end of the
// heap.
int my_check() {
  for (int i = 0; i < NUM_LISTS; ++i) {
    Node *n = BFL[i];
    while (n != NULL) {
      if (n->id != i) {
        printf("Block has id %d but is in list %d\n", n->id, i);
        return -1;
      }
      n = n->next;
    }
  }

  // char *lo = mem_heap_lo();
  // char *hi = mem_heap_hi() + 1;
  // size_t size = 0;
  // while (lo < hi) {
  //   Node *n = (Node *)lo;
  //   lo += BLOCK_SIZE(n->id);
  //   size += BLOCK_SIZE(n->id);
  // }
  // if (lo != hi) {
  //   printf("Blocks are not aligned size: %zu lo %p hi %p\n", size, lo, hi);
  //   return -1;
  // }
  return 0;
}

// init - Initialize the malloc package.  Called once before any other
// calls are made.  Since this is a very simple implementation, we just
// return success.
int my_init() {
  for (int i = 0; i < NUM_LISTS; ++i) {
    BFL[i] = NULL;
  }
  uint32_t initial_req = 1024;
  uint8_t id = compute_id(initial_req);
  assert(id == 7);
  void *new = mem_sbrk(BLOCK_SIZE(id));
  if (new == NULL) {
    return -1;
  }
  BFL[id] = new;
  *(Node *)BFL[id] = (Node){.id = id, .next = NULL};
  return 0;
}

void distribute_block(uint8_t id) {
  Node *head = BFL[id];
  assert(head);
  char *max_ptr = (char *)head + BLOCK_SIZE(id);
  BFL[id] = BFL[id]->next;

  for (int j = id - 1; j >= 0; --j) {
    if ((char *)head + BLOCK_SIZE(j) > max_ptr) {
      return;
    }
    *head = (Node){.id = j, .next = BFL[j]};
    BFL[j] = (Node *)head;
    // increment the head by the current block size
    head = (Node *)((char *)head + BLOCK_SIZE(j));
  }
}

//  Always allocate a block whose size is a multiple of the alignment.
void *my_malloc(size_t size) {
  if (size == 0)
    return NULL;
  uint8_t id = compute_id(size);
  if (BFL[id] != NULL) {
    Node *ret = BFL[id];
    BFL[id] = BFL[id]->next;
    ret->next = NULL;
    return DATA_PTR(ret);
  }

  for (int i = id + 1; i < NUM_LISTS; ++i) {
    if (BFL[i] == NULL)
      continue;
    distribute_block(i);
    if (BFL[id] == NULL) {
      break;
    }
    Node *ret = BFL[id];
    BFL[id] = BFL[id]->next;
    ret->next = NULL;
    return DATA_PTR(ret);
  }

  void *ret = mem_sbrk(BLOCK_SIZE(id));
  if (ret == NULL) {
    return NULL;
  }
  *(Node *)ret = (Node){.id = id, .next = NULL};
  return DATA_PTR(ret);
}

void my_free(void *ptr) {
  Node *n = (Node *)((char *)ptr - NODE_SIZE);
  int8_t id = n->id;
  n->next = BFL[id];
  BFL[id] = n;
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

  Node *n = (Node *)((char *)ptr - NODE_SIZE);
  copy_size = BLOCK_SIZE(n->id) - NODE_SIZE;

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
