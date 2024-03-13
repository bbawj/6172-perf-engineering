/**
 * collision_world.c -- detect and handle line segment intersections
 * Copyright (c) 2012 the Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **/

#define QUADTREE
#include "./collision_world.h"
#include <assert.h>
#include <cilk/cilk.h>
#include <cilk/cilk_api.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "./intersection_detection.h"
#include "./intersection_event_list.h"
#include "./line.h"

CollisionWorld *CollisionWorld_new(const unsigned int capacity) {
  assert(capacity > 0);

  CollisionWorld *collisionWorld = malloc(sizeof(CollisionWorld));
  if (collisionWorld == NULL) {
    return NULL;
  }

  collisionWorld->numLineWallCollisions = 0;
  collisionWorld->numLineLineCollisions = 0;
  collisionWorld->timeStep = 0.5;
  collisionWorld->lines = malloc(capacity * sizeof(Line *));
  collisionWorld->numOfLines = 0;
  return collisionWorld;
}

void CollisionWorld_delete(CollisionWorld *collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    free(collisionWorld->lines[i]);
  }
  free(collisionWorld->lines);
  free(collisionWorld);
}

unsigned int CollisionWorld_getNumOfLines(CollisionWorld *collisionWorld) {
  return collisionWorld->numOfLines;
}

void CollisionWorld_addLine(CollisionWorld *collisionWorld, Line *line) {
  collisionWorld->lines[collisionWorld->numOfLines] = line;
  collisionWorld->numOfLines++;
}

Line *CollisionWorld_getLine(CollisionWorld *collisionWorld,
                             const unsigned int index) {
  if (index >= collisionWorld->numOfLines) {
    return NULL;
  }
  return collisionWorld->lines[index];
}

void CollisionWorld_updateLines(CollisionWorld *collisionWorld) {
#ifdef QUADTREE
  CollisionWorld_detectIntersection_new(collisionWorld);
#else
  CollisionWorld_detectIntersection(collisionWorld);
#endif
  CollisionWorld_updatePosition(collisionWorld);
  CollisionWorld_lineWallCollision(collisionWorld);
}

void CollisionWorld_updatePosition(CollisionWorld *collisionWorld) {
  double t = collisionWorld->timeStep;
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    line->p1 = Vec_add(line->p1, Vec_multiply(line->velocity, t));
    line->p2 = Vec_add(line->p2, Vec_multiply(line->velocity, t));
  }
}

void CollisionWorld_lineWallCollision(CollisionWorld *collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    bool collide = false;

    // Right side
    if ((line->p1.x > BOX_XMAX || line->p2.x > BOX_XMAX) &&
        (line->velocity.x > 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Left side
    if ((line->p1.x < BOX_XMIN || line->p2.x < BOX_XMIN) &&
        (line->velocity.x < 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Top side
    if ((line->p1.y > BOX_YMAX || line->p2.y > BOX_YMAX) &&
        (line->velocity.y > 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Bottom side
    if ((line->p1.y < BOX_YMIN || line->p2.y < BOX_YMIN) &&
        (line->velocity.y < 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Update total number of collisions.
    if (collide == true) {
      collisionWorld->numLineWallCollisions++;
    }
  }
}
/**
 *Number of frames = 4000
Input file path is: input/mit.in
---- RESULTS ----
Elapsed execution time: 49.269134s
1262 Line-Wall Collisions
19806 Line-Line Collisions
---- END RESULTS ----
./screensaver 4000  49.27s user 0.00s system 99% cpu 49.281 total

 *
*/
void CollisionWorld_detectIntersection(CollisionWorld *collisionWorld) {
  IntersectionEventList intersectionEventList = IntersectionEventList_make();

  // Test all line-line pairs to see if they will intersect before the
  // next time step.
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *l1 = collisionWorld->lines[i];

    for (int j = i + 1; j < collisionWorld->numOfLines; j++) {
      Line *l2 = collisionWorld->lines[j];

      // intersect expects compareLines(l1, l2) < 0 to be true.
      // Swap l1 and l2, if necessary.
      if (compareLines(l1, l2) >= 0) {
        Line *temp = l1;
        l1 = l2;
        l2 = temp;
      }

      IntersectionType intersectionType =
          intersect(l1, l2, collisionWorld->timeStep);
      if (intersectionType != NO_INTERSECTION) {
        IntersectionEventList_appendNode(&intersectionEventList, l1, l2,
                                         intersectionType);
        collisionWorld->numLineLineCollisions++;
      }
    }
  }

  // Sort the intersection event list.
  IntersectionEventNode *startNode = intersectionEventList.head;
  while (startNode != NULL) {
    IntersectionEventNode *minNode = startNode;
    IntersectionEventNode *curNode = startNode->next;
    while (curNode != NULL) {
      if (IntersectionEventNode_compareData(curNode, minNode) < 0) {
        minNode = curNode;
      }
      curNode = curNode->next;
    }
    if (minNode != startNode) {
      IntersectionEventNode_swapData(minNode, startNode);
    }
    startNode = startNode->next;
  }

  // Call the collision solver for each intersection event.
  IntersectionEventNode *curNode = intersectionEventList.head;

  while (curNode != NULL) {
    CollisionWorld_collisionSolver(collisionWorld, curNode->l1, curNode->l2,
                                   curNode->intersectionType);
    curNode = curNode->next;
  }

  IntersectionEventList_deleteNodes(&intersectionEventList);
}

unsigned int
CollisionWorld_getNumLineWallCollisions(CollisionWorld *collisionWorld) {
  return collisionWorld->numLineWallCollisions;
}

unsigned int
CollisionWorld_getNumLineLineCollisions(CollisionWorld *collisionWorld) {
  return collisionWorld->numLineLineCollisions;
}

void CollisionWorld_collisionSolver(CollisionWorld *collisionWorld, Line *l1,
                                    Line *l2,
                                    IntersectionType intersectionType) {
  assert(compareLines(l1, l2) < 0);
  assert(intersectionType == L1_WITH_L2 || intersectionType == L2_WITH_L1 ||
         intersectionType == ALREADY_INTERSECTED);

  // Despite our efforts to determine whether lines will intersect ahead
  // of time (and to modify their velocities appropriately), our
  // simplified model can sometimes cause lines to intersect.  In such a
  // case, we compute velocities so that the two lines can get unstuck in
  // the fastest possible way, while still conserving momentum and kinetic
  // energy.
  if (intersectionType == ALREADY_INTERSECTED) {
    Vec p = getIntersectionPoint(l1->p1, l1->p2, l2->p1, l2->p2);

    if (Vec_length(Vec_subtract(l1->p1, p)) <
        Vec_length(Vec_subtract(l1->p2, p))) {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p2, p)),
                                  Vec_length(l1->velocity));
    } else {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p1, p)),
                                  Vec_length(l1->velocity));
    }
    if (Vec_length(Vec_subtract(l2->p1, p)) <
        Vec_length(Vec_subtract(l2->p2, p))) {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p2, p)),
                                  Vec_length(l2->velocity));
    } else {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p1, p)),
                                  Vec_length(l2->velocity));
    }
    return;
  }

  // Compute the collision face/normal vectors.
  Vec face;
  Vec normal;
  if (intersectionType == L1_WITH_L2) {
    Vec v = Vec_makeFromLine(*l2);
    face = Vec_normalize(v);
  } else {
    Vec v = Vec_makeFromLine(*l1);
    face = Vec_normalize(v);
  }
  normal = Vec_orthogonal(face);

  // Obtain each line's velocity components with respect to the collision
  // face/normal vectors.
  double v1Face = Vec_dotProduct(l1->velocity, face);
  double v2Face = Vec_dotProduct(l2->velocity, face);
  double v1Normal = Vec_dotProduct(l1->velocity, normal);
  double v2Normal = Vec_dotProduct(l2->velocity, normal);

  // Compute the mass of each line (we simply use its length).
  double m1 = Vec_length(Vec_subtract(l1->p1, l1->p2));
  double m2 = Vec_length(Vec_subtract(l2->p1, l2->p2));

  // Perform the collision calculation (computes the new velocities along
  // the direction normal to the collision face such that momentum and
  // kinetic energy are conserved).
  double newV1Normal =
      ((m1 - m2) / (m1 + m2)) * v1Normal + (2 * m2 / (m1 + m2)) * v2Normal;
  double newV2Normal =
      (2 * m1 / (m1 + m2)) * v1Normal + ((m2 - m1) / (m2 + m1)) * v2Normal;

  // Combine the resulting velocities.
  l1->velocity =
      Vec_add(Vec_multiply(normal, newV1Normal), Vec_multiply(face, v1Face));
  l2->velocity =
      Vec_add(Vec_multiply(normal, newV2Normal), Vec_multiply(face, v2Face));

  return;
}

void deinit_node(Node *n) {
  if (n->children != NULL) {
    for (int i = 0; i < 4; ++i) {
      deinit_node(n->children[i]);
      free(n->children[i]);
    }
  }
  free(n->lines);
}

void destroy_quad_tree(QuadTree *q) {
  deinit_node(q->root);
  free(q->leaves);
}

Lines *init_lines(void) {
  Lines *l = malloc(sizeof(Lines));
  l->len = 0;
  l->cap = 256;
  l->lines = malloc(l->cap * sizeof(Line *));
  return l;
}

QuadTree build_quadtree(CollisionWorld *collisionWorld) {
  QuadTree qt = {0};
  qt.root = malloc(sizeof(Node));
  qt.root->parent = NULL;
  qt.root->children = NULL;
  qt.root->bl = (Vec){BOX_XMIN, BOX_YMIN};
  qt.root->tl = (Vec){BOX_XMIN, BOX_YMAX};
  qt.root->br = (Vec){BOX_XMAX, BOX_YMIN};
  qt.root->tr = (Vec){BOX_XMAX, BOX_YMAX};

  qt.leaves = malloc(sizeof(NodeQueue));
  qt.leaves->size = 0;
  qt.leaves->cap = 256;
  qt.leaves->nodes = malloc(sizeof(Node *) * qt.leaves->cap);

  Lines *root_line = malloc(sizeof(Lines));
  root_line->lines = collisionWorld->lines;
  root_line->cap = collisionWorld->numOfLines;
  root_line->len = collisionWorld->numOfLines;
  qt.root->lines = root_line;

  int r = 3;
  NodeQueue q = {0};
  q.size = 0;
  q.cap = 256;
  q.nodes = malloc(sizeof(Node *) * q.cap);
  push(&q, qt.root);

  while (q.size > 0) {
    Node *n = pop(&q);
    if (n->lines != NULL && n->lines->len > r) {
      n->children = malloc(sizeof(Node *) * 4);
      double mid_y = (n->bl.y + n->tl.y) / 2;
      double mid_x = (n->bl.x + n->br.x) / 2;

      Vec mid_left = (Vec){n->bl.x, mid_y};
      Vec mid_right = (Vec){n->br.x, mid_y};
      Vec mid_bot = (Vec){mid_x, n->bl.y};
      Vec mid_top = (Vec){mid_x, n->tl.y};
      Vec mid = (Vec){mid_x, mid_y};
      n->children[0] = malloc(sizeof(Node));
      n->children[0]->parent = n;
      n->children[0]->children = NULL;
      n->children[0]->lines = init_lines();
      n->children[0]->bl = n->bl;
      n->children[0]->tl = mid_left;
      n->children[0]->br = mid_bot;
      n->children[0]->tr = mid;
      n->children[0]->tested = false;

      n->children[1] = malloc(sizeof(Node));
      n->children[1]->parent = n;
      n->children[1]->children = NULL;
      n->children[1]->lines = init_lines();
      n->children[1]->bl = mid_left;
      n->children[1]->tl = n->tl;
      n->children[1]->br = mid;
      n->children[1]->tr = mid_top;
      n->children[1]->tested = false;

      n->children[2] = malloc(sizeof(Node));
      n->children[2]->parent = n;
      n->children[2]->children = NULL;
      n->children[2]->lines = init_lines();
      n->children[2]->bl = mid_bot;
      n->children[2]->tl = mid;
      n->children[2]->br = n->br;
      n->children[2]->tr = mid_right;
      n->children[2]->tested = false;

      n->children[3] = malloc(sizeof(Node));
      n->children[3]->parent = n;
      n->children[3]->children = NULL;
      n->children[3]->lines = init_lines();
      n->children[3]->bl = mid;
      n->children[3]->tl = mid_top;
      n->children[3]->br = mid_right;
      n->children[3]->tr = n->tr;
      n->children[3]->tested = false;

      push(&q, n->children[0]);
      push(&q, n->children[1]);
      push(&q, n->children[2]);
      push(&q, n->children[3]);

      double time = collisionWorld->timeStep;
      Lines *not_fit = NULL;
      for (int i = 0; i < n->lines->len; i++) {
        Line *l1 = n->lines->lines[i];
        Vec velocity = l1->velocity;
        // Get the parallelogram.
        Vec p1 = Vec_add(l1->p1, Vec_multiply(velocity, time));
        Vec p2 = Vec_add(l1->p2, Vec_multiply(velocity, time));
        int fit = 0;
        for (int j = 0; j < 4; ++j) {
          Node *c = n->children[j];
          if ((l1->p1.x > c->bl.x && l1->p1.x < c->br.x && l1->p1.y > c->bl.y &&
               l1->p1.y < c->tl.y) &&
              (l1->p2.x > c->bl.x && l1->p2.x < c->br.x && l1->p2.y > c->bl.y &&
               l1->p2.y < c->tl.y) &&
              (p1.x > c->bl.x && p1.x < c->br.x && p1.y > c->bl.y &&
               p1.y < c->tl.y) &&
              (p2.x > c->bl.x && p2.x < c->br.x && p2.y > c->bl.y &&
               p2.y < c->tl.y)) {
            l1->quad_tree_node = c;
            add_line(c->lines, l1);
            fit = 1;
            break;
          }
        }

        // Could not fit the parallelogram into any child quads
        if (!fit) {
          if (not_fit == NULL) {
            not_fit = init_lines();
          }
          add_line(not_fit, l1);
        }
      }
      if (n != qt.root) {
        deinit_lines(n->lines);
      }
      // Lines that cannot fit go back into the parent
      n->lines = not_fit;
    } else {
      // Leaf node
      push(qt.leaves, n);
    }
  }
  free(q.nodes);
  return qt;
}

Lines *merge_lines(Lines *l1, Lines *l2) {
  if (l1 == NULL && l2 == NULL)
    return NULL;
  Lines *ret = malloc(sizeof(Lines));
  assert(ret);
  if (l2 == NULL || l2->lines == NULL) {
    ret->len = l1->len;
    ret->cap = l1->cap;
    ret->lines = malloc(ret->cap * sizeof(Line *));
    assert(ret->lines);
    memcpy(ret->lines, l1->lines, sizeof(Line *) * l1->len);
  } else if (l1 == NULL || l1->lines == NULL) {
    ret->len = l2->len;
    ret->cap = l2->cap;
    ret->lines = malloc(ret->cap * sizeof(Line *));
    assert(ret->lines);
    memcpy(ret->lines, l2->lines, sizeof(Line *) * l2->len);
  } else {
    ret->len = l1->len + l2->len;
    ret->cap = l1->len + l2->len;
    ret->lines = malloc(ret->cap * sizeof(Line *));
    assert(ret->lines);
    memcpy(ret->lines, l1->lines, sizeof(Line *) * l1->len);
    memcpy(&ret->lines[l1->len], l2->lines, sizeof(Line *) * l2->len);
  }
  return ret;
}

void new_list(void *view) {
  *(IntersectionEventList *)view =
      (IntersectionEventList){.head = NULL, .tail = NULL};
}

void list_reduce(void *left, void *right) {
  if (((IntersectionEventList *)left)->head == NULL) {
    ((IntersectionEventList *)left)->head =
        ((IntersectionEventList *)right)->head;
  } else if (((IntersectionEventList *)left)->tail == NULL) {
    ((IntersectionEventList *)left)->tail =
        ((IntersectionEventList *)right)->head;
  } else {
    ((IntersectionEventList *)left)->tail->next =
        ((IntersectionEventList *)right)->head;
  }
  ((IntersectionEventList *)left)->tail =
      ((IntersectionEventList *)right)->tail;
}

IntersectionEventList cilk_reducer(new_list, list_reduce)
    intersectionEventList = (IntersectionEventList){.head = NULL, .tail = NULL};

void check_collision(CollisionWorld *collisionWorld, Node *n, Lines *prev) {
  if (n == NULL)
    return;
  // Test lines within node itself
  if (n->lines != NULL) {
    for (int i = 0; i < n->lines->len; ++i) {
      Line *l1 = n->lines->lines[i];
      for (int j = i + 1; j < n->lines->len; ++j) {
        Line *l2 = n->lines->lines[j];
        IntersectionType intersectionType;
        // intersect expects compareLines(l1, l2) < 0 to be true.
        // Swap l1 and l2, if necessary.
        if (compareLines(l1, l2) >= 0) {
          intersectionType = intersect(l2, l1, collisionWorld->timeStep);
          if (intersectionType != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&intersectionEventList, l2, l1,
                                             intersectionType);
            collisionWorld->numLineLineCollisions++;
          }
        } else {
          intersectionType = intersect(l1, l2, collisionWorld->timeStep);
          if (intersectionType != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&intersectionEventList, l1, l2,
                                             intersectionType);
            collisionWorld->numLineLineCollisions++;
          }
        }
      }
    }
    if (prev != NULL && prev->lines != NULL) {
      // Test previous lines against new lines
      for (int i = 0; i < prev->len; ++i) {
        Line *l1 = prev->lines[i];
        for (int j = 0; j < n->lines->len; ++j) {
          Line *l2 = n->lines->lines[j];
          IntersectionType intersectionType;
          // intersect expects compareLines(l1, l2) < 0 to be true.
          // Swap l1 and l2, if necessary.
          if (compareLines(l1, l2) >= 0) {
            intersectionType = intersect(l2, l1, collisionWorld->timeStep);
            if (intersectionType != NO_INTERSECTION) {
              IntersectionEventList_appendNode(&intersectionEventList, l2, l1,
                                               intersectionType);
              collisionWorld->numLineLineCollisions++;
            }
          } else {
            intersectionType = intersect(l1, l2, collisionWorld->timeStep);
            if (intersectionType != NO_INTERSECTION) {
              IntersectionEventList_appendNode(&intersectionEventList, l1, l2,
                                               intersectionType);
              collisionWorld->numLineLineCollisions++;
            }
          }
        }
      }
    }
  }
  Lines *new[] = {merge_lines(prev, n->lines), merge_lines(prev, n->lines),
                  merge_lines(prev, n->lines), merge_lines(prev, n->lines)};
  cilk_scope {
    if (n->children != NULL) {
      for (int i = 0; i < 4; ++i) {
        cilk_spawn check_collision(collisionWorld, n->children[i], new[i]);
        // check_collision(collisionWorld, n->children[i], new[i]);
      }
    }
  }

  for (int i = 0; i < 4; ++i) {
    deinit_lines(new[i]);
  }
}

void CollisionWorld_detectIntersection_new(CollisionWorld *collisionWorld) {
  QuadTree q = build_quadtree(collisionWorld);
  check_collision(collisionWorld, q.root, NULL);
  // Sort the intersection event list.
  IntersectionEventNode *startNode = intersectionEventList.head;
  while (startNode != NULL) {
    IntersectionEventNode *minNode = startNode;
    IntersectionEventNode *curNode = startNode->next;
    while (curNode != NULL) {
      if (IntersectionEventNode_compareData(curNode, minNode) < 0) {
        minNode = curNode;
      }
      curNode = curNode->next;
    }
    if (minNode != startNode) {
      IntersectionEventNode_swapData(minNode, startNode);
    }
    startNode = startNode->next;
  }

  // Call the collision solver for each intersection event.
  IntersectionEventNode *curNode = intersectionEventList.head;

  while (curNode != NULL) {
    CollisionWorld_collisionSolver(collisionWorld, curNode->l1, curNode->l2,
                                   curNode->intersectionType);
    curNode = curNode->next;
  }

  IntersectionEventList_deleteNodes(&intersectionEventList);
  destroy_quad_tree(&q);
}
