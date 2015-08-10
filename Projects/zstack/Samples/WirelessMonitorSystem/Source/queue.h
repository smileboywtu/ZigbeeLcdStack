#ifndef __ZSTACK_QUEUE__
#define __ZSTACK_QUEUE__

// include the header
#include "stdio.h"
#include "stdlib.h"

// define the byte here
typedef unsigned char byte;

/////////////////////////////////////////////
// Queue Data Struct

// Queue node
struct QueueNode{
  byte* data;
  QueueNode* pNext;
};

// Queue handler
struct Queue{
  QueueNode* pHeader;
  QueueNode* pTail;
};

/////////////////////////////////////////////
// Queue Interface

void initQueue(Queue* q);   // initQueue
void push(Queue* q, byte[] str); // push
byte* pop(Queue* q); // pop
byte isEmpty(Queue q);  // empty

#endif
