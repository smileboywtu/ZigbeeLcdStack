// include the header
#include "queue.h"

// initQueue
void initQueue(Queue* q){
  // reset the header
  q->pHeader = NULL:
  // reset the tail
  q->pTail = NULL;
}

// push
void push(Queue* q, byte[] str){
  // new the node
  QueueNode* newNode = (QueueNode*)osal_mem_alloc(sizeof(QueueNode));
  // check new memory
  if(NULL == newNode){
    return;
  }else{
    // push the data
    newNode.data = str; // here just record the reference
    // set the tail
    newNode.pNext = q->pTail;
    // insert into the queue
    q->pTail = newNode;
    // if the queue is empty
    if(!q->pHeaderead){
      q->pHeader = newNode;
    }
  }
}

// pop
byte* pop(Queue* q){
  // check if not empty
  if(NULL != q->pHeader){
    QueueNode* freeNode = q->pHeader;
    // move the header
    q->pHeader = q->pHeader->pNext;
    // free the string first
    osal_mem_free((byte*)freeNode->data);
    // free the
    osal_mem_free((QueueNode*)freeNode);
  }
}

// is empty
byte isEmpty(Queue q){
  if(q.pHeader == q.pTail){
    return 0x01;
  }else{
    return 0x00;
  }
}
