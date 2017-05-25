#ifndef __GOLDO_COMMAND_FIFO_H__
#define __GOLDO_COMMAND_FIFO_H__
#include <pthread.h>

typedef struct goldo_command_fifo_s
{
  int fifo_size;
  int item_size;
  int index;
  int end;
  char* buffer;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  int threads_waiting;
  bool destroy;
} goldo_command_fifo_s;

int goldo_command_fifo_init(goldo_command_fifo_s* f,void* buffer, int item_size, int num_items);
void goldo_command_fifo_clear(goldo_command_fifo_s* f);
void* goldo_command_fifo_current(goldo_command_fifo_s* f);
void* goldo_command_fifo_next(goldo_command_fifo_s* f);
void* goldo_command_fifo_wait_for_next(goldo_command_fifo_s* f);
void* goldo_command_fifo_write_begin(goldo_command_fifo_s* f);
int goldo_command_fifo_write_end(goldo_command_fifo_s* f);




#endif /* __GOLDO_COMMAND_FIFO_H__ */