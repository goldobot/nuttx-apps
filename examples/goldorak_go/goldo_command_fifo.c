#include "goldo_command_fifo.h"


int goldo_command_fifo_init(goldo_command_fifo_s* f,void* buffer, int item_size, int num_items)
{
  f->buffer = buffer;
  f->index = 0;
  f->end = 0;
  f->fifo_size = num_items;
  f->item_size = item_size;
  f->destroy = false;
  f->threads_waiting = 0;
  pthread_mutex_init(&f->mutex,NULL);
  pthread_cond_init(&f->cond,NULL);
  return OK;
}

int goldo_command_fifo_release(goldo_command_fifo_s* f)
{
  pthread_mutex_lock(&f->mutex);
  f->destroy = true;
  pthread_cond_broadcast(&f->cond);
  while(f->threads_waiting > 0)
  {
    pthread_cond_wait(&f->cond,&f->mutex);
  }  
  pthread_mutex_unlock(&f->mutex);
  pthread_mutex_destroy(&f->mutex);
  pthread_cond_destroy(&f->cond);
  return OK;
}

void goldo_command_fifo_clear(goldo_command_fifo_s* f)
{
  pthread_mutex_lock(&f->mutex);
  bool was_empty = (f->index == f->end);
  f->index = 0;
  f->end = 0;
  if(was_empty)
  {
    pthread_cond_broadcast(&f->cond);
  }
  pthread_mutex_unlock(&f->mutex);  
}

void* goldo_command_fifo_current(goldo_command_fifo_s* f)
{
  if(f->index != f->end)
  {
    return f->buffer + f->index * f->item_size;
  } else
  {
    return NULL;
  }
}
void* goldo_command_fifo_next(goldo_command_fifo_s* f)
{
  pthread_mutex_lock(&f->mutex);
  if(f->index != f->end)
  {
    f->index++;
    if(f->index==f->fifo_size)
    {
      f->index=0;
    }
    pthread_cond_broadcast(&f->cond);
  }  
  pthread_mutex_unlock(&f->mutex);
  if(f->index != f->end)
  {
    return f->buffer + f->index * f->item_size;
  } else
  {
    return NULL;
  }
}

void* goldo_command_fifo_wait_for_next(goldo_command_fifo_s* f)
{
  pthread_mutex_lock(&f->mutex);
  f->threads_waiting++;
  while(f->index == f->end && !f->destroy)
  {
    pthread_cond_wait(&f->cond,&f->mutex);
  }
  f->index++;
  if(f->index==f->fifo_size)
  {
    f->index=0;
  }
  f->threads_waiting--;
  pthread_cond_broadcast(&f->cond);  
  pthread_mutex_unlock(&f->mutex);
  if(f->index != f->end)
  {
    return f->buffer + f->index * f->item_size;
  } else
  {
    return NULL;
  }
}

void* goldo_command_fifo_write_begin(goldo_command_fifo_s* f)
{
  pthread_mutex_lock(&f->mutex);
  int next_command_index = f->end+1;
  if(next_command_index == f->fifo_size)
  {
    next_command_index = 0;
  }
  
  pthread_mutex_unlock(&f->mutex);
  if(next_command_index == f->index)
  {    
    return NULL;
  } else
  {
    return f->buffer + f->end * f->item_size;
  }  
}

int goldo_command_fifo_write_end(goldo_command_fifo_s* f)
{
  int next_command_index = f->end+1;
  if(next_command_index == f->fifo_size)
  {
    next_command_index = 0;
  }
  f->end = next_command_index;
  pthread_cond_broadcast(&f->cond);
  pthread_mutex_unlock(&f->mutex);
  return OK;
}