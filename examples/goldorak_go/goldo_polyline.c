#include "goldo_polyline.h"
#include <stdlib.h>


bool goldo_polyline_sample(goldo_polyline_s* p, float x, float* val, float* deriv, int* hint)
{
	int index = 0;
	if(hint != NULL)
  {
    index = *hint;
    if(p->points[index].x > p->x_end)
    {
      index = 0;
    }
  }
  if(x < p->points[0].x || x > p->x_end)
  {
    return false;
  }
  while(index+1 < p->num_points && p->points[index+1].x < x)
  {
    index++;
  };
  if(hint != NULL)
  {
    *hint = index;
  }
  float u=x-p->points[index].x;
  float c0 = p->points[index].c0;
  float c1 = p->points[index].c1;
  float c2 = p->points[index].c2;
  float c3 = p->points[index].c3;


  if(val != NULL)
  {
    *val = c0 + u*(c1 + u*(c2 + u*c3));
  }
  if(deriv != NULL)
  {
    *deriv = c1 + u * ( 2*c2 + 3*u*c3);
  }
  return true;  
}

int goldo_polyline_gen_trapezoidal_1(goldo_polyline_s* p, float x1, float y1, float y2, float speed, float accel, float deccel,float t_stabilize)
{
  float distance = y2 > y1 ? y2 - y1 : y1 - y2;
  float d_a = (speed * speed) * 0.5f / accel;  
  float d_d = (speed * speed) * 0.5f / deccel;
  float d_c = distance - d_a - d_d;
  float t_a = 0;
  float t_d = 0;
  float t_c = 0;
  
  while(d_c < 0)
  {
    speed *= 0.95;
    d_a = (speed * speed) * 0.5f / accel;  
    d_d = (speed * speed) * 0.5f / deccel;
    d_c = distance - d_a - d_d;
  }
  
  t_a = speed/accel;
  t_d = speed/deccel;
  t_c = (distance - d_a - d_d)/speed;

  if(y1 > y2)
  {
    speed = -speed;
    accel = -accel;
    deccel = -deccel;
    d_a = -d_a;
    d_d = -d_d;
    d_c = -d_c;
  }
  goldo_polyline_pt_s* pt = p->points;

  float cx = x1;
  float cy = y1;

  pt->x = cx;
  pt->c0=cy;
  pt->c1=0;
  pt->c2=0.5f*accel;
  pt->c3=0;

  pt++;
  cx += t_a;
  cy += d_a;

  pt->x = cx;
  pt->c0=cy;
  pt->c1=speed;
  pt->c2=0;
  pt->c3=0;

  pt++;
  cx += t_c;
  cy += d_c;

  pt->x = cx;
  pt->c0=cy;
  pt->c1=speed;
  pt->c2=-0.5f*deccel;
  pt->c3=0;

  pt++;
  cx += t_d;
  cy += d_d;

  pt->x = cx;
  pt->c0= y2;
  pt->c1=0;
  pt->c2=0;
  pt->c3=0;

  p->x_end = cx + t_d;
  p->y_end = y2;
  p->num_points = 4;

  return OK;  
}