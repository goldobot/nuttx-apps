#ifndef __GOLDO_POLYLINE_H__
#define __GOLDO_POLYLINE_H__
#include "goldo_config.h"

/* Polyline segment. The formula is y = c0 + c1 * x + c2 * x^2 + c3 * x^3 */
typedef struct goldo_polyline_pt_s
{
  float x;
  float c0;                                                                                                                                 ;
  float c1;
  float c2;
  float c3;  
} goldo_polyline_pt_s;

typedef struct goldo_polyline_s
{
    goldo_polyline_pt_s* points;
    int num_points;
    float x_end; /*The abscissa goes from points[0].x to x_end */
    float y_end;
} goldo_polyline_s;

int goldo_polyline_init(goldo_polyline_s* p, goldo_polyline_pt_s* points, int num_points);
bool goldo_polyline_sample(goldo_polyline_s* p, float x, float* val, float* deriv, int* hint);/* hint is a pointer to the index of current segment*/
int goldo_polyline_gen_trapezoidal_1(goldo_polyline_s* p, float x1, float y1, float y2, float speed, float accel, float deccel,float t_stabilize);

#endif /* __GOLDO_POLYLINE_H_ */