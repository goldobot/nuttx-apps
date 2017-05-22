//-*-C++-*-

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <semaphore.h>
#include <pthread.h>
#include <signal.h>
#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <string.h>

#include <nuttx/semaphore.h>
#include <nuttx/timers/timer.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/sensors/qencoder.h>

#include "goldorak_go.h"


extern void goldo_maxon2_dir_p(void);
extern void goldo_maxon2_dir_n(void);
extern void goldo_maxon2_en(void);
extern void goldo_maxon2_dis(void);
extern void goldo_maxon2_speed(int32_t s);
extern void goldo_maxon1_dir_p(void);
extern void goldo_maxon1_dir_n(void);
extern void goldo_maxon1_en(void);
extern void goldo_maxon1_dis(void);
extern void goldo_maxon1_speed(int32_t s);


#define ROBOT_CMD_TYPE_NONE         0
#define ROBOT_CMD_TYPE_TRANSLATION  1
#define ROBOT_CMD_TYPE_ROTATION     2
#define ROBOT_CMD_TYPE_STATIC       3

#define ROBOT_I2C_CONF_TRANSLATION  (ROBOT_I2C_CMD_SET_TRAJ_T|ROBOT_CMD_TYPE_TRANSLATION)
#define ROBOT_I2C_CONF_ROTATION     (ROBOT_I2C_CMD_SET_TRAJ_T|ROBOT_CMD_TYPE_ROTATION)
#define ROBOT_I2C_CONF_STATIC       (ROBOT_I2C_CMD_SET_TRAJ_T|ROBOT_CMD_TYPE_STATIC)


#define SEUIL_FRICTION_BRUSHLESS_1P 14600 // 15720 16260 15780 16020 // 15720
#define SEUIL_FRICTION_BRUSHLESS_1N 14500 // 15720 16440 16080 17340 // 16395
#define SEUIL_FRICTION_BRUSHLESS_2P 17000 // 14700 15540 14940 15660 // 15210
#define SEUIL_FRICTION_BRUSHLESS_2N 14500 // 15000 15660 16020 16080 // 15690
#define SEUIL_PROTECT_BRUSHLESS 60000


extern int asserv_todo_dist;

/*
  Les constantes de conversion mm<->increment sont representees en virgule fixe
  avec 12 bit de precision
*/
//#define ROBOT_MM_PER_INC 0.07832980203364314775
#define ROBOT_MM_PER_INC_FP12 320
//#define ROBOT_INC_PER_MM 12.76653296749676
#define ROBOT_INC_PER_MM_FP12 52291


#if 0 /* FIXME : TODO ++ */
/* Table de sin precalculee:
  (ROBOT_SIN_TABLE_SIZE-1)*64=ROBOT_INC_05PI_ROT_M 
  sin_table[0] correspond a sin(0);
  sin_table[ROBOT_SIN_TABLE_SIZE] correspond a sin(ROBOT_INC_05PI_ROT_M);
  Les valeurs de la table contiennent les valeurs de sin(x) sur l'intervalle
  x=[0 pi/2] representees avec 16 bits de decimale binaire (virgule fixe)
*/
#define ROBOT_INC_05PI_ROT_M   9600
#define ROBOT_SIN_TABLE_SIZE   151

extern int sin_table[];

/* 
  ROBOT_INC_05PI_ROT = nb. d'incr. correspondant a une rot de pi/2 du robot
  Les constantes de conversion mm<->increment sont representees en virgule fixe
  avec 12 bit de precision

  Les constantes de conversion rad<->increment sont representees en virgule fixe
  avec 16 bit de precision
*/
#define ROBOT_INC_05PI_ROT     3785

#define ROBOT_INC_2PI_ROT      4*(ROBOT_INC_05PI_ROT)
#define ROBOT_INC_PI_ROT       2*(ROBOT_INC_05PI_ROT)

//#define ROBOT_RAD_PER_INC 0.00041695446558875915
#define ROBOT_RAD_PER_INC_FP16 27
//#define ROBOT_INC_PER_RAD 2397.51
#define ROBOT_INC_PER_RAD_FP16 157177834

int robot_sin_prim (int x)
{
  int i,j,k;
  int sin_val, s0, s1;
  int sign;

  if ((x<0) || (x>ROBOT_INC_2PI_ROT)) {
    return 0;
  }

  if (x>(3*(ROBOT_INC_05PI_ROT))) {
    i = ROBOT_INC_2PI_ROT-x;
    sign = -1;
  } else if (x>(2*(ROBOT_INC_05PI_ROT))) {
    i = x-ROBOT_INC_PI_ROT;
    sign = -1;
  } else if (x>(1*(ROBOT_INC_05PI_ROT))) {
    i = ROBOT_INC_PI_ROT-x;
    sign = 1;
  } else {
    i = x;
    sign = 1;
  }

  i = i*ROBOT_INC_05PI_ROT_M/ROBOT_INC_05PI_ROT;
  {
    j=i>>6;
    k=i-(j<<6);
    s0=sin_table[j];
    s1=sin_table[j+1];
    sin_val = s0 + (((s1-s0)*k)>>6);
    if ((i&3)!=0) sin_val++;
  }

  return sign*sin_val;
}

int robot_normalise_rot (int x)
{
  int i;

  i=x;
  if (i<0) {
    while (i<0) i+=ROBOT_INC_2PI_ROT;
  } else {
    while (i>=(ROBOT_INC_2PI_ROT)) i-=ROBOT_INC_2PI_ROT;
  }

  return i;
}

int robot_sin (int x)
{
  int i;

  i=robot_normalise_rot (x);

  return robot_sin_prim(i);
}

int robot_cos (int x)
{
  int i;

  i=robot_normalise_rot (x+ROBOT_INC_05PI_ROT);

  return robot_sin_prim(i);
}
#endif /* FIXME : TODO -- */


/* Parametres d'initialisation pour l'automate generateur de trajectoire
   et definition des prametres geometriques du segment elementaire de 
   trajectoire a executer (longueur, angle de rotation). 
   Les elements de trajectoire peuvent etre de 2 types : 
    -1) ligne (idealement) droite;
    -2) rotation (pivotement) autour de l'axe central (vertical) du robot; */
/* NOTE : "D" represente l'abscisse curviligne du segment de
          trajectoire elementaire execute par l'automate d'asservissement. 
          On le caracterise comme "lineaire" par facilite (et abus..) de 
          langage. (Dans le cas ideal il s'agit effectivement d'un segment
          rectiligne) */
int cmd_type       =      0; /* type de trajectoire: 1=translation, 2=rotation*/
int cmd_final_D    =      0; /* distance a parcourir (translation) */
int cmd_final_Th   =      0; /* "distance" angulaire a parcourir   */
                             /* (commande de rotation)             */

int cmd_init_rmb_1 =    280; /* init offset commande moteur 1      */
int cmd_init_rmb_2 =    200; /* init offset commande moteur 2      */
int cmd_max_d2D    =      1; /* seuil d'acceleration lineaire      */
int cmd_max_d2Th   =      1; /* seuil d'acceleration angulaire     */
int cmd_max_dD     =    250; /* seuil de vitesse lineaire          */
int cmd_max_dTh    =    250; /* seuil de vitesse angulaire         */

/* variables d'etat pour l'automate generateur de trajectoire et consigne */
int cmd_fake_dD    =      0;
int cmd_fake_dTh   =      0;
int cmd_req_rmb_1  =      0; /* offset commande moteur 1           */
int cmd_req_rmb_2  =      0; /* offset commande moteur 2           */
int cmd_req_d2D    =      0; /* acceleration lineaire              */
int cmd_req_d2Th   =      0; /* acceleration angulaire             */
int cmd_req_dD     =      0; /* consigne de vitesse lineaire       */
int cmd_req_dTh    =      0; /* consigne de vitesse angulaire      */
int cmd_req_D      =      0; /* consigne de position (translation) */
int cmd_req_Th     =      0; /* consigne d'angle (rotation)        */

void cmd_gen_set_params (int _init_rmb_1, int _init_rmb_2, 
                         int _max_d2D, int _max_d2Th, 
                         int _max_dD, int _max_dTh)
{
  cmd_init_rmb_1 = _init_rmb_1;
  cmd_init_rmb_2 = _init_rmb_2;
  cmd_max_d2D    = _max_d2D;
  cmd_max_d2Th   = _max_d2Th;
  cmd_max_dD     = _max_dD;
  cmd_max_dTh    = _max_dTh;
}

int cmd_get_duration (int _type, int _D) /* in microseconds */
{
  /* FIXME : TODO : quelque chose de + intelligent.. */
  return 2400.0*_D + 1000000;
}

int cmd_init_new_traj (int _type, int _D)
{
  cmd_type = _type;
  switch (cmd_type) {
  case ROBOT_CMD_TYPE_TRANSLATION:
    cmd_final_D  = _D;
    cmd_final_Th =  0;
    break;
  case ROBOT_CMD_TYPE_ROTATION:
    cmd_final_D  =  0;
    cmd_final_Th = _D;
    break;
  case ROBOT_CMD_TYPE_STATIC:
    cmd_final_D  =  0;
    cmd_final_Th =  0;
    break;
  default:
    cmd_type = ROBOT_CMD_TYPE_NONE;
  }
  cmd_fake_dD    =      0;
  cmd_fake_dTh   =      0;
  cmd_req_rmb_1  =      0;
  cmd_req_rmb_2  =      0;
  cmd_req_d2D    =      0;
  cmd_req_d2Th   =      0;
  cmd_req_dD     =      0;
  cmd_req_d2Th   =      0;
  cmd_req_D      =      0;
  cmd_req_Th     =      0;

  return cmd_get_duration (_type,  _D);
}

/* Automate generateur de trajectoire de type 1 : 
   - segment (idealement) droit de longueur "D" (en increments) */
static void cmd_gen_translation (void)
{
  if (abs(cmd_req_D)<abs(cmd_final_D/2)) {
    cmd_req_d2D = cmd_max_d2D;
    cmd_req_rmb_1 = cmd_init_rmb_1;
    cmd_req_rmb_2 = cmd_init_rmb_2;
  } else /*if (abs(cmd_req_D)<abs(cmd_final_D))*/ {
    if (cmd_final_D>0) cmd_req_d2D = -cmd_max_d2D;
    else cmd_req_d2D = cmd_max_d2D;
    cmd_req_rmb_1 = 0;
    cmd_req_rmb_2 = 0;
  }
  cmd_fake_dD = cmd_fake_dD + cmd_req_d2D;
  if (abs(cmd_fake_dD)>abs(cmd_max_dD)) cmd_req_dD=cmd_max_dD;
  else cmd_req_dD=cmd_fake_dD;
  if ((cmd_final_D*cmd_req_dD<0) || (abs(cmd_req_D)>=abs(cmd_final_D))) 
    cmd_req_dD=0;
  cmd_req_D = cmd_req_D + cmd_req_dD;

  cmd_req_d2Th = 0;
  cmd_req_dTh = 0;
  cmd_req_Th = 0;

  asserv_todo_dist = abs(cmd_final_D - cmd_req_D);
}

/* Automate generateur de trajectoire de type 2 : 
    - rotation d'angle "Th" (represente comme l'abscisse curviligne de l'arc 
      de cercle decrit par le bord lateral du robot en rotation et exprime en 
      increments) 
/!\ ATTENTION : pour obtenir le bon sens de rotation il faut que :
 + : cmd_final_Th>0 : pour tourner a GAUCHE (i.e. robot_theta AUGMENTE)
 - : cmd_final_Th<0 : pour tourner a DROITE (i.e. robot_theta DIMINUE)
*/
static void cmd_gen_rotation (void)
{
  cmd_req_d2D = 0;
  cmd_req_dD = 0;
  cmd_req_D = 0;

  if (abs(cmd_req_Th)<abs(cmd_final_Th/2)) {
    if (cmd_final_Th>0) cmd_req_d2Th = cmd_max_d2Th;
    else cmd_req_d2Th = -cmd_max_d2Th;
    cmd_req_rmb_1 = 0;
    cmd_req_rmb_2 = 0;
  } else /*if (abs(cmd_req_Th)<abs(cmd_final_Th))*/ {
    if (cmd_final_Th>0) cmd_req_d2Th = -cmd_max_d2Th;
    else cmd_req_d2Th = cmd_max_d2Th;
    cmd_req_rmb_1 = 0;
    cmd_req_rmb_2 = 0;
  }
  cmd_fake_dTh = cmd_fake_dTh + cmd_req_d2Th;
  if (abs(cmd_fake_dTh)>abs(cmd_max_dTh)) cmd_req_dTh=cmd_max_dTh;
  else cmd_req_dTh=cmd_fake_dTh;
  if ((cmd_final_Th*cmd_req_dTh<0) || (abs(cmd_req_Th)>=abs(cmd_final_Th))) 
    cmd_req_dTh=0;
  cmd_req_Th = cmd_req_Th + cmd_req_dTh;

  asserv_todo_dist = abs(cmd_final_Th - cmd_req_Th);
}

/* Automate generateur de trajectoire de type 3 : 
    - consigne en position nulle */
static void cmd_gen_static (void)
{
  cmd_req_d2D = 0;
  cmd_req_dD = 0;
  cmd_req_D = 0;

  cmd_req_d2Th = 0;
  cmd_req_dTh = 0;
  cmd_req_Th = 0;

  asserv_todo_dist = 0;
}

void asserv_cmd_gen (void)
{
  switch (cmd_type) {
  case ROBOT_CMD_TYPE_TRANSLATION:
    cmd_gen_translation ();
    break;
  case ROBOT_CMD_TYPE_ROTATION:
    cmd_gen_rotation ();
    break;
  case ROBOT_CMD_TYPE_STATIC:
    cmd_gen_static ();
    break;
  }
}

/* Parametres de l'automate d'asservissement (avec filtre PD) */
int corr_KD_D  = 15;
int corr_KD_Th = 15;
int corr_KP_D  = 30;
int corr_KP_Th = 30;

void cmd_corr_set_params (int nKD_D, int nKD_Th, int nKP_D, int nKP_Th)
{
  corr_KD_D  = nKD_D;
  corr_KD_Th = nKD_Th;
  corr_KP_D  = nKP_D;
  corr_KP_Th = nKP_Th;
}

/* Automate d'asservissement (avec filtre PD) */
void asserv_cmd_corr_pd (int rc_val_1, int rc_val_2, 
                         int speed_val_1, int speed_val_2, 
                         int *cmd_motor_1, int *cmd_motor_2)
{
  int rc_D, rc_Th, rc_dD, rc_dTh;
  int mD, mTh, m1, m2;
  int cmd_req_dTh_l, cmd_req_Th_l;

  cmd_req_dTh_l = cmd_req_dTh/2;
  cmd_req_Th_l  = cmd_req_Th/2;

  rc_D  = (rc_val_1+rc_val_2)/2;
  rc_Th = (rc_val_1-rc_val_2)/2;

  rc_dD  = (speed_val_1+speed_val_2)/2;
  rc_dTh = (speed_val_1-speed_val_2)/2;

  if ((cmd_type!=ROBOT_CMD_TYPE_TRANSLATION) || 
      (abs(cmd_req_D)<abs(cmd_final_D*7/8))) {
    mD  = corr_KP_D *(cmd_req_D   -rc_D ) + corr_KD_D *(cmd_req_dD   -rc_dD );
    mTh = corr_KP_Th*(cmd_req_Th_l-rc_Th) + corr_KD_Th*(cmd_req_dTh_l-rc_dTh);

    m1 = mD + mTh;
    m2 = mD - mTh;
  } else {
    m1  = corr_KP_D*(cmd_req_D-rc_val_1) + corr_KD_D*(cmd_req_dD-speed_val_1);
    m2  = corr_KP_D*(cmd_req_D-rc_val_2) + corr_KD_D*(cmd_req_dD-speed_val_2);
  }

  m1 += cmd_req_rmb_1*100;
  m2 += cmd_req_rmb_2*100;

  m1 = m1/100;
  m2 = m2/100;

#if 1 /* FIXME : TODO : calibration pour nucleo.. */
  m1 = m1*64;
  m2 = m2*64;
#endif

  if (m1>0) m1+=SEUIL_FRICTION_BRUSHLESS_1P;
  if (m1<0) m1-=SEUIL_FRICTION_BRUSHLESS_1N;
  if (m2>0) m2+=SEUIL_FRICTION_BRUSHLESS_2P;
  if (m2<0) m2-=SEUIL_FRICTION_BRUSHLESS_2N;

  if (m1> SEUIL_PROTECT_BRUSHLESS) m1= SEUIL_PROTECT_BRUSHLESS;
  if (m1<-SEUIL_PROTECT_BRUSHLESS) m1=-SEUIL_PROTECT_BRUSHLESS;
  if (m2> SEUIL_PROTECT_BRUSHLESS) m2= SEUIL_PROTECT_BRUSHLESS;
  if (m2<-SEUIL_PROTECT_BRUSHLESS) m2=-SEUIL_PROTECT_BRUSHLESS;

  *cmd_motor_1 = m1;
  *cmd_motor_2 = m2;
}


