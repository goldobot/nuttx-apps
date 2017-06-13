/****************************************************************************
 * examples/dynamixel/dynamixel_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * dynamixel_main
 ****************************************************************************/
#define AX12_MAX_SERVOS             18
#define AX12_BUFFER_SIZE            32

/** EEPROM AREA **/
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23
/** RAM AREA **/
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49
/** Status Return Levels **/
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2
/** Instruction Set **/
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

/** AX-S1 **/
#define AX_LEFT_IR_DATA             26
#define AX_CENTER_IR_DATA           27
#define AX_RIGHT_IR_DATA            28
#define AX_LEFT_LUMINOSITY          29
#define AX_CENTER_LUMINOSITY        30
#define AX_RIGHT_LUMINOSITY         31
#define AX_OBSTACLE_DETECTION       32
#define AX_BUZZER_INDEX             40
void ax12SetRegister(int id, int regstart, int data);
#define SetPosition(id, pos) (ax12SetRegister2(id, AX_GOAL_POSITION_L, pos))
#define GetPosition(id) (ax12GetRegister2(id, AX_PRESENT_POSITION_L, 2))
#define GetGoalPosition(id) (ax12GetRegister2(id, AX_GOAL_POSITION_L, 2))
#define GetMaxTorque(id) (ax12GetRegister2(id, AX_MAX_TORQUE_L, 2))
#define GetTorqueLimit(id) (ax12GetRegister2(id, AX_TORQUE_LIMIT_L, 2))
#define TorqueEnable(id) (ax12SetRegister(id, AX_TORQUE_ENABLE, 1))
#define TorqueDisable(id) (ax12SetRegister(id, AX_TORQUE_ENABLE, 0))

int fd = -1;

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
volatile int ax_rx_int_Pointer;
/** read back the error code for our latest packet read */
int ax12Error;
/** > 0 = success */
int ax12ReadPacket(int length){
    //unsigned long ulCounter;
    unsigned char offset, /*blength,*/ checksum, timeout;
    unsigned char volatile bcount; 
    char c; 

    offset = 0;
    timeout = 0;
    bcount = 0;
    while(bcount < length){
#if 0 /* FIXME : DEBUG : HACK GOLDO */
        ulCounter = 0;
        while((bcount + offset) == ax_rx_int_Pointer){
            if(ulCounter++ > 1000L){ // was 3000
                timeout = 1;
                break;
            }
        }
        if(timeout) break;
        ax_rx_buffer[bcount] = ax_rx_int_buffer[bcount + offset];
#else
	usleep (20); /* cette tempo correspond grosso-modo a la transmission d'1 caractere a 1Mbaud (+ un peu de marge..) */
	if(read (fd, &c, 1)!=1) break; /* si la valeur retournee par la fonction 'read()' n'est pas 1 => il n'y + rien a lire */
        ax_rx_buffer[bcount] = c;
#endif
        if((bcount == 0) && (ax_rx_buffer[0] != 0xff))
            offset++;
        else
            bcount++;
    }

    //blength = bcount;
    checksum = 0;
    for(offset=2;offset<bcount;offset++)
        checksum += ax_rx_buffer[offset];
    if((checksum%256) != 255){
        return 0;
    }else{
        return 1;
    }
}
void ax12SetRegister(int id, int regstart, int data){
    //setTX(id);
    int length = 4;
    int checksum = ~((id + length + AX_WRITE_DATA + regstart + (data&0xFF)) % 256);
    char c;
    //printf("Data sent :%d, reg :%d \n",data,regstart);
    c=0xFF;
    write(fd, &c, 1);
    c=0xFF;
    write(fd, &c, 1);
    c=id;
    write(fd, &c, 1);
    c=length; // length
    write(fd, &c, 1);
    c=AX_WRITE_DATA;
    write(fd, &c, 1);
    c=regstart;
    write(fd, &c, 1);
    c=data&0xff;
    write(fd, &c, 1);
    // checksum =
    c=checksum;
    write(fd, &c, 1);
    //setRX(id);
    //ax12ReadPacket();
}
void ax12SetRegister2(int id, int regstart, int data){
    //setTX(id);   
    int checksum = ~((id + 5 + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    char c;

    c=0xFF;
    write(fd, &c, 1);
    c=0xFF;
    write(fd, &c, 1);
    c=id;
    write(fd, &c, 1);
    c=5; // length
    write(fd, &c, 1);
    c=AX_WRITE_DATA;
    write(fd, &c, 1);
    c=regstart;
    write(fd, &c, 1);
    c=data&0xff;
    write(fd, &c, 1);
    c=(data&0xff00)>>8;
    write(fd, &c, 1);
    // checksum =
    c=checksum;
    write(fd, &c, 1);
    //setRX(id);
    //ax12ReadPacket();
}
int ax12GetRegister(int id, int regstart, int length){  
    //setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    char c;
    c=0xFF;
	write(fd, &c, 1);
    c=0xFF;
	write(fd, &c, 1);
    c=id;
	write(fd, &c, 1);
    c=4;    // length
	write(fd, &c, 1);
	c=AX_READ_DATA;
	write(fd, &c, 1);
    c=regstart;
	write(fd, &c, 1);
    c=length;
	write(fd, &c, 1);
    c=checksum;  
	write(fd, &c, 1);
    //setRX(id);    
    if(ax12ReadPacket(length + 6) > 0){
        ax12Error = ax_rx_buffer[4];
        if(length == 1)
            return ax_rx_buffer[5];
        else
            return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}
int ax12GetRegister2(int id, int regstart, int length){  
    //setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    char c;
	c=0xFF;
	write(fd, &c, 1);
	c=0xFF;
	write(fd, &c, 1);
	c=id;
	write(fd, &c, 1);
	c=4;
	write(fd, &c, 1);
	c=AX_READ_DATA;
	write(fd, &c, 1);
	c=regstart;
	write(fd, &c, 1);
#if 0 /* FIXME : DEBUG */
	c=data&0xff;
	write(fd, &c, 1);
#endif
	c=length;
	write(fd, &c, 1);
	c=checksum;
	write(fd, &c, 1);
	
 
    //setRX(id);    
    if(ax12ReadPacket(length + 6) > 0){
        ax12Error = ax_rx_buffer[4];
        if(length == 1)
            return ax_rx_buffer[5];
        else
            return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}

void ax12Action(int id){
    
  //int id = 0xFE;    // Broadcast ID
    char c;
    //setTX(id);    
    //int checksum = ~((id + 2 + AX_ACTION) % 256);
     c=0xFF;
    write(fd, &c, 1);
     c=0xFF;
    write(fd, &c, 1);
    c=id;
    write(fd, &c, 1);    //                Byte 1
    c=2;
    write(fd, &c, 1);    // length = 4
    c=AX_ACTION;
    write(fd, &c, 1);    // Byte 2
    c=0xFA;
    write(fd, &c, 1);
    //setRX(id);    // No status pack is sent with this command
    //ax12ReadPacket();
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dynamixel_main(int argc, char *argv[])
#endif
{
  int my_id = 0;
  int my_pos = 1024;

  fd = open ("/dev/ttyS1", O_RDWR);
  if (fd<0) {
    return -1;
  }

  if (argc>=3) {
    my_id = atoi (argv[1]);

    if (*argv[2] == 'e') {
      TorqueEnable(my_id);
      return 0;
    } else if (*argv[2] == 'd') {
      TorqueDisable(my_id);
      return 0;
    }

    my_pos = atoi (argv[2]);

    printf ("SET:\n");
    printf ("my_id = %d \n", my_id);
    printf ("my_pos = %d \n", my_pos);

    ax12SetRegister2(my_id, AX_MAX_TORQUE_L, 512);   
    ax12SetRegister2(my_id, AX_TORQUE_LIMIT_L, 512);   

    SetPosition (my_id, my_pos);
    //ax12Action(my_id);
  } else if (argc>=2) {
    my_id = atoi (argv[1]);
    printf ("my_id = %d \n", my_id);

    my_pos = GetPosition (my_id);
    if (my_pos < 0) {
      printf ("GetPosition() : error.");
    } else {
      printf ("GetPosition:\n");
      printf (" my_pos = %d \n", my_pos);
    }

    my_pos = GetGoalPosition (my_id);
    if (my_pos < 0) {
      printf ("GetGoalPosition() : error.");
    } else {
      printf ("GetGoalPosition:\n");
      printf (" my_pos = %d \n", my_pos);
    }

    my_pos = GetMaxTorque (my_id);
    if (my_pos < 0) {
      printf ("GetMaxTorque() : error.");
    } else {
      printf ("GetMaxTorque:\n");
      printf (" my_pos = %d \n", my_pos);
    }

    my_pos = GetTorqueLimit (my_id);
    if (my_pos < 0) {
      printf ("GetTorqueLimit() : error.");
    } else {
      printf ("GetTorqueLimit:\n");
      printf (" my_pos = %d \n", my_pos);
    }
  } else {
    int i,ret;
    printf ("usage <id> <new_pos>\n");

    printf ("Bras droit:\n");
    my_id = 1;
    //my_pos = GetPosition (my_id);
    //printf (" - mouv av-arr.(id:%d) : %d\n", my_id, my_pos);
    printf (" - mouv av-arr.(id:%d) : ??\n", my_id);
    my_id = 81;
    my_pos = GetPosition (my_id);
    printf (" - rot. bras(id:%d) : %d\n", my_id, my_pos);
    my_id = 82;
    my_pos = GetPosition (my_id);
    printf (" - pivot epaule(id:%d) : %d\n", my_id, my_pos);
    my_id = 2;
    my_pos = GetPosition (my_id);
    printf (" - pivot coude(id:%d) : %d\n", my_id, my_pos);
    my_id = 3;
    my_pos = GetPosition (my_id);
    printf (" - pivot poigner(id:%d) : %d\n", my_id, my_pos);

    printf ("Bras gauche:\n");
    my_id = 4;
    my_pos = GetPosition (my_id);
    printf (" - mouv av-arr.(id:%d) : %d\n", my_id, my_pos);
    my_id = 83;
    my_pos = GetPosition (my_id);
    printf (" - rot. bras(id:%d) : %d\n", my_id, my_pos);
    my_id = 84;
    my_pos = GetPosition (my_id);
    printf (" - pivot epaule(id:%d) : %d\n", my_id, my_pos);
    my_id = 5;
    my_pos = GetPosition (my_id);
    printf (" - pivot coude(id:%d) : %d\n", my_id, my_pos);
    my_id = 6;
    my_pos = GetPosition (my_id);
    printf (" - pivot poigner(id:%d) : %d\n", my_id, my_pos);

#if 0
    ax12SetRegister(254, AX_ID, 3);

    my_pos = ax12GetRegister(254, AX_ID, 1);
    printf ("AX_ID = %d\n",my_pos);
#endif

    return -1;
  }

  return 0;
}
