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
 * Send postion with synchronisation
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

#define msleep(i) usleep(i*100)
void SetPosition(int id, int pos);
#define PreparePostion(id,pos) (ax12SetRegisterSync2(id, AX_GOAL_POSITION_L, pos))
#define SetID(id, newID) (ax12SetRegister(id, AX_ID, newID))
#define SetBD(id, newBD) (ax12SetRegister(id, AX_BAUD_RATE, newBD))
#define GetID(id) (ax12GetRegister(id,AX_ID,1))
#define GetBD(id) (ax12GetRegister(id,AX_BAUD_RATE,1))
#define IsMoving(id) (ax12GetRegister(id,AX_MOVING,1))
#define GetPreseLoad(id) (ax12GetRegister(id, AX_PRESENT_LOAD_L, 2))

static int fd = -1;
#define id1  81
#define id2 82
#define id3  2
#define id4 101
unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
volatile int ax_rx_int_Pointer;
/** read back the error code for our latest packet read */
int ax12Error;
int pos1;
int pos2;
int pos3;
int pos4;
/** > 0 = success */


int goldo_dynamixels_init(void)
{
    fd = open ("/dev/ttyS1", O_RDWR | O_NONBLOCK);
    if (fd<0) {
        return ERROR;
      }
    return OK;
}
int ax12ReadPacket(int length){
    //unsigned long ulCounter;
    unsigned char offset, /*blength,*/ checksum /*, timeout*/;
    unsigned char volatile bcount; 
    char c;
    int i; 
    //printf("Read Packet");
    offset = 0;
    //timeout = 0;
    bcount = 0;
#if 1
    for (i=0; i<4; i++) {
    if(read (fd, &c, 1)!=1) break; /* on poubelise les 4 premiers chars (FIXME : TODO : trouver l'explication) */
    }
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
    //usleep(10); /* cette tempo correspond grosso-modo a la transmission d'1 caractere a 1Mbaud (+ un peu de marge..) */
    if(read (fd, &c, 1)!=1) break; /* si la valeur retournee par la fonction 'read()' n'est pas 1 => il n'y + rien a lire */
        ax_rx_buffer[bcount] = c;
#endif
        if((bcount == 0) && (ax_rx_buffer[0] != 0xff))
            offset++;
        else
            bcount++;
    }

    for (i=0; i<bcount; i++) {
      //printf ("recv : %.2x\n", ax_rx_buffer[i]);
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
#else
    printf ("\n");
    while(read (fd, &c, 1)==1) {
      printf ("recv : %.2x\n", c);
    }
    return 0;
#endif
}

typedef struct dynamixel_packet_header_s
{
    uint16_t magic;//0xFFFF
    uint8_t id;
    uint8_t length;
    uint8_t command;

} dynamixel_packet_header_s;


static void dynamixel_write_packet(uint8_t id, uint8_t command, size_t length, const char* buffer)
{
    dynamixel_packet_header_s header = {0xFFFF,id,length+2,command};
    uint8_t checksum = id + command + length;
    for(int i=0;i<length;i++)
    {
        checksum += (uint8_t)(buffer[i]);
    }
    checksum = ~checksum;
    write(fd, &header, sizeof(header));
    write(fd, buffer, length);
    write(fd,&checksum,1);
}

static int dynamixel_read_packet(size_t buffer_length, const char* buffer, uint8_t* error)
{
    dynamixel_packet_header_s* header_ptr;
    uint8_t c;
    //Wait till the 2 start bytes are received.
    int offset=0;
    while(offset != 2)
    {
        if(read(fd,&c,1))
        {
            //\todo manage timeout
            if(c == 0xFF)
            {
                ax_rx_buffer[offset] = c;
                offset++;
            } else
            {
                offset=0;
            }            

        } else
        {
            usleep(100);
        }        
    }

    read(fd,ax_rx_buffer+offset,sizeof(dynamixel_packet_header_s)-2);
    header_ptr = (dynamixel_packet_header_s*)(ax_rx_buffer);
    read(fd,buffer+sizeof(dynamixel_packet_header_s),header_ptr->length);

    uint8_t checksum;    
    return header_ptr->length-2;

    //printf("Read packet , length: %i, error: %i\n", header.length);
   
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
    write(fd, &c, 1);
    ax12ReadPacket(6);
}
void ax12SetRegister2(int id, int regstart, int data){
    //setTX(id);
    int length = 5;
    int checksum = ~((id + length + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    char c;
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
    c=(data&0xff00)>>8;
    write(fd, &c, 1);
    // checksum =
    c=checksum;
    write(fd, &c, 1);
    //setRX(id);
    ax12ReadPacket(6);
}
void ax12SetRegisterSync2(int id, int regstart, int data){
    //setTX(id);
    int length = 5;
    int checksum = ~((id + length + AX_REG_WRITE + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    char c;
    
    c=0xFF;
    write(fd, &c, 1);
    c=0xFF;
    write(fd, &c, 1);
    c=id;
    write(fd, &c, 1);
    c=length; // length
    write(fd, &c, 1);
    c=AX_REG_WRITE;
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
    ax12ReadPacket(6);
}
void ax12SetRegisterSync(int id, int regstart, int data){
    //setTX(id);
    int length = 4;
    int checksum = ~((id + length + AX_REG_WRITE + regstart + (data&0xFF)) % 256);
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
    c=AX_REG_WRITE;
    write(fd, &c, 1);
    c=regstart;
    write(fd, &c, 1);
    c=data&0xff;
    write(fd, &c, 1);
    // checksum =
    c=checksum;
    write(fd, &c, 1);
    //setRX(id);
    ax12ReadPacket(6);
}
void ax12Action(void){
    
    int id = 0xFE;    // Broadcast ID
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
int ax12GetRegister(int id, int regstart, int length){  
    //setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    char c;
    //sleep(1);
//printf("ax12GetRegister");
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
int dynamixel_get_current_position(int id)    {
        return ax12GetRegister(id, AX_PRESENT_POSITION_L,2);
    }

int dynamixel_set_led(int id, int enable)   
{
    uint8_t val = enable;
    uint8_t error;
    dynamixel_write_packet(id,AX_WRITE_DATA,1,&val);
    dynamixel_read_packet(0,NULL,&error);

    //ax12SetRegister(id, AX_LED, enable);
    return OK;
}

    int GetPosition(int id)    {
        return ax12GetRegister(id, AX_GOAL_POSITION_L, 2);
    }

void SetTorque(int id,int value){
   ax12SetRegister2(id, AX_TORQUE_LIMIT_L, value); 
   if(value>0) 
   {
    ax12SetRegister(id,AX_TORQUE_ENABLE,1);
} else
{
    ax12SetRegister(id,AX_TORQUE_ENABLE,0);
}
    
   //printf("Return torque enable : %i\n",ax12ReadPacket(6));
   //msleep(100);
}

void SetPosition(int id,int pos) 
{
    ax12SetRegister2(id, AX_GOAL_POSITION_L, pos);
}

void goldo_dynamixels_set_position(int id,int pos) 
{
    ax12SetRegister2(id, AX_GOAL_POSITION_L, pos);
}

void goldo_dynamixels_set_position_sync(int id,int pos)
{
    //printf("goldo_dynamixels_set_position_sync: %i, %i\n",id,pos);
    ax12SetRegisterSync2(id, AX_GOAL_POSITION_L, pos);
}

void goldo_dynamixels_do_action()
{
    ax12Action();
}


int chooseId(int id){
    if(id == 1){ return id1;}
    else if (id == 2){return id2;}
    else if(id ==3){return id3;}
    else if(id ==4){return id4;}
    else{ return -1;}    
}
int checkPos(int id,int pos){
    if(id == id1 || id == id2){ 
        if (pos > -1 && pos < 4096){
            return pos;
                
            }
            else{
                return -1;
            }
    }
    else if (id == id3||id == id4){
        if (pos > -1 && pos < 1024){
            return pos;
                
            }
            else{
                return -1;
            }
    }
    else{ 
        return -1;
        
    }    

}
void printPosition(void){
    printf("Moto positions :\n");
    printf("MX-28 id 81, position : ");
    printf("%i \n",GetPosition(id1));
    ax12ReadPacket(6);
    printf("MX-28 id 82, position : ");
    printf("%i \n",GetPosition(id2));
    ax12ReadPacket(6);
    printf("AX-12 id 2, position : ");
    printf("%i \n",GetPosition(id3));
    ax12ReadPacket(6);
    printf("AX-12 id 101, position : ");
    printf("%i \n",GetPosition(id4));
    ax12ReadPacket(6);
    
}
void printLoad(void){
    printf("Moto load :\n");
    printf("MX-28 id 81, load : ");
    printf("%i \n",GetPreseLoad(id1));
    ax12ReadPacket(6);
    printf("MX-28 id 82, load : ");
    printf("%i \n",GetPreseLoad(id2));
    ax12ReadPacket(6);
    printf("AX-12 id 2, load : ");
    printf("%i \n",GetPreseLoad(id3));
    ax12ReadPacket(6);
    printf("AX-12 id 101, load : ");
    printf("%i \n",GetPreseLoad(id4));
    ax12ReadPacket(6);
    
}
void savePosition(int id,int pos){//save goal position to compare after move
    if(id==1){
            pos1 = pos;
    }
    else if(id==2){
            pos2 = pos;
    }
    else if(id==3){
        pos3= pos;
            
    }
    else if(id==4){
    pos4= pos;
        
    }
}
void movingLoad(void){
    printf("ici\n");
    /*for(int i = 0;i<100;i++){
        printf("%i\n",IsMoving(id2));
        msleep(1);
    }*/
    while(IsMoving(id1) == 1 || IsMoving(id2) == 1|| IsMoving(id3)==1||IsMoving(id4)==1){
            printf("Load id 1 : %d\n",GetPreseLoad(id1));
            msleep(1);
            printf("Load id 2 : %d\n",GetPreseLoad(id2));
            msleep(1);
            printf("Load id 3 : %d\n",GetPreseLoad(id3));
            msleep(1);
            printf("Load id 4 : %d\n",GetPreseLoad(id4));
            msleep(1);}
}
void correctPosition(int id,int currentPos,int goalPos,int maxPos){
    if(goalPos-currentPos>0){
        if(currentPos-100<0){
            SetPosition(id,currentPos-100);
        }
        else{
            SetPosition(id,0);
        }
    }
    else{
        if(currentPos+100>maxPos){
            SetPosition(id,currentPos+100);
        }
        else{
            SetPosition(id,maxPos);
        }
        
    }
    
    
}
int checkMove(void){
    movingLoad();
    if (pos1 != GetPosition(id1)){
    printf("Error on servo 1\n");
        correctPosition(id1,GetPosition(id1),pos1, 4095);
    return 0;}
    if (pos2 != GetPosition(id2)){
    printf("Error on servo 2\n");
        correctPosition(id2,GetPosition(id2),pos2, 4095);
    return 0;}
    if (pos3 != GetPosition(id3)){
    printf("Error on servo 3\n");
        correctPosition(id3,GetPosition(id3),pos3, 1023);
    return 0;}
    if (pos4!= GetPosition(id4)){
        correctPosition(id4,GetPosition(id4),pos4, 1023);
    printf("Error on servo 4\n");
    return 0;}
    return 1;
    
}
/*
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int dynamixel_main(int argc, char *argv[])
#endif
{
int pos = -1;
int id =-1;
  fd = open ("/dev/ttyS1", O_RDWR | O_NONBLOCK);
  if (fd<0) {
    return -1;
  }
  
#if 1

SetTorque(id1,512);
SetTorque(id2,512);
SetTorque(id3,512);
SetTorque(id4,512);
pos1 = GetPosition(id1);
pos2 = GetPosition(id2);
pos3 = GetPosition(id3);
pos4 = GetPosition(id4);
if(argc>1){
  if (argc%2 != 0) {
    int i ;
    for(i = 1;i< argc;i+=2){
        id =  atoi (argv[i]);
        id = chooseId(id);
        if(id != -1){
            pos = atoi (argv[i+1]);
            //printf("Position recived : %i\n",pos);
            pos = checkPos(id,pos);
            if (pos !=-1){
                printf("Motor %i, position %i\n",id,pos);
                PreparePostion (id, pos);
                savePosition(id,pos);
                ax12ReadPacket(6);
            }
            else{
                printf("Wrong postion, 0 to 1024 for motor 1 & 2 or 0 to 4096 for motor 3 & 4 \n");
            }
        }
        else{
             printf ("Wrong motor number, from 1 to 4 \n");
        }
    }
    ax12Action();
        movingLoad();
    /*if (checkMove()==0){
        printf("Error during moove\n");
        
    }
  }
  else {
    printf ("Wrong number of argument. The format is : motor number, position\n");
  }
}
else{
  msleep(1);
  printPosition();
  printLoad();
}
#else
printPosition();
initTorque();
for(int i = 0;i<5;i++){
printPosition();
}
#endif
  return 0;
}
*/