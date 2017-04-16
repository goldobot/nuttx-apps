/****************************************************************************
 * examples/robot_master_spi/robot_master_spi.c
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
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

unsigned char spi_buf_out[256];
unsigned char spi_buf_in[256];


/****************************************************************************
 * Public Functions
 ****************************************************************************/
#define SPI_FRAME_SZ 6

void send_spi_frame(void)
{
  int i;
  volatile uint32_t *spi_dr = (uint32_t *) 0x4001300c;
  volatile uint32_t *spi_sr = (uint32_t *) 0x40013008;

  spi_buf_in[0] = *spi_dr;
  for (i=0; i<SPI_FRAME_SZ; i++) {
    while (((*spi_sr)&2)==0);
    *spi_dr = spi_buf_out[i];
    while (((*spi_sr)&1)==0);
    spi_buf_in[i] = *spi_dr;
  }
  //while (((*spi_sr)&1)==0);
  spi_buf_in[SPI_FRAME_SZ] = *spi_dr;
}

int master_spi_read_word (unsigned int apb_addr, unsigned int *pdata)
{
  int i;
  uint32_t val;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  /* 1) sending APB_ADDR */
  spi_buf_out[0] = 0x30;
  spi_buf_out[1] = (apb_addr>>24) & 0xff;
  spi_buf_out[2] = (apb_addr>>16) & 0xff;
  spi_buf_out[3] = (apb_addr>>8) & 0xff;
  spi_buf_out[4] = (apb_addr) & 0xff;
  spi_buf_out[5] = 0;

  send_spi_frame();

  /* 2) reading data */
  spi_buf_out[0] = 0x50;
  spi_buf_out[1] = 0;
  spi_buf_out[2] = 0;
  spi_buf_out[3] = 0;
  spi_buf_out[4] = 0;
  spi_buf_out[5] = 0;

  send_spi_frame();

  val = 
    (spi_buf_in[1]<<24) + 
    (spi_buf_in[2]<<16) + 
    (spi_buf_in[3]<<8)  + 
    (spi_buf_in[4]);
  *pdata = val;

  return 0;
}

int master_spi_write_word (unsigned int apb_addr, unsigned int data)
{
  int i;

  for (i=0; i<SPI_FRAME_SZ; i++) {
    spi_buf_in[i] = 0;
    spi_buf_out[i] = 0;
  }

  /* 1) sending APB_ADDR */
  spi_buf_out[0] = 0x30;
  spi_buf_out[1] = (apb_addr>>24) & 0xff;
  spi_buf_out[2] = (apb_addr>>16) & 0xff;
  spi_buf_out[3] = (apb_addr>>8) & 0xff;
  spi_buf_out[4] = (apb_addr) & 0xff;
  spi_buf_out[5] = 0;

  send_spi_frame();

  /* 2) writing data */
  spi_buf_out[0] = 0x40;
  spi_buf_out[1] = (data>>24) & 0xff;
  spi_buf_out[2] = (data>>16) & 0xff;
  spi_buf_out[3] = (data>>8) & 0xff;
  spi_buf_out[4] = (data) & 0xff;
  spi_buf_out[5] = 0;

  send_spi_frame();

  return 0;
}


/****************************************************************************
 * hello_spi_main
 ****************************************************************************/
void usage(const char *prog_name)
{
  printf("Usage:\n");
  printf(" %s w <apb_addr> <data>\n", prog_name);
  printf(" %s r <apb_addr>\n", prog_name);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int robot_master_spi_main(int argc, char *argv[])
#endif
{
  int is_write=0;
  unsigned int data = 0x42424242;
  unsigned int apb_addr;

  if(argc<3) {
    usage(argv[0]);
    return 1;
  }

  printf("CMD : %s\n", argv[1]);

  if (argv[1][0]=='w') {
    is_write=1;
    if(argc<4) {
      usage(argv[0]);
      return 1;
    }
    apb_addr = strtoul(argv[2], NULL, 16);
    data = strtoul(argv[3], NULL, 16);
  } else if (argv[1][0]=='r') {
    is_write=0;
    apb_addr = strtoul(argv[2], NULL, 16);
  } else {
    usage(argv[0]);
    return 1;
  }

  if (is_write==1) {
    master_spi_write_word (apb_addr, data);
    printf(" @0x%.8x : W 0x%.8x \n", apb_addr, data);
  } else {
    master_spi_read_word (apb_addr, &data);
    printf(" @0x%.8x : R 0x%.8x \n", apb_addr, data);
  }

  return 0;
}
