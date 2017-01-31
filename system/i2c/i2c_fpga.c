/****************************************************************************
 * apps/system/i2c/i2c_set.c
 *
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>

#include <nuttx/i2c/i2c_master.h>

#include "i2ctool.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int i2ctool_fpga_write(FAR struct i2ctool_s *i2ctool, int fd, uint8_t i2c_reg,
		       uint32_t value);

int i2ctool_fpga_read (FAR struct i2ctool_s *i2ctool, int fd, uint8_t i2c_reg,
		       FAR uint32_t *pvalue);


int i2ccmd_fpga_write(FAR struct i2ctool_s *i2ctool, int argc, FAR char **argv)
{
  FAR char *ptr;
  uint32_t apb_addr;
  uint32_t fpga_reg_val;
  int nargs;
  int argndx;
  int ret;
  int fd;

  /* Parse any command line arguments */

  for (argndx = 1; argndx < argc; )
    {
      /* Break out of the look when the last option has been parsed */

      ptr = argv[argndx];
      if (*ptr != '-')
        {
          break;
        }

      /* Otherwise, check for common options */

      nargs = common_args(i2ctool, &argv[argndx]);
      if (nargs < 0)
        {
          return ERROR;
        }
      argndx += nargs;
    }

  /* There must be one more thing on the command line: 
   * The APB address.
   */

  if (argndx < argc)
    {
      apb_addr = strtol(argv[argndx], NULL, 16);
      i2ctool_printf(i2ctool, "apb_addr = %x\n", apb_addr);

      argndx++;
    }
  else
    {
      i2ctool_printf(i2ctool, g_i2cargrequired, argv[0]);
      return ERROR;
    }

  /* There must be one more thing on the command line: 
   * The value to be written in FPGA register
   */

  if (argndx < argc)
    {
      fpga_reg_val = strtol(argv[argndx], NULL, 16);
      i2ctool_printf(i2ctool, "fpga_reg_val = %x\n", fpga_reg_val);

      argndx++;
    }
  else
    {
      i2ctool_printf(i2ctool, g_i2cargrequired, argv[0]);
      return ERROR;
    }

  if (argndx != argc)
    {
      i2ctool_printf(i2ctool, g_i2ctoomanyargs, argv[0]);
      return ERROR;
    }

#if 0 /* FIXME : DEBUG */
  i2ctool_printf(i2ctool, "i2ctool->addr = %x\n", i2ctool->addr);
#endif


  fd = i2cdev_open(i2ctool->bus);
  if (fd < 0)
    {
       i2ctool_printf(i2ctool, "Failed to get bus %d\n", i2ctool->bus);
       return ERROR;
    }

  ret = i2ctool_fpga_write(i2ctool, fd, 0x03, apb_addr);
  if (ret != OK)
    {
      i2ctool_printf(i2ctool, "I2C Send apb_addr (0x03) failed (%d)\n", -ret);
      goto end;
    }

  ret = i2ctool_fpga_write(i2ctool, fd, 0x04, fpga_reg_val);
  if (ret != OK)
    {
      i2ctool_printf(i2ctool, "I2C APB write (0x04) failed (%d)\n", -ret);
      goto end;
    }

  i2ctool_printf(i2ctool, "WROTE %08x @ %08x\n", fpga_reg_val, apb_addr);

 end:
  (void)close(fd);
  return ret;
}

int i2ccmd_fpga_read(FAR struct i2ctool_s *i2ctool, int argc, FAR char **argv)
{
  FAR char *ptr;
  uint32_t apb_addr;
  uint32_t fpga_reg_val;
  int nargs;
  int argndx;
  int ret;
  int fd;

  /* Parse any command line arguments */

  for (argndx = 1; argndx < argc; )
    {
      /* Break out of the look when the last option has been parsed */

      ptr = argv[argndx];
      if (*ptr != '-')
        {
          break;
        }

      /* Otherwise, check for common options */

      nargs = common_args(i2ctool, &argv[argndx]);
      if (nargs < 0)
        {
          return ERROR;
        }
      argndx += nargs;
    }

  /* There must be one more thing on the command line: 
   * The APB address.
   */

  if (argndx < argc)
    {
      apb_addr = strtol(argv[argndx], NULL, 16);
      i2ctool_printf(i2ctool, "apb_addr = %x\n", apb_addr);

      argndx++;
    }
  else
    {
      i2ctool_printf(i2ctool, g_i2cargrequired, argv[0]);
      return ERROR;
    }

  if (argndx != argc)
    {
      i2ctool_printf(i2ctool, g_i2ctoomanyargs, argv[0]);
      return ERROR;
    }

#if 0 /* FIXME : DEBUG */
  i2ctool_printf(i2ctool, "i2ctool->addr = %x\n", i2ctool->addr);
#endif


  fd = i2cdev_open(i2ctool->bus);
  if (fd < 0)
    {
       i2ctool_printf(i2ctool, "Failed to get bus %d\n", i2ctool->bus);
       return ERROR;
    }

  ret = i2ctool_fpga_write(i2ctool, fd, 0x03, apb_addr);
  if (ret != OK)
    {
      i2ctool_printf(i2ctool, "I2C Send apb_addr (0x03) failed (%d)\n", -ret);
      goto end;
    }

  ret = i2ctool_fpga_read(i2ctool, fd, 0x05, &fpga_reg_val);
  if (ret != OK)
    {
      i2ctool_printf(i2ctool, "I2C APB read (0x05) failed (%d)\n", -ret);
      goto end;
    }

  i2ctool_printf(i2ctool, "READ %08x @ %08x\n", fpga_reg_val, apb_addr);

 end:
  (void)close(fd);
  return ret;
}


int i2ctool_fpga_write(FAR struct i2ctool_s *i2ctool, int fd, uint8_t i2c_reg,
                uint32_t value)
{
  struct i2c_msg_s msg[1];
  uint8_t  i2c_buf[5];
  int ret;

  i2c_buf[0] = i2c_reg;
  i2c_buf[1] = (value>>24) & 0xff;
  i2c_buf[2] = (value>>16) & 0xff;
  i2c_buf[3] = (value>>8) & 0xff;
  i2c_buf[4] = (value) & 0xff;

  msg[0].frequency = i2ctool->freq;
#if 0 /* FIXME : DEBUG */
  msg[0].addr      = i2ctool->addr;
#else
  msg[0].addr      = 0x42;
#endif
  msg[0].flags     = 0;
  msg[0].buffer    = i2c_buf;
  msg[0].length    = 5;

  ret = i2cdev_transfer(fd, msg, 1);

  return ret;
}

int i2ctool_fpga_read (FAR struct i2ctool_s *i2ctool, int fd, uint8_t i2c_reg,
		       FAR uint32_t *pvalue)
{
  struct i2c_msg_s msg[2];
  uint8_t i2c_buf[4];
  uint32_t data;
  int ret;

  msg[0].frequency = i2ctool->freq;
#if 0 /* FIXME : DEBUG */
  msg[0].addr      = i2ctool->addr;
#else
  msg[0].addr      = 0x42;
#endif
  msg[0].flags     = 0;
  msg[0].buffer    = &i2c_reg;
  msg[0].length    = 1;

  msg[1].frequency = i2ctool->freq;
#if 0 /* FIXME : DEBUG */
  msg[1].addr      = i2ctool->addr;
#else
  msg[1].addr      = 0x42;
#endif
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = i2c_buf;
  msg[1].length    = 4;

#if 1 /* FIXME : DEBUG */
  ret = i2cdev_transfer(fd, &msg[0], 1);
  if (ret== OK)
    {
      ret = i2cdev_transfer(fd, &msg[1], 1);
    }
#else
  ret = i2cdev_transfer(fd, &msg[0], 2);
#endif

  if (ret == OK)
    {
      data= (i2c_buf[0]<<24)+ (i2c_buf[1]<<16)+ (i2c_buf[2]<<8)+ (i2c_buf[3]);
      *pvalue = data;
    }

  return ret;
}
