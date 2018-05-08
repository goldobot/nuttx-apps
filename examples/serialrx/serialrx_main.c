/****************************************************************************
 * examples/serialrx/serialrx_main.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

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
 * serialrx_main
 ****************************************************************************/
#define MY_BUF_SZ 256
char my_buf[MY_BUF_SZ];

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int serialrx_main(int argc, char *argv[])
#endif
{
  int fd;
#ifdef CONFIG_EXAMPLES_SERIALRX_PRINTHYPHEN
  int count = 0;
#endif
#ifdef CONFIG_EXAMPLES_SERIALRX_PRINTHEX
  int i;
#endif
  bool eof = false;
  FAR char *buf;
  FAR char *devpath;

  if (argc == 1) {
    devpath = CONFIG_EXAMPLES_SERIALRX_DEVPATH;
  } else if (argc == 2) {
    devpath = argv[1];
  } else {
    printf("Usage: %s [devpath]\n", argv[0]);
    goto errout;
  }

  buf = (FAR char *)my_buf;

  fd = open(devpath, O_RDWR);
  if (fd < 0) {
    printf("ERROR: open failed: %d\n", errno);
    goto errout_with_buf;
  }

  write(fd, "Serial test\n", 13);

  printf("Reading from %s\n", devpath);

  while (!eof) {
    int n = read(fd, buf, 1);
    if (n == 0) {
      eof = true;
    } else if (n < 0) {
      printf("read failed: %d\n", errno);
    } else {
      printf("%c\n", buf[0]);
      buf[0]++;
      write(fd, buf, 1);
    }
  }

#ifdef CONFIG_EXAMPLES_SERIALRX_PRINTHYPHEN
  printf("\n");
#endif
  printf("EOF reached\n");

  close(fd);

  free(buf);
  return EXIT_SUCCESS;

errout_with_buf:
  free(buf);

errout:
  return EXIT_FAILURE;
}
