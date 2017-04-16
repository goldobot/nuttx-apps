//***************************************************************************
// examples/demo_apds9960/demo_apds9960_main.cxx
//
//   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>

#include <cstdio>
#include <debug.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>

#include "SparkFun_APDS9960_goldo.h"

//***************************************************************************
// Definitions
//***************************************************************************
// Configuration ************************************************************
// C++ initialization requires CXX initializer support

#if !defined(CONFIG_HAVE_CXX) || !defined(CONFIG_HAVE_CXXINITIALIZE)
#  undef CONFIG_EXAMPLES_DEMO_APDS9960_CXXINITIALIZE
#endif

// Debug ********************************************************************
// Non-standard debug that may be enabled just for testing the constructors

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_CXX
#endif

#ifdef CONFIG_DEBUG_CXX
#  define cxxinfo     _info
#else
#  define cxxinfo(x...)
#endif

//***************************************************************************
// Private Classes
//***************************************************************************

//***************************************************************************
// Private Data
//***************************************************************************

//***************************************************************************
// Public Functions
//***************************************************************************

/****************************************************************************
 * Name: demo_apds9960_main
 ****************************************************************************/

extern "C"
{
  int demo_apds9960_main(int argc, char *argv[])
 {
    // If C++ initialization for static constructors is supported, then do
    // that first

#ifdef CONFIG_EXAMPLES_DEMO_APDS9960_CXXINITIALIZE
    up_cxxinitialize();
#endif

    SparkFun_APDS9960 *my_spark = new SparkFun_APDS9960();
    uint16_t my_val = 0x42;

    if (my_spark->init()) {
      printf("my_spark->init() : OK\n");
    } else {
      printf("my_spark->init() : KO\n");
      return -1;
    }

    if (my_spark->enableLightSensor(false)) {
      printf("my_spark->enableLightSensor() : OK\n");
    } else {
      printf("my_spark->enableLightSensor() : KO\n");
      return -1;
    }

    usleep (200000);

    if (my_spark->readAmbientLight(my_val)) {
      printf("my_spark->readAmbientLight() : OK\n");
      printf(" val = %d\n", my_val);
    } else {
      printf("my_spark->readAmbientLight() : KO\n");
      return -1;
    }

    if (my_spark->readRedLight(my_val)) {
      printf("my_spark->readRedLight() : OK\n");
      printf(" val = %d\n", my_val);
    } else {
      printf("my_spark->readRedLight() : KO\n");
      return -1;
    }

    if (my_spark->readGreenLight(my_val)) {
      printf("my_spark->readGreenLight() : OK\n");
      printf(" val = %d\n", my_val);
    } else {
      printf("my_spark->readGreenLight() : KO\n");
      return -1;
    }

    if (my_spark->readBlueLight(my_val)) {
      printf("my_spark->readBlueLight() : OK\n");
      printf(" val = %d\n", my_val);
    } else {
      printf("my_spark->readBlueLight() : KO\n");
      return -1;
    }

    delete my_spark;

    return 0;
  }
}
