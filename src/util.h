/*
 * Copyright (c) 2025 Br1Ge9se
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * 
 * Author: Bruno Genovese <br1ge9se>
 *
 */
#ifndef __SHELL_UTIL_H__
#define __SHELL_UTIL_H__

#include <zephyr/kernel.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <zephyr/drivers/uart.h>
#include "types.h"
#include "board_conf.h"
 
#define CFG_PBSIZE 256

#define VARSIZE	32
typedef struct {
	char name[VARSIZE];
	char value[VARSIZE];
} ENV;
#define ENVNUM	16

#define ERR_ENVFULL     1
#define ERR_ENVPARAM    2

typedef char8* charptr;
typedef s32 (*func_ptr)(int c);


typedef struct params_s {
    s32 len;
    s32 num1;
    s32 num2;
    char8 pad_character;
    s32 do_padding;
    s32 left_flag;
    s32 unsigned_flag;
} params_t;

/*               Prototypes                          */

void set_print_flag(bool val);

bool chk_print_flag(void);

#ifdef DONGLE
void nc_printf( const char8 *ctrl1, ...);
#else
#define nc_printf zprintf
#endif
#define my_printf nc_printf

//#define nc_printf zprintf

void ctrlc_handled(void);

int ctrlc(void);

int nc_puts(const char *str);

int setenv(const char * env_name, const char * env_value);

const char * getenv(const char *env_name);

void clrenv(void);

void initenv(void);

void init_console(const struct device *mydev);

void console_putc(unsigned char c);

static int console_puts(const char *str);

int zprintf(const char *fmt, ...);

void reverse(char str[], int length);

char* itoa(int num, char* str, int base);

bool console_active(void);

int nc_getchar(void);
 
#endif /* __SHELL_UTIL_H__ */