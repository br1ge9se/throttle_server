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

/**
 * @file
 * @brief Utilities functions
 *
 */
 
#include "util.h"

static ENV env[ENVNUM];

static const struct device *dev = NULL;

static bool print_flag = false;

static int ctrlc_abort;

extern int nc_getchar(void);


/**
 * @brief 
 *
 * 
 */
void set_print_flag(bool val) {
    print_flag = val;
}

/**
 * @brief 
 *
 * 
 */
bool chk_print_flag(void) {
#ifdef DONGLE
    return (print_flag);
#else
    return true;
#endif
}
/**
 * @brief 
 *
 * 
 */
void initenv(void) {
    
}

/**
 * @brief 
 *
 * 
 */
void clrenv(void) {
	for (int i=0; i<ENVNUM; i++) {
		for (int j=0; j<VARSIZE; j++) {
			env[i].name[j] = 0;
			env[i].value[j] = 0;
		}

	}
}

/**
 * @brief Set enviromental variable
 * @param env. var. name
 * @param env. var. value
 *
 * @return success or error value 
 */
int setenv(const char * env_name, const char * env_value) {
	int nlen = strlen(env_name);
	int vlen = strlen(env_value);
	int i, ret = 0;
    
	if ((env_name != NULL) && (nlen != 0)) {
		if (nlen > VARSIZE)
			nlen = VARSIZE;
		if (vlen > VARSIZE)
			vlen = VARSIZE;
        // search for env name
		for (i=0; i<ENVNUM; i++) {
			if (strncmp(env_name, env[i].name, nlen) == 0) {
				strncpy(env[i].value, env_value, vlen);
				break;
			}
		}
		if (i == ENVNUM) {
           // name not found, create it
			for (i=0; i<ENVNUM; i++) {
				if (env[i].name[0] == 0) {
                    // free, place here
					strncpy(env[i].name, env_name, nlen);
					strncpy(env[i].value, env_value, vlen);
				}
			}
            if (i == ENVNUM) {
                // no free space
                ret = ERR_ENVFULL;
            }
		}
    } else {
        // wrong parameters
        ret = ERR_ENVPARAM;
    }
    return (ret);
}

/**
 * @brief Get enviromental variable
 * @param env. var. name
 *
 * @return value string pointer or NULL on error
 */
const char * getenv(const char *env_name) {
    int j, nlen;
    if (env_name != NULL) {
        nlen = strlen(env_name);
        if (nlen > VARSIZE)
            nlen = VARSIZE;
        for(j=0; j<nlen; j++){
            if (strncmp(env_name, env[j].name, nlen) == 0) {
                return (env[j].value);
            }
        }
	}
    return NULL;
}

/**
 * @brief 
 *
 * 
 */
void init_console(const struct device *mydev)
{
    dev = mydev;
}

/**
 * @brief Default character output routine
 * @param c Character to swallow
 *
 */
void console_putc(unsigned char c)
{
    if ((dev != NULL) && (chk_print_flag() == true)) {
        uart_poll_out(dev, c);
        if (c == '\n')
			uart_poll_out(dev, '\r');
    }
}

/**
 * @brief Default string output routine
 * @param pointer to the string
 *
 */
static int console_puts(const char *str)
{
	int n = 0;

	while (*str) {
//		if (*str == '\n')
//			console_putc('\r');

		console_putc(*str);
		str++;
		n++;
	}

	return n;
}

/**
 * @brief Default printf routine
 * @param args
 *
 */
int zprintf(const char *fmt, ...)
{
	va_list args;
	unsigned int i;
	char printbuffer[CFG_PBSIZE];

	va_start(args, fmt);
	i = vsprintf(printbuffer, fmt, args);
	va_end(args);
#ifdef DONGLE
	console_puts(printbuffer);
#else
    printk("%s", printbuffer);
#endif

	return i;
}

/**
 * @brief Function to reverse a string
 * @param string
 * @param lenght
 *
 */
void reverse(char str[], int length)
{
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        end--;
        start++;
    }
}

/**
 * @brief Implementation of itoa()
 * @param integer number
 * @param pointer to the string
 * @param numerical base
 * 
 */
char* itoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;
 
    /* Handle 0 explicitly, otherwise empty string is
     * printed for 0 */
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }
 
    // In standard itoa(), negative numbers are handled
    // only with base 10. Otherwise numbers are
    // considered unsigned.
    if (num < 0 && base == 10) {
        isNegative = true;
        num = -num;
    }
 
    // Process individual digits
    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }
 
    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';
 
    str[i] = '\0'; // Append string terminator
 
    // Reverse the string
    reverse(str, i);
 
    return str;
}

/**
 * @brief 
 *
 * 
 */
bool console_active(void) {
	return (chk_print_flag());
}


/*---------------------------------------------------*/
/* The purpose of this routine is to output data the */
/* same as the standard printf function without the  */
/* overhead most run-time libraries involve. Usually */
/* the printf brings in many kilobytes of code and   */
/* that is unacceptable in most embedded systems.    */
/*---------------------------------------------------*/
int nc_puts(const char *str)
{
	const u8_t *s = (const u8_t *) str;
	int n = 0;

	while (*s) {
//		if (*s == '\n')
//			n++;

		console_putc(*s);
		n++;
		s++;
	}
	return n;
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine puts pad characters into the output  */
/* buffer.                                           */
/*                                                   */
static void padding( const s32 l_flag, const struct params_s *par)
{
    s32 i;

    if ((par->do_padding != 0) && (l_flag != 0) && (par->len < par->num1)) {
		i=(par->len);
        for (; i<(par->num1); i++) {
            console_putc( par->pad_character);
		}
    }
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a string to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */
static void outs(const charptr lp, struct params_s *par)
{
    charptr LocalPtr;
	LocalPtr = lp;
    /* pad on left if needed                         */
	if(LocalPtr != NULL) {
		par->len = (s32)strlen( LocalPtr);
		padding( !(par->left_flag), par);
		/* Move string to the buffer                     */
		while (((*LocalPtr) != (char8)0) && ((par->num2) != 0)) {
			(par->num2)--;
			console_putc(*LocalPtr);
			LocalPtr += 1;
			//usleep(100);
//            k_sleep(K_MSEC(100));
		}
}

    /* Pad on right if needed                        */
    /* CR 439175 - elided next stmt. Seemed bogus.   */
    /* par->len = strlen( lp)                      */
    padding( par->left_flag, par);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine moves a number to the output buffer  */
/* as directed by the padding and positioning flags. */
/*                                                   */

static void outnum( const s32 n, const s32 base, struct params_s *par)
{
    s32 negative;
	s32 i;
    char8 outbuf[32];
    const char8 digits[] = "0123456789ABCDEF";
    u32 num;
    for(i = 0; i<32; i++) {
	outbuf[i] = '0';
    }

    /* Check if number is negative                   */
    if ((par->unsigned_flag == 0) && (base == 10) && (n < 0L)) {
        negative = 1;
		num =(-(n));
    }
    else{
        num = n;
        negative = 0;
    }

    /* Build number (backwards) in outbuf            */
    i = 0;
    do {
		outbuf[i] = digits[(num % base)];
		i++;
		num /= base;
    } while (num > 0);

    if (negative != 0) {
		outbuf[i] = '-';
		i++;
	}

    outbuf[i] = '\0';
    i--;

    /* Move the converted number to the buffer and   */
    /* add in the padding where needed.              */
    par->len = (s32)strlen(outbuf);
    padding( !(par->left_flag), par);
    while (&outbuf[i] >= outbuf) {
	console_putc( outbuf[i] );
		i--;
}
    padding( par->left_flag, par);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine gets a number from the format        */
/* string.                                           */
/*                                                   */
static s32 getnum(charptr* linep)
{
	s32 n = 0;
	s32 ResultIsDigit = 0;
	charptr cptr = *linep;

	while (cptr != NULL) {
		ResultIsDigit = isdigit(((s32)*cptr));
		if (ResultIsDigit == 0)
			break;
		n = ((n*10) + (((s32)*cptr) - (s32)'0'));
		cptr += 1;
	}

	*linep = ((charptr)(cptr));
	return(n);
}

/*---------------------------------------------------*/
/*                                                   */
/* This routine operates just like a printf/sprintf  */
/* routine. It outputs a set of data under the       */
/* control of a formatting string. Not all of the    */
/* standard C format control are supported. The ones */
/* provided are primarily those needed for embedded  */
/* systems work. Primarily the floating point        */
/* routines are omitted. Other formats could be      */
/* added easily by following the examples shown for  */
/* the supported formats.                            */
/*                                                   */
#ifdef DONGLE
/* void esp_printf( const func_ptr f_ptr,
   const charptr ctrl1, ...) */
void nc_printf( const char8 *ctrl1, ...)
{
	s32 Check;
    s32 dot_flag;

    params_t par;

    char8 ch;
    va_list argp;
    char8 *ctrl = (char8 *)ctrl1;

    va_start( argp, ctrl1);

    while ((ctrl != NULL) && (*ctrl != (char8)0)) {

        /* move format string chars to buffer until a  */
        /* format control is found.                    */
        if (*ctrl != '%') {
            console_putc(*ctrl);
			ctrl += 1;
            continue;
        }

        /* initialize all the flags for this format.   */
        dot_flag = 0;
        par.unsigned_flag = 0;
		par.left_flag = 0;
		par.do_padding = 0;
        par.pad_character = ' ';
        par.num2=32767;
		par.num1=0;
		par.len=0;

 try_next:
		if(ctrl != NULL) {
			ctrl += 1;
		}
		if(ctrl != NULL) {
			ch = *ctrl;
		} else {
			break;
		}

        if (isdigit((s32)ch) != 0) {
            if (dot_flag != 0) {
                par.num2 = getnum(&ctrl);
			}
            else {
                if (ch == '0') {
                    par.pad_character = '0';
				}
				if(ctrl != NULL) {
			par.num1 = getnum(&ctrl);
				}
                par.do_padding = 1;
            }
            if(ctrl != NULL) {
			ctrl -= 1;
			}
            goto try_next;
        }

        switch (tolower((s32)ch)) {
            case '%':
                console_putc( '%');
                Check = 1;
                break;

            case '-':
                par.left_flag = 1;
                Check = 0;
                break;

            case '.':
                dot_flag = 1;
                Check = 0;
                break;

            case 'l':
                Check = 0;
                break;

            case 'u':
                par.unsigned_flag = 1;
                /* fall through */
            case 'i':
            case 'd':
                outnum( va_arg(argp, s32), 10L, &par);
				Check = 1;
                break;
            case 'p':
                break;
            case 'X':
            case 'x':
                par.unsigned_flag = 1;
                outnum((s32)va_arg(argp, s32), 16L, &par);
                Check = 1;
                break;

            case 's':
                outs( va_arg( argp, char *), &par);
                Check = 1;
                break;

            case 'c':
                console_putc( va_arg( argp, s32));
                Check = 1;
                break;

            case '\\':
                switch (*ctrl) {
                    case 'a':
                        console_putc( ((char8)0x07));
                        break;
                    case 'h':
                        console_putc( ((char8)0x08));
                        break;
                    case 'r':
                        console_putc( ((char8)0x0D));
                        break;
                    case 'n':
                        console_putc( ((char8)0x0D));
                        console_putc( ((char8)0x0A));
                        break;
                    default:
                        console_putc( *ctrl);

                        break;
                }
                ctrl += 1;
                Check = 0;
                break;

            default:
		Check = 1;
		break;
        }
        if(Check == 1) {
			if(ctrl != NULL) {
				ctrl += 1;
			}
                continue;
        }
        goto try_next;
    }
    va_end( argp);
}
#endif

void ctrlc_handled(void)
{
	ctrlc_abort = 0;
}

/* test if ctrl-c was pressed */
int ctrlc(void)
{
	int ret = 0;

	if (ctrlc_abort)
		return 1;

//	if (nc_tstc() && nc_getchar() == 3)
//		ret = 1;

	if (ret)
		ctrlc_abort = 1;

	return ret;
}
