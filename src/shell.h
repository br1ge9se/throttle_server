 #ifndef NC_PRINTF_H
 #define NC_PRINTF_H

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>

#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include "types.h"

/*----------------------------------------------------*/
/* Use the following parameter passing structure to   */
/* make nc_printf re-entrant.                        */
/*----------------------------------------------------*/

struct params_s;

/*               Prototypes                          */

void nc_printf( const char8 *ctrl1, ...);
void nc_print( const char8 *ptr);
int nc_puts(const char *str);
//extern void nc_putc (unsigned char c);
extern void console_putc (unsigned char c);
#define nc_putc console_putc


#endif	/* end of protection macro */
