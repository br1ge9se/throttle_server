/**
 * @file
 * @brief Simple Shell/Command Parser
 *
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include "shell.h"
#include "util.h"

#define CONFIG_CBSIZE 128
#define CONFIG_MAXARGS 5
#define CONFIG_PROMPT "shell>"
#ifndef __maybe_unused
#define __maybe_unused		__attribute__((unused))
#endif

static char console_buffer[CONFIG_CBSIZE];		/* console I/O buffer	*/
static char erase_seq[] = "\b \b";		/* erase sequence	*/
static char   tab_seq[] = "        ";		/* used to expand TABs	*/

#define COMMAND_SUCCESS		0
int execute_command(int argc, char **argv);

extern char* itoa(int num, char* str, int base);
extern int nc_getchar(void);


static char * delete_char (char *buffer, char *p, int *colp, int *np, int plen)
{
	char *s;

	if (*np == 0) {
		return (p);
	}

	if (*(--p) == '\t') {			/* will retype the whole line	*/
		while (*colp > plen) {
			nc_puts (erase_seq);
			(*colp)--;
		}
		for (s=buffer; s<p; ++s) {
			if (*s == '\t') {
				nc_puts (tab_seq+((*colp) & 07));
				*colp += 8 - ((*colp) & 07);
			} else {
				++(*colp);
				console_putc (*s);
			}
		}
	} else {
		nc_puts (erase_seq);
		(*colp)--;
	}
	(*np)--;
	return (p);
}

/*
 * Prompt for input and read a line.
 * Return:	number of read characters
 *		-1 if break
 */
int readline (const char *prompt, char *line, int len)
{
	static char *p;
	static int	n = 0;				/* buffer index		*/
	static int	plen = 0;			/* prompt length	*/
	static int fprompt = 0;			/* flag */
	int	col;						/* output column cnt	*/
	int	c;

	if (!fprompt) {
		/* print prompt */
		if (prompt) {
			plen = strlen (prompt);
			nc_puts (prompt);
			memset(line, 0, len);
			fprompt = 1;
			p = line;
			n = 0;
		}
	}
	col = plen;

	c = nc_getchar();
	if (c < 0)
		return (-1);

	/*
	 * Special character handling
	 */
	switch (c) {
		case '\r':				/* Enter		*/
		case '\n':
			*p = '\0';
			nc_puts ("\r\n");
			fprompt = 0;
			return (p - line);

		case '\0':				/* nul			*/
			break;

		case 0x03:				/* ^C - break		*/
			line[0] = '\0';	/* discard input */
			fprompt = 0;
			nc_puts ("\r\n");
			return (-1);

		case 0x15:				/* ^U - erase line	*/
			while (col > plen) {
				nc_puts (erase_seq);
				--col;
			}
			p = line;
			n = 0;
			break;

		case 0x17:				/* ^W - erase word 	*/
			p=delete_char(line, p, &col, &n, plen);
			while ((n > 0) && (*p != ' ')) {
				p=delete_char(line, p, &col, &n, plen);
			}
			break;

		case 0x08:				/* ^H  - backspace	*/
		case 0x7F:				/* DEL - backspace	*/
			p=delete_char(line, p, &col, &n, plen);
			break;

		default:
			/*
			 * Must be a normal character then
			 */
			if (n < len-2) {
				if (c == '\t') {	/* expand TABs		*/
					nc_puts (tab_seq+(col&07));
					col += 8 - (col&07);
				} else {
					++col;		/* echo input		*/
					console_putc (c);
				}
				*p++ = c;
				++n;
			} else {			/* Buffer full		*/
				console_putc ('\a');
			}
			break;
	}
	return 0;
}

static int parse_line (char *line, char *argv[])
{
	int nargs = 0;

//	xil_printf("parse_line: %s\r\n", line);

	while (nargs < CONFIG_MAXARGS) {

		/* skip any white space */
		while ((*line == ' ') || (*line == '\t')) {
			++line;
		}

		if (*line == '\0') {	/* end of line, no more args	*/
			argv[nargs] = NULL;

//			xil_printf("parse_line: nargs=%d\n\r", nargs);

			return (nargs);
		}

		argv[nargs++] = line;	/* begin of argument string	*/

		/* find end of string */
		while (*line && (*line != ' ') && (*line != '\t')) {
			++line;
		}

		if (*line == '\0') {	/* end of line, no more args	*/
			argv[nargs] = NULL;

//			xil_printf("parse_line: nargs=%d\n\r", nargs);

			return (nargs);
		}

		*line++ = '\0';		/* terminate current arg	 */
	}

//	xil_printf ("** Too many args (max. %d) **\n\r", CONFIG_MAXARGS);
	nc_puts("** Too many args (max. ");
	char bf[2] = {0,0};
	itoa(CONFIG_MAXARGS, bf, 10);
	nc_puts(bf);
	nc_puts(") **\n\r");
//	xil_printf("parse_line: nargs=%d\n\r", nargs);
	return (nargs);
}

static void process_macros (const char *input, char *output)
{
	char c, prev;
	const char *varname_start = NULL;
	int inputcnt = strlen (input);
	int outputcnt = CONFIG_CBSIZE;
	int state = 0;		/* 0 = waiting for '$'  */

	/* 1 = waiting for '(' or '{' */
	/* 2 = waiting for ')' or '}' */
	/* 3 = waiting for '''  */
	char __maybe_unused *output_start = output;

//	xil_printf("[PROCESS_MACROS] INPUT len %d: - %s\r\n", strlen (input), input);

	prev = '\0';		/* previous character   */

	while (inputcnt && outputcnt) {
		c = *input++;
		inputcnt--;

		if (state != 3) {
			/* remove one level of escape characters */
			if ((c == '\\') && (prev != '\\')) {
				if (inputcnt-- == 0)
					break;
				prev = c;
				c = *input++;
			}
		}

		switch (state) {
		case 0:	/* Waiting for (unescaped) $    */
			if ((c == '\'') && (prev != '\\')) {
				state = 3;
				break;
			}
			if ((c == '$') && (prev != '\\')) {
				state++;
			} else {
				*(output++) = c;
				outputcnt--;
			}
			break;
		case 1:	/* Waiting for (        */
			if (c == '(' || c == '{') {
				state++;
				varname_start = input;
			} else {
				state = 0;
				*(output++) = '$';
				outputcnt--;

				if (outputcnt) {
					*(output++) = c;
					outputcnt--;
				}
			}
			break;
		case 2:	/* Waiting for )        */
			if (c == ')' || c == '}') {
				int i;
				char envname[CONFIG_CBSIZE];
				const char *envval;
				int envcnt = input - varname_start - 1;	/* Varname # of chars */

				/* Get the varname */
				for (i = 0; i < envcnt; i++) {
					envname[i] = varname_start[i];
				}
				envname[i] = 0;

				/* Get its value */
				envval = getenv (envname);

				/* Copy into the line if it exists */
				if (envval != NULL)
					while ((*envval) && outputcnt) {
						*(output++) = *(envval++);
						outputcnt--;
					}
				/* Look for another '$' */
				state = 0;
			}
			break;
		case 3:	/* Waiting for '        */
			if ((c == '\'') && (prev != '\\')) {
				state = 0;
			} else {
				*(output++) = c;
				outputcnt--;
			}
			break;
		}
		prev = c;
	}

	if (outputcnt)
		*output = 0;
/*
	xil_printf("[PROCESS_MACROS] INPUT len %d: - %s\r\n",
		strlen (output_start), output_start);
*/
}

/**
 * @brief Run command.
 *
 * @param cmd - command name string
 *
 * @return int - 0: command executed,
 *               -1: not executed
 *                -unrecognized or too many args
 *                -If cmd is NULL or "" or longer than
 *               CONFIG_CBSIZE-1 it is considered unrecognized
 */

int run_command(const char *cmd)
{
	char cmdbuf[CONFIG_CBSIZE];	/* working copy of cmd		*/
	char *token;			/* start of token in cmdbuf	*/
	char *sep;			/* end of token (separator) in cmdbuf */
	char finaltoken[CONFIG_CBSIZE];
	char *str = cmdbuf;
	char *argv[CONFIG_MAXARGS + 1];	/* NULL terminated	*/
	int argc, inquotes;
	int rc = 0;

#ifdef DEBUG
	xil_printf("[RUN_COMMAND] cmd[%p]=", cmd);
	xil_printf (cmd ? cmd : "NULL");	/* use puts - string may be loooong */
	xil_printf ("\r\n");
#endif

	if (!cmd || !*cmd) {
		return -1;	/* empty command */
	}

	if (strlen(cmd) >= CONFIG_CBSIZE) {
		nc_puts ("## Command too long!\n\r");
		return -1;
	}

	strcpy (cmdbuf, cmd);

	/*
	 * Process separators and check for invalid
	 * repeatable commands
	 */

//	xil_printf("[PROCESS_SEPARATORS] %s\n\r", cmd);

	while (*str) {

		/*
		 * Find separator, or string end
		 * Allow simple escape of ';' by writing "\;"
		 */
		for (inquotes = 0, sep = str; *sep; sep++) {
			if ((*sep=='\'') &&
			    (*(sep-1) != '\\'))
				inquotes=!inquotes;

			if (!inquotes &&
			    (*sep == ';') &&	/* separator		*/
			    ( sep != str) &&	/* past string start	*/
			    (*(sep-1) != '\\'))	/* and NOT escaped	*/
				break;
		}

		/*
		 * Limit the token to data between separators
		 */
		token = str;
		if (*sep) {
			str = sep + 1;	/* start of command for next pass */
			*sep = '\0';
		}
		else {
			str = sep;	/* no more commands for next pass */
		}

	//	xil_printf("token: %s\r\n", token);

		/* find macros in this token and replace them */
		process_macros (token, finaltoken);

		/* Extract arguments */
		if ((argc = parse_line (finaltoken, argv)) == 0) {
			rc = -1;	/* no command at all */
			continue;
		}

		if (execute_command(argc, argv) != COMMAND_SUCCESS)
			rc = -1;
	}

	return rc;
}

/**
 * @brief The shell entry point.
 *
 * This is called as part of machine loop.
 *
 * @return void
 */
void run_shell (void)
{
    static char lastcommand[CONFIG_CBSIZE] = { 0, };
	int len;

	if (console_active()) {
		len = readline (CONFIG_PROMPT, console_buffer, CONFIG_CBSIZE);
		if (len > 0) {
			strcpy (lastcommand, console_buffer);
			const int rc = run_command(lastcommand);
			if (rc < 0) {
				/* invalid command or not repeatable, forget it */
				lastcommand[0] = 0;
			}
			ctrlc_handled();
		}
	}
}
