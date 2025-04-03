/**
 * @file
 * @brief Shell Commands Handler
 *
 */

#include <string.h>
#include <stdio.h>
#include "util.h"

#define MAX_CMD 6

#define COMMAND_SUCCESS		0
#define COMMAND_ERROR		1
#define COMMAND_ERROR_USAGE	2

/*
 * Monitor Command Table
 */
typedef struct command {
	const char	*name;		/* Command Name			*/
	int		(*cmd)(int, char *[]); /* Implementation function	*/
	const char	*desc;		/* Short command description, start with lowercase */
	const char	*help;
}command;

extern int mem_md(int argc, char *argv[]);
extern int ble_info(void);
extern int print_rx_value(void);
extern int print_temp_value(void);

int helpCmd(int argc, char *argv[]);
int versCmd(int argc, char *argv[]);

struct command cmdTable[MAX_CMD] = {
		{"version", versCmd, "print out version info", NULL},
		{"info", ble_info, "print out BLE info", NULL},
		{"help", helpCmd, "print out command list", NULL},
		{"val", print_rx_value, "print out BLE Rx Value from client", NULL},
		{"temp", print_temp_value, "print out chip temperature", NULL},
		{"md", mem_md, "display (hex dump) a memory region",
		"Memory regions can be specified in two different forms: START+SIZE or START-END, if SIZE is omitted it defaults to 0x100"}
};


#define VERSION "  Shell V0.1 (beta) - "

int versCmd(int argc, char *argv[])
{
	char s1[60]=VERSION, s2[]=__DATE__, s3[]=__TIME__;
	strcat(s1, s2);
	strcat(s1, " * ");
	strcat(s1, s3);
	nc_printf ("\n%s\n\n", s1);
	return (COMMAND_SUCCESS);
}

char * help_string = "  ****** Commands help ******";

int helpCmd(int argc, char *argv[])
{
	int i;
	nc_printf ("\n\r%s\n\r", help_string);
	for (i=0; i<MAX_CMD; i++)
	{
		nc_printf("%s - %s\n\r", cmdTable[i].name, cmdTable[i].desc);
	}
	console_putc('\n');
	return (COMMAND_SUCCESS);
}


void cmd_usage(struct command *cmdtp)
{
	if (cmdtp->desc)
		nc_printf("\n%s - %s\n\n", cmdtp->name, cmdtp->desc);
	if (cmdtp->help)
		nc_printf("\nUsage: %s %s\n\n", cmdtp->name, cmdtp->help);
}

/*
 * find command table entry for a command
 */
struct command *find_cmd (const char *cmd)
{
	struct command *cmdtp;
	int i;

	for (i=0; i<MAX_CMD; i++) {
		cmdtp = &cmdTable[i];
		if (!strcmp(cmd, cmdtp->name))
			return cmdtp;
	}
	return NULL;	/* not found or ambiguous command */
}

int execute_command(int argc, char **argv)
{
	struct command *cmdtp;
	int ret;

	/* Look up command in command table */
	if ((cmdtp = find_cmd(argv[0]))) {
		/* OK - call function to do the command */
		ret = cmdtp->cmd(argc, argv);
		if (ret == COMMAND_ERROR_USAGE) {
			cmd_usage(cmdtp);
			ret = COMMAND_ERROR;
		}
	} else {
		nc_printf ("Unknown command '%s'\n", argv[0]);
		ret = COMMAND_ERROR;	/* give up after bad command */
	}
	return ret;
}
