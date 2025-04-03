/**
 * Memory Functions
 *
 * Copied from FADS ROM, Dan Malek (dmalek@jlc.net)
 */



#include "util.h"

#define MSG_EMERG      0    /* system is unusable */
#define O_RWSIZE_SHIFT	18
#define O_RWSIZE_1	001000000
#define O_RWSIZE_2	002000000
#define O_RWSIZE_4	004000000
#define O_RWSIZE_8	010000000

#define PROT_READ	1

#define RW_BUF_SIZE	(unsigned)4096

#define COMMAND_SUCCESS		0
#define COMMAND_ERROR		1
#define COMMAND_ERROR_USAGE	2

#ifdef VERBOSE_DEBUG
#define LOGLEVEL	3
#elif defined DEBUG
#define LOGLEVEL	2
#else
#define LOGLEVEL	1
#endif

#define MAP_FAILED ((void *)-1)

#define DISP_LINE_LEN	16

typedef long long loff_t;
struct va_format {
	const char *fmt;
	va_list *va;
};

extern int optind;

volatile int arm_ignore_data_abort;
volatile int arm_data_abort_occurred;


/**
 * swab64 - return a byteswapped 64-bit value
 * @x: value to byteswap
 */
unsigned long long int swab64 (unsigned long long int x) {
    unsigned long long int ret = 0;
    unsigned char c;
    unsigned char * p = (unsigned char *) &x;
	int i;
    
    for (i=0; i<7; i++) {
        c = p[i];
        ret += c;
        ret = ret << 8;
    }
	c = p[i];
    ret += c;
    return ret;
}

/**
 * swab32 - return a byteswapped 32-bit value
 * @x: value to byteswap
 */
unsigned int swab32 (unsigned int x) {
    unsigned int i, ret = 0;
    unsigned char c;
    unsigned char * p = (unsigned char *) &x;
    
    for (i=0; i<3; i++) {
        c = p[i];
        ret += c;
        ret = ret << 8;
    }
	c = p[i];
    ret += c;
    return ret;
}

/**
 * swab16 - return a byteswapped 16-bit value
 * @x: value to byteswap
 */
unsigned short int swab16 (unsigned short int x) {
    unsigned short int ret = 0;
    unsigned char * p = (unsigned char *) &x;
    
	ret += p[0];
	ret = ret << 8;
	ret += p[1];

    return ret;
}


void data_abort_mask(void)
{
	arm_data_abort_occurred = 0;
	arm_ignore_data_abort = 1;
}

int data_abort_unmask(void)
{
	arm_ignore_data_abort = 0;

	return arm_data_abort_occurred != 0;
}

int memory_display(const void *addr, loff_t offs, unsigned nbytes, int size,
		   int swab);
int __pr_memory_display(int level, const void *addr, loff_t offs, unsigned nbytes,
			int size, int swab, const char *format, ...);

#define pr_fmt(fmt) fmt
#define pr_memory_display(level, addr, offs, nbytes, size, swab) \
	({	\
		(level) <= LOGLEVEL ? __pr_memory_display((level), (addr), \
				(offs), (nbytes), (size), (swab), pr_fmt("")) : 0; \
	 })

int __pr_memory_display(int level, const void *addr, loff_t offs, unsigned nbytes,
			int size, int swab, const char *fmt, ...)
{	        
	unsigned long linebytes, i;
	unsigned char *cp;
	unsigned char line[sizeof("00000000: 0000 0000 0000 0000  0000 0000 0000 0000            ................")];
	struct va_format vaf;
	int ret;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	/* Print the lines.
	 *
	 * We buffer all read data, so we can make sure data is read only
	 * once, and all accesses are with the specified bus width.
	 */
	do {
		unsigned char linebuf[DISP_LINE_LEN];
		uint64_t *ullp = (uint64_t *)linebuf;
		uint32_t *uip = (uint32_t *)linebuf;
		uint16_t *usp = (uint16_t *)linebuf;
		uint8_t *ucp = (uint8_t *)linebuf;
		unsigned char *pos = line;

		pos += sprintf(pos, "%08llx:", offs);
		linebytes = (nbytes > DISP_LINE_LEN) ? DISP_LINE_LEN : nbytes;

		for (i = 0; i < linebytes; i += size) {
			if (size == 8) {
				uint64_t res;
				data_abort_mask();
				res = *((uint64_t *)addr);
				if (swab)
					res = swab64(res);
				if (data_abort_unmask()) {
					res = 0xffffffffffffffffULL;
					pos += sprintf(pos, " xxxxxxxxxxxxxxxx");
				} else {
					pos += sprintf(pos, " %016llx", res);
				}
				*ullp++ = res;
			} else if (size == 4) {
				uint32_t res;
				data_abort_mask();
				res = *((uint32_t *)addr);
				if (swab)
					res = swab32(res);
				if (data_abort_unmask()) {
					res = 0xffffffff;
					pos += sprintf(pos, " xxxxxxxx");
				} else {
					pos += sprintf(pos, " %08x", res);
				}
				*uip++ = res;
			} else if (size == 2) {
				uint16_t res;
				data_abort_mask();
				res = *((uint16_t *)addr);
				if (swab)
					res = swab16(res);
				if (i > 1 && i % 8 == 0)
					pos += sprintf(pos, " ");
				if (data_abort_unmask()) {
					res = 0xffff;
					pos += sprintf(pos, " xxxx");
				} else {
					pos += sprintf(pos, " %04x", res);
				}
				*usp++ = res;
			} else {
				uint8_t res;
				data_abort_mask();
				res = *((uint8_t *)addr);
				if (i > 1 && i % 8 == 0)
					pos += sprintf(pos, " ");
				if (data_abort_unmask()) {
					res = 0xff;
					pos += sprintf(pos, " xx");
				} else {
					pos += sprintf(pos, " %02x", res);
				}
				*ucp++ = res;
			}
			addr += size;
			offs += size;
		}

		pos += sprintf(pos, "%*s", (int)(61 - (pos - line)), "");

		cp = linebuf;
		for (i = 0; i < linebytes; i++) {
			if ((*cp < 0x20) || (*cp > 0x7e))
				sprintf(pos, ".");
			else
				sprintf(pos, "%c", *cp);
			pos++;
			cp++;
		}

//		if (level >= MSG_EMERG)
//			xil_printf(level, "%pV%s\n", &vaf, line);
//		else
			nc_printf("%s\n", line);
//			k_sleep(K_MSEC(20));

		nbytes -= linebytes;
		if (ctrlc()) {
			ret = -EINTR;
			goto out;
		}

	} while (nbytes > 0);

	va_end(args);
	ret = 0;
out:

	return ret;
}

int memory_display(const void *addr, loff_t offs, unsigned nbytes,
		   int size, int swab)
{
	return pr_memory_display(-1, addr, offs, nbytes, size, swab);
}

/*
 * Common function for parsing options for the 'md', 'mw', 'memcpy', 'memcmp'
 * commands.
 */
int mem_parse_options(int argc, char *argv[], char *optstr, int *mode, int *swab)
{
	int opt;

	while((opt = getopt(argc, argv, optstr)) > 0) {
		switch(opt) {
		case 'b':
			*mode = O_RWSIZE_1;
			break;
		case 'w':
			*mode = O_RWSIZE_2;
			break;
		case 'l':
			*mode = O_RWSIZE_4;
			break;
		case 'q':
			*mode = O_RWSIZE_8;
			break;
		case 'x':
			*swab = 1;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Like simple_strtoull() but handles an optional G, M, K or k
 * suffix for Gigabyte, Megabyte or Kilobyte
 */
unsigned long long strtoull_suffix(const char *str, char **endp, int base)
{
	unsigned long long val;
	char *end;

	val = strtoull(str, &end, base);

	switch (*end) {
	case 'G':
		val *= 1024;
	case 'M':
		val *= 1024;
	case 'k':
	case 'K':
		val *= 1024;
		end++;
	default:
		break;
	}

	if (strncmp(end, "iB", 2) == 0)
		end += 2;

	if (endp)
		*endp = end;

	return val;
}


unsigned long strtoul_suffix(const char *str, char **endp, int base)
{
	return strtoull_suffix(str, endp, base);
}


/*
 * This function parses strings in the form <startadr>[-endaddr]
 * or <startadr>[+size] and fills in start and size accordingly.
 * <startadr> and <endadr> can be given in decimal or hex (with 0x prefix)
 * and can have an optional G, M, K or k suffix.
 *
 * examples:
 * 0x1000-0x2000 -> start = 0x1000, size = 0x1001
 * 0x1000+0x1000 -> start = 0x1000, size = 0x1000
 * 0x1000        -> start = 0x1000, size = ~0
 * 1M+1k         -> start = 0x100000, size = 0x400
 */
int parse_area_spec(const char *str, loff_t *start, loff_t *size)
{
	char *endp;
	loff_t end, _start, _size;

	if (!isdigit(*str))
		return -1;

	_start = strtoull_suffix(str, &endp, 0);

	str = endp;

	if (!*str) {
		/* beginning given, but no size, assume maximum size */
		_size = ~0;
		goto success;
	}

	if (*str == '-') {
		/* beginning and end given */
		if (!isdigit(*(str + 1)))
			return -1;

		end = strtoull_suffix(str + 1, &endp, 0);
		str = endp;
		if (end < _start) {
			nc_printf("end < start\n");
			return -1;
		}
		_size = end - _start + 1;
		goto success;
	}

	if (*str == '+') {
		/* beginning and size given */
		if (!isdigit(*(str + 1)))
			return -1;

		_size = strtoull_suffix(str + 1, &endp, 0);
		str = endp;
		goto success;
	}

	return -1;

success:
	if (*str && !isspace(*str))
		return -1;
	*start = _start;
	*size = _size;
	return 0;
}

int mem_md(int argc, char *argv[])
{
	loff_t	start = 0, size = 0x100;
	int	ret = 0;
	int mode = O_RWSIZE_4;
	int swab = 0;
	void *map = NULL;

	if (argc < 2)
		return COMMAND_ERROR_USAGE;

	if (mem_parse_options(argc, argv, "bwlqs:x", &mode,	&swab) < 0)
		return 1;

	if (optind < argc) {
		if (parse_area_spec(argv[optind], &start, &size)) {
			nc_printf("could not parse: %s\n", argv[optind]);
			return 1;
		}
		if (size == ~0)
			size = 0x100;
	}
	/*
	if (((start>0x20040000) || (start<0x20000000) ) || ((size)>0x00040000))
	{
		nc_printf("Invalid memory (valid ram: 0x20000000-0x20040000)\n");
		return 1;
	}
	*/
	ret = memory_display(map + start, start, size, mode >> O_RWSIZE_SHIFT, swab);

	optind = 1;

	return ret ? 1 : 0;
}


