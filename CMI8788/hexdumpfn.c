//
//  hexdumpfn.c
//  PCIAudioDriver
//
//  Created by Gagan on 2018-04-23.
//  Copyright © 2018 CMedia. All rights reserved.
//

enum {
    DUMP_PREFIX_NONE,
    DUMP_PREFIX_ADDRESS,
    DUMP_PREFIX_OFFSET
};

extern const unsigned char _ctype[];
#define _U	0x01	/* upper */
#define _L	0x02	/* lower */
#define _D	0x04	/* digit */
#define _C	0x08	/* cntrl */
#define _P	0x10	/* punct */
#define _S	0x20	/* white space (space/lf/tab) */
#define _X	0x40	/* hex digit */
#define _SP	0x80	/* hard space (0x20) */

#define __ismask(x) (_ctype[(int)(unsigned char)(x)])
#define isprint(c)	((__ismask(c)&(_P|_U|_L|_D|_SP)) != 0)
#define isascii(c) (((unsigned char)(c))<=0x7f)

const char hex_asc[] = "0123456789abcdef";
extern void __bad_unaligned_access_size(void);

#define hex_asc_lo(x)	hex_asc[((x) & 0x0f)]
#define hex_asc_hi(x)	hex_asc[((x) & 0xf0) >> 4]
#define __get_unaligned_le(ptr) ((typeof(*(ptr)))({			\
__builtin_choose_expr(sizeof(*(ptr)) == 1, *(ptr),			\
__builtin_choose_expr(sizeof(*(ptr)) == 2, get_unaligned_le16((ptr)),	\
__builtin_choose_expr(sizeof(*(ptr)) == 4, get_unaligned_le32((ptr)),	\
__builtin_choose_expr(sizeof(*(ptr)) == 8, get_unaligned_le64((ptr)),	\
__bad_unaligned_access_size()))));					\
}))



static inline UInt16 __get_unaligned_le16(const UInt8 *p)
{
    return p[0] | p[1] << 8;
}

static inline UInt32 __get_unaligned_le32(const UInt8 *p)
{
    return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
}

static inline UInt64 __get_unaligned_le64(const UInt8 *p)
{
    return (UInt64)__get_unaligned_le32(p + 4) << 32 |
    __get_unaligned_le32(p);
}

static inline uint16_t get_unaligned_le16(const void *p)
{
    return __get_unaligned_le16((const uint8_t *)p);
}

static inline uint32_t get_unaligned_le32(const void *p)
{
    return __get_unaligned_le32((const uint8_t *)p);
}

static inline uint64_t get_unaligned_le64(const void *p)
{
    return __get_unaligned_le64((const uint8_t *)p);
}




bool is_power_of_2(unsigned long n)
{
    return (n != 0 && ((n & (n - 1)) == 0));
}

/**
 * hex_dump_to_buffer - convert a blob of data to "hex ASCII" in memory
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 * @rowsize: number of bytes to print per line; must be 16 or 32
 * @groupsize: number of bytes to print at a time (1, 2, 4, 8; default = 1)
 * @linebuf: where to put the converted data
 * @linebuflen: total size of @linebuf, including space for terminating NUL
 * @ascii: include ASCII after the hex output
 *
 * hex_dump_to_buffer() works on one "line" of output at a time, i.e.,
 * 16 or 32 bytes of input data converted to hex + ASCII output.
 *
 * Given a buffer of u8 data, hex_dump_to_buffer() converts the input data
 * to a hex + ASCII dump at the supplied memory location.
 * The converted output is always NUL-terminated.
 *
 * E.g.:
 *   hex_dump_to_buffer(frame->data, frame->len, 16, 1,
 *			linebuf, sizeof(linebuf), true);
 *
 * example output buffer:
 * 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f  @ABCDEFGHIJKLMNO
 *
 * Return:
 * The amount of bytes placed in the buffer without terminating NUL. If the
 * output was truncated, then the return value is the number of bytes
 * (excluding the terminating NUL) which would have been written to the final
 * string if enough space had been available.
 */
int hex_dump_to_buffer(const void *buf, size_t len, int rowsize, int groupsize,
                       char *linebuf, size_t linebuflen, bool ascii)
{
    const UInt8 *ptr = (UInt8 *) buf;
    int ngroups;
    
    UInt8 ch;
    int j, lx = 0;
    int ascii_column;
    int ret;
    
    if (rowsize != 16 && rowsize != 32)
        rowsize = 16;
    
    if (len > rowsize)		/* limit to one line at a time */
        len = rowsize;
    if (!is_power_of_2(groupsize) || groupsize > 8)
        groupsize = 1;
    if ((len % groupsize) != 0)	/* no mixed size output */
        groupsize = 1;
    
    ngroups = len / groupsize;
    ascii_column = rowsize * 2 + rowsize / groupsize + 1;
    
    if (!linebuflen)
        goto overflow1;
    
    if (!len)
        goto nil;
    
    if (groupsize == 8) {
        const UInt64 *ptr8 = (UInt64 *) buf;
        
        for (j = 0; j < ngroups; j++) {
            ret = snprintf(linebuf + lx, linebuflen - lx,
                           "%s%16.16llx", j ? " " : "",
                           __get_unaligned_le(ptr8 + j));
            if (ret >= linebuflen - lx)
                goto overflow1;
            lx += ret;
        }
    } else if (groupsize == 4) {
        const UInt32 *ptr4 = (UInt32 *) buf;
        
        for (j = 0; j < ngroups; j++) {
            ret = snprintf(linebuf + lx, linebuflen - lx,
                           "%s%8.8x", j ? " " : "",
                           __get_unaligned_le(ptr4 + j));
            if (ret >= linebuflen - lx)
                goto overflow1;
            lx += ret;
        }
    } else if (groupsize == 2) {
        const UInt16 *ptr2 = (UInt16*) buf;
        
        for (j = 0; j < ngroups; j++) {
            ret = snprintf(linebuf + lx, linebuflen - lx,
                           "%s%4.4x", j ? " " : "",
                           __get_unaligned_le(ptr2 + j));
            if (ret >= linebuflen - lx)
                goto overflow1;
            lx += ret;
        }
    } else {
        for (j = 0; j < len; j++) {
            if (linebuflen < lx + 2)
                goto overflow2;
            ch = ptr[j];
            linebuf[lx++] = hex_asc_hi(ch);
            if (linebuflen < lx + 2)
                goto overflow2;
            linebuf[lx++] = hex_asc_lo(ch);
            if (linebuflen < lx + 2)
                goto overflow2;
            linebuf[lx++] = ' ';
        }
        if (j)
            lx--;
    }
    if (!ascii)
        goto nil;
    
    while (lx < ascii_column) {
        if (linebuflen < lx + 2)
            goto overflow2;
        linebuf[lx++] = ' ';
    }
    for (j = 0; j < len; j++) {
        if (linebuflen < lx + 2)
            goto overflow2;
        ch = ptr[j];
        linebuf[lx++] = (isascii(ch) && isprint(ch)) ? ch : '.';
    }
nil:
    linebuf[lx] = '\0';
    return lx;
overflow2:
    linebuf[lx++] = '\0';
overflow1:
    return ascii ? ascii_column + len : (groupsize * 2 + 1) * ngroups - 1;
}

/**
 * print_hex_dump - print a text hex dump to syslog for a binary blob of data
 * @level: kernel log level (e.g. KERN_DEBUG)
 * @prefix_str: string to prefix each line with;
 *  caller supplies trailing spaces for alignment if desired
 * @prefix_type: controls whether prefix of an offset, address, or none
 *  is printed (%DUMP_PREFIX_OFFSET, %DUMP_PREFIX_ADDRESS, %DUMP_PREFIX_NONE)
 * @rowsize: number of bytes to print per line; must be 16 or 32
 * @groupsize: number of bytes to print at a time (1, 2, 4, 8; default = 1)
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 * @ascii: include ASCII after the hex output
 *
 * Given a buffer of u8 data, print_hex_dump() prints a hex + ASCII dump
 * to the kernel log at the specified kernel log level, with an optional
 * leading prefix.
 *
 * print_hex_dump() works on one "line" of output at a time, i.e.,
 * 16 or 32 bytes of input data converted to hex + ASCII output.
 * print_hex_dump() iterates over the entire input @buf, breaking it into
 * "line size" chunks to format and print.
 *
 * E.g.:
 *   print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_ADDRESS,
 *		    16, 1, frame->data, frame->len, true);
 *
 * Example output using %DUMP_PREFIX_OFFSET and 1-byte mode:
 * 0009ab42: 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f  @ABCDEFGHIJKLMNO
 * Example output using %DUMP_PREFIX_ADDRESS and 4-byte mode:
 * ffffffff88089af0: 73727170 77767574 7b7a7978 7f7e7d7c  pqrstuvwxyz{|}~.
 */

void print_hex_dump(const char *level, const char *prefix_str, int prefix_type,
                    int rowsize, int groupsize,
                    const void *buf, size_t len, bool ascii)
{
    const UInt8 *ptr = (UInt8 *) buf;
    int i, linelen, remaining = len;
    unsigned char linebuf[32 * 3 + 2 + 32 + 1];
    
    if (rowsize != 16 && rowsize != 32)
        rowsize = 16;
    
    for (i = 0; i < len; i += rowsize) {
        linelen = min(remaining, rowsize);
        remaining -= rowsize;
        
        hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize, (char*) linebuf, sizeof(linebuf), ascii);
        
        switch (prefix_type) {
            case DUMP_PREFIX_ADDRESS:
                kprintf("%s%s%p: %s\n",
                        level, prefix_str, ptr + i, linebuf);
                break;
            case DUMP_PREFIX_OFFSET:
                kprintf("%s%s%.8x: %s\n", level, prefix_str, i, linebuf);
                break;
            default:
                kprintf("%s%s%s\n", level, prefix_str, linebuf);
                break;
        }
    }
}
/**
 * print_hex_dump_bytes - shorthand form of print_hex_dump() with default params
 * @prefix_str: string to prefix each line with;
 *  caller supplies trailing spaces for alignment if desired
 * @prefix_type: controls whether prefix of an offset, address, or none
 *  is printed (%DUMP_PREFIX_OFFSET, %DUMP_PREFIX_ADDRESS, %DUMP_PREFIX_NONE)
 * @buf: data blob to dump
 * @len: number of bytes in the @buf
 *
 * Calls print_hex_dump(), with log level of KERN_DEBUG,
 * rowsize of 16, groupsize of 1, and ASCII output included.
 */
void print_hex_dump_bytes(const char *prefix_str, int prefix_type,
                          const void *buf, size_t len)
{
    print_hex_dump("7", prefix_str, prefix_type, 16, 1,
                   buf, len, true);
}
