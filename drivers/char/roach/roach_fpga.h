#ifndef ROACH_COMMON_H_
#define ROACH_COMMON_H_

#define ROACH_DEV_MAGIC 0xfeedbeef

#ifdef PDEBUG
#  define PDEBUG(lvl, fmt, args...) printk(KERN_DEBUG roach ": " fmt, ## args)
#endif

/* Some device specific definitions */
#define ROACH_CPLD_BASE         0x1C0000000ull
#define ROACH_SMAP_BASE         0x1C0100000ull
#define ROACH_FPGA_BASE         0x1D0000000ull

#define ROACH_CPLD_LENGTH       0x000100000
#define ROACH_SMAP_LENGTH       0x000100000
#define ROACH_FPGA_LENGTH       0x008000000

/*
 * Definitions of exported memory layout.
 * The Read and Write methods map the dive spaces into a common linear address range mapping
 * The FPGA Start at offset 0 to 0x7FFFFFF, the SMAP start at offset 0x00800000 to 0x080FFFFF
 * the CPLD addresses are mapped to 0x08100000 - 0x0801FFFFF
 */
#define ROACH_FPGA_OFFSET 0
#define ROACH_SMAP_OFFSET 0x08000000
#define ROACH_CPLD_OFFSET 0x08100000

/* SMAP Control register definitions */
#define CPLD_SM_STATUS  0x8
#define CPLD_SM_OREGS   0x9
#define CPLD_SM_DATA    0xa
#define CPLD_SM_CTRL    0xb

#define CPLD_SM_DONE    0x2
#define CPLD_SM_INIT_N  0x1

#define SM_INITB_WAIT 100000
#define SM_DONE_WAIT  100000


/* Default SX475T image size in bytes */
#define SMAP_IMAGE_SIZE 19586188
#define V5_XC5VSX95T_ID 0x2ECE093

#endif /* ROACH_COMMON_H_ */






