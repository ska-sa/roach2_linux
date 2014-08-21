#ifndef ROACH_COMMON_H_
#define ROACH_COMMON_H_

#define ROACH_DEV_MAGIC 0xdeadbeef

#ifdef PDEBUG
#  define PDEBUG(lvl, fmt, args...) printk(KERN_DEBUG roach ": " fmt, ## args)
#endif


/* Reference Purpose Only */
#define GPIO_SMAP_INITN 12
#define GPIO_SMAP_DONE  13
#define GPIO_SMAP_PROGN 14
#define GPIO_SMAP_RDWRN 15
#define GPIO_SMAP_GPIO0 28
#define GPIO_SMAP_GPIO1 27
#define GPIO_SMAP_GPIO2 30
#define GPIO_SMAP_GPIO3 31
#define GPIO_SMAP_LED   29


/* SMAP Control register definitions */
/* Delay fors smap checks */
#define SMAP_INITN_WAIT 100000
#define SMAP_DONE_WAIT  100000


/* Default SX475T image size in bytes */
#define SMAP_IMAGE_SIZE 19586188
#define V6_XQ6VSX475T_ID 0x04288093

/* 128MB */
#define ROACH_FPGA_LENGTH       0x08000000


#endif /* ROACH_COMMON_H_ */






