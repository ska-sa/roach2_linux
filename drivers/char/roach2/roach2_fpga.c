/*
  * ROACH2 memory mapped FPGA access driver
  *
  * Copyright (c) 2012 Shanly Rajan <shanly.rajan@ska.ac.za>
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
 */

/*
 * Basic Description:
 * This driver is used to configure and access the FPGA on the 
 * ROACH2 hardware board using memory mapped approach. 
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mm.h>


#include <asm/uaccess.h>  

#ifdef CONFIG_OF
/* For open firmware. */
//#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#endif

#include "roach2_fpga.h"        /* local definitions */
#define DRIVER_NAME "roach2_fpga"
#define ROACH2_NR_DEVS 2

static int roach_major = 0;
static int roach_minor = 0;
static int tmp = 1;
uint32_t fpga_device_id;

static DEFINE_MUTEX(roach2_mutex);

#define DRIVER_NAME "roach2_fpga"
#define ROACH2_NR_DEVS 2

#define DBG(x...) \
        pr_debug(DRIVER_NAME ": " x)
#define DBGF(f, x...) \
        pr_debug(DRIVER_NAME " [%s()]: " f, __func__ , ##x)

struct fpga_device {
        /* character device */
        struct device *dev;
        struct cdev rdev[ROACH2_NR_DEVS];
        dev_t devt;

        /* FPGA Physical Address/Size Information */
        resource_size_t fpga_addr;
        resource_size_t fpga_size;
        void __iomem   *fpga_virt;

        /* SMAP Physical Address/Size Information */
        resource_size_t smap_addr;
        resource_size_t smap_size;
        void __iomem   *smap_virt;
        
        /* Protection device access*/
        bool is_config_open;
        bool is_mem_open;
        struct mutex mutex;

        /* tx and rx buffers*/
        void *tx_buf;
        void *rx_buf;

        /* gpio signals*/
        unsigned int initn;
        unsigned int done;
        unsigned int progn;
        unsigned int rdwrn;

        /* Gateware bytes count */
        unsigned int gw_bytes;
};

/*
 * ROACH2 gpio setup
 */

/*
 * This function allows selectmap routines to use ppc gpios on the ROACH2 board
 * GPIO pins inherit their location from the OF device tree entries in roach2.dts 
 */

static int roach2_setup_gpios(struct fpga_device *fdev)
{
        struct device_node *np, *child;
        int i;

        np = of_find_compatible_node(NULL, NULL, "gpio-pins");
        if (!np) {
                printk(KERN_ERR __FILE__ ": Unable to find pins\n");
                return -ENOENT;
        }

        for_each_child_of_node(np, child)
                if (strcmp(child->name, "init_n") == 0)
                        fdev->initn = of_get_gpio(child, 0);

                else if (strcmp(child->name, "done") == 0)
                        fdev->done = of_get_gpio(child, 0);

                else if (strcmp(child->name, "prog_n") == 0)
                        fdev->progn = of_get_gpio(child, 0);

                else if (strcmp(child->name, "rdwr_n") == 0)
                        fdev->rdwrn = of_get_gpio(child, 0);


        of_node_put(np);

        gpio_set_value(fdev->rdwrn, 0);
        gpio_set_value(fdev->initn, 1);
        gpio_set_value(fdev->progn, 1);
        wmb();

        for (i = 0; i < 32; i++) { /* Delay for at least 350ns  */
                /* Set Write mode, and disable init_n and prog_n pins */
                gpio_set_value(fdev->initn, 0);
                gpio_set_value(fdev->progn, 0);
        }
        wmb();
        /* Clear prog_n */
        gpio_set_value(fdev->progn, 1);
        wmb();


        /* Here we need to wait for initn to be driven to one by the v6 */
        for (i = 0; i < SMAP_INITN_WAIT + 1; i++) {
                if (gpio_get_value(fdev->initn)) {
                        break;
                }
                if (i == SMAP_INITN_WAIT) {
                        printk(KERN_INFO "SelectMap - Init_n pin has not been asserted\n");
                        return -EIO;
                }
        }

        return 0;
}

/*
 * FPGA Configuration Code
 */

static int roach_config_open(struct inode *inode, struct file *filp)
{
        struct fpga_device *rdev_config;
        int status = 0;

        mutex_lock(&roach2_mutex);
        rdev_config = container_of(inode->i_cdev, struct fpga_device, rdev[0]);
 
        printk(KERN_NOTICE "roach open config called\n");

        status = mutex_lock_interruptible(&rdev_config->mutex);
        if(status)
                goto out;

        /* WARNING */
        if (rdev_config->is_mem_open > 0) {
                printk(KERN_WARNING "Another process busy accessing FPGA, proceed with caution reprogramming FPGA\n");
        }

        if (rdev_config->is_config_open) {
                status = -EBUSY;
                goto error;
        }

        filp->private_data = rdev_config;
        rdev_config->is_config_open = 1;

        status = roach2_setup_gpios(rdev_config);
        if (status)
                goto error;

        printk(KERN_NOTICE "rdev gpio preconfig done\n");

        rdev_config->gw_bytes = 0;

error:
        mutex_unlock(&rdev_config->mutex);
out:
        mutex_unlock(&roach2_mutex);

        return status;
}

/*
 * Programs the FPGA device given an input buffer holding the bit-file,
 * through the selectmap interface
 */

ssize_t roach_config_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *f_pos)
{
        int i;
        ssize_t retval = -EIO;
        uint32_t word_count;
        volatile uint32_t *src;
        size_t have_written, will_write = 0;
        struct fpga_device *fdev = filp->private_data;

        retval = mutex_lock_interruptible(&fdev->mutex);
        if (retval)
                return retval;

        have_written = 0; /* the number of bytes which we were given which we have written */
        if (fdev->gw_bytes >= SMAP_IMAGE_SIZE) {
                printk(KERN_WARNING "request for way too much data, gateware size %u\n", SMAP_IMAGE_SIZE);
                have_written = -EINVAL;
                goto out_free_mutex;
        }
        while (have_written < cnt) {
                will_write = cnt - have_written;
                if (will_write > PAGE_SIZE)
                        will_write = PAGE_SIZE;

                retval = copy_from_user((uint32_t *)(fdev->tx_buf), (uint32_t *)(buf + have_written), will_write);
                if (retval != 0){
                        printk(KERN_WARNING "copy from user failed with code %d, at %u/%u in write, %u bytes programmed so far\n", retval, have_written, cnt, fdev->gw_bytes + have_written);
                        goto out_free_mutex;
                }

                if ((fdev->gw_bytes + have_written + will_write) > SMAP_IMAGE_SIZE) {
                        if ((fdev->gw_bytes + have_written) < SMAP_IMAGE_SIZE) {
                                printk(KERN_WARNING "truncating buffer to %u as total exceeds gatware size %u\n", will_write, SMAP_IMAGE_SIZE);
                                will_write = SMAP_IMAGE_SIZE - (fdev->gw_bytes + have_written);
                        } else {
                                printk(KERN_WARNING "writing too much data, want %u, gateware size %u\n", fdev->gw_bytes + have_written + will_write, SMAP_IMAGE_SIZE);
                                will_write = 0;
                                break; /* WARNING: end loop, but check if we need to do GPIO */
                        }
                }
                word_count = will_write / 4;
                src = fdev->tx_buf;

                if(tmp == 1){
                        printk(KERN_INFO "Programmed fpga device id = %08x\n", *(src+32));
                        fpga_device_id = *(src + 32);
                        tmp++;
                }

                for (i = 0; i < word_count; i++) {
                        out_be32((uint32_t *)(fdev->smap_virt), *src);
                        src++;
                }

                if ((i * 4) < will_write) {
                        printk(KERN_WARNING "request to write %u was not a multiple of 4, only writing %u\n", i * 4, will_write);
                        have_written += (i * 4);
                        break; /* WARNING: assumes that this only occurs for last few bytes of cnt */
                }

                have_written += will_write;
        }

        fdev->gw_bytes += have_written;

        /* Virtex6 device:XQ6VSX475T = 0x04288093 */
        /* FPGA bitfiles have an exact size */
        if ((fdev->gw_bytes == SMAP_IMAGE_SIZE) && (fpga_device_id == V6_XQ6VSX475T_ID)) {
                /* Poll until done pin is enabled */
                for (i = 0; (i < SMAP_DONE_WAIT) && (gpio_get_value(fdev->done) == 0); i++);

                if (i == SMAP_DONE_WAIT) {
                        printk(KERN_ERR "error: SelectMAP programming failed, done stuck low\n");
                        have_written = -EIO;
                } else {
                        printk(KERN_INFO "ROACH2 Virtex-6 configuration completed successfully\n");
                }

                goto out_free_mutex;
        }
        else{
                if(fpga_device_id != V6_XQ6VSX475T_ID){
                        printk(KERN_ERR "Attempted to program incorrect configuration onto Virtex6 FPGA\n");
                        have_written = -EIO;

                }
        }
        retval = have_written;
out_free_mutex:
        mutex_unlock(&fdev->mutex);
        return have_written;
}


static int roach_config_release(struct inode *inode, struct file *filp)
{
        struct fpga_device *rdev_config = filp->private_data;
        int status = 0;
        tmp = 1;

        printk(KERN_NOTICE "roach release config called\n");

        mutex_lock(&rdev_config->mutex);

        rdev_config->is_config_open = 0;

        mutex_unlock(&rdev_config->mutex);
        return status;
}

/*
 * FPGA Access Code 
 */

static int roach_mem_open(struct inode *inode, struct file *filp)
{
        struct fpga_device *rdev_mem;
        int status = 0;

        mutex_lock(&roach2_mutex);

        rdev_mem = container_of(inode->i_cdev, struct fpga_device, rdev[1]);

        status = mutex_lock_interruptible(&rdev_mem->mutex);
        if (status)
                goto out;

        filp->private_data = rdev_mem;

        /* Multiple access allowed */
        rdev_mem->is_mem_open++;

        mutex_unlock(&rdev_mem->mutex);
out:
        mutex_unlock(&roach2_mutex);

        return status;
}

/*
 * Write a set of bytes to the FPGA memory space from user buffer.
 */

static ssize_t roach_mem_write(struct file *filp, const char __user *buf,
                               size_t cnt, loff_t *f_pos)
{
        size_t have_written, will_write = 0;
        uint32_t word_count;
        volatile uint32_t *src;
        uint32_t offset;
        ssize_t retval = -EIO;
        int i;
        struct fpga_device *fdev = filp->private_data;

        retval = mutex_lock_interruptible(&fdev->mutex);
        if (retval)
                return retval;

        /* WARNING */
        if (fdev->is_mem_open <= 0) {
                printk(KERN_WARNING "Logic problem in accessing IO\n");
        }

        have_written = 0; /* the number of bytes which we were given which we have written */

        while (have_written < cnt) {
                will_write = cnt - have_written;
                if (will_write > PAGE_SIZE)
                        will_write = PAGE_SIZE;

                retval = copy_from_user((uint32_t *)(fdev->tx_buf), (uint32_t *)(buf + have_written), will_write);
                if (retval != 0){
                        retval = -EFAULT;
                        goto out_free_mutex;
                }

                word_count = will_write / 4;
                src = fdev->tx_buf;
                offset = *f_pos;
                for (i = 0; i < word_count; i++) {
                        out_be32((uint32_t *)(fdev->fpga_virt + offset), *src);
                        printk(KERN_INFO "request to write to pos=0x%p+offset=%u data_sample=0x%x\n", fdev->fpga_virt, offset, *src);
                        offset += 4;
                        src++;
                }

                if ((i * 4) < will_write) {
                        printk(KERN_WARNING "request to write %u was not a multiple of 4, only writing %u\n", will_write, i * 4);
                        have_written += (i * 4);
                        break; /* WARNING: assumes that this only occurs for last few bytes of cnt */
                }

                *f_pos = offset;

                have_written += will_write;
        }
        retval = have_written;

out_free_mutex:
        mutex_unlock(&fdev->mutex);
        return retval;  

}

/*
 * Read a set of bytes from the FPGA memory space to user buffer.
 */
static ssize_t roach_mem_read(struct file *filp, char __user *buf, 
                              size_t cnt, loff_t *f_pos)
{
        size_t have_read, will_read = 0;
        uint32_t word_count, offset = 0;
        volatile uint32_t *src;
        ssize_t retval = -EIO;
        int i;
        struct fpga_device *fdev = filp->private_data;

        retval = mutex_lock_interruptible(&fdev->mutex);
        if (retval)
                return retval;

        have_read = 0; /* the number of bytes which we were given which we have written */
        offset = *f_pos;

        while (have_read < cnt) {
                will_read = cnt - have_read;
                if (will_read > PAGE_SIZE)
                        will_read = PAGE_SIZE;

                word_count = will_read / 4;
                src = fdev->rx_buf;

                for (i = 0; i < word_count; i++) {
                        *src = in_be32((uint32_t *)(fdev->fpga_virt + offset));
                        printk(KERN_INFO "sample read from 0x%p+%u is 0x%x\n", fdev->fpga_virt, offset, *src);
                        src++;
                        offset += 4;
                }

                retval = copy_to_user((uint32_t *)(buf + have_read),(uint32_t *)(fdev->rx_buf), will_read);
                if (retval != 0) {
                        retval = -EFAULT;
                        goto out_free_mutex;
                }

                have_read += will_read;
        }

        *f_pos = offset;
        retval = have_read;

out_free_mutex:
        mutex_unlock(&fdev->mutex);
        return retval;
}

static loff_t roach_mem_llseek(struct file *filp, loff_t offset, int origin)
{

        loff_t newpos;

        /* only read-only opens are allowed to seek */
        if ((filp->f_flags & O_ACCMODE) != O_RDONLY)
                return -EINVAL;

        switch (origin) {
        case SEEK_SET: /* seek relative to the beginning of the file */
                newpos = offset;
                break;
        case SEEK_CUR: /* seek relative to current position in the file */
                newpos = filp->f_pos + offset;
                break;
        case SEEK_END: /* seek relative to the end of the file */
                newpos = ROACH_FPGA_LENGTH - offset;
                break;
        default:
                return -EINVAL;
        }

        filp->f_pos = newpos;
        return newpos;
}

/*
 * Common roach VMA ops.
 */

void roach_vma_open(struct vm_area_struct *vma)
{
        printk(KERN_NOTICE "roach VMA open, virt %lx, phys %lx\n",
                        vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void roach_vma_close(struct vm_area_struct *vma)
{
        printk(KERN_NOTICE "roach VMA close\n");
}

static struct vm_operations_struct roach_remap_vm_ops = {
        .open = roach_vma_open,
        .close = roach_vma_close,
};

/*
 * Simple memory mapped interface for FPGA access
 * Memory map the FPGA to the user process address space in the PowerPC
 */

static int roach_mem_mmap(struct file *file, struct vm_area_struct *vma)
{
        unsigned long start, vsize, offset;
        unsigned long page, pos;
        struct fpga_device *rdev_mem = file->private_data;


        /* VMA properties */
        start = vma->vm_start;
        vsize = vma->vm_end - vma->vm_start;
        offset = vma->vm_pgoff << PAGE_SHIFT;

        printk(KERN_NOTICE "%s: vm_start %lx, vm_end %lx, vsize %lx, offset %lx\n",
                        __func__, vma->vm_start, vma->vm_end, (vma->vm_end - vma->vm_start), vma->vm_pgoff << PAGE_SHIFT);

        if (offset + vsize > ROACH_FPGA_LENGTH)
                return -EINVAL;


        pos = (unsigned long)(rdev_mem->fpga_virt) + offset;

        printk(KERN_NOTICE "%s: pos to be converted : %lx\n", __func__, pos);


        /* Avoid to swap out this VMA */
        /* vma->vm_flags |= VM_RESERVED; */
        /* VM_RESERVED replacement with VM_IO that accounts as reserved_vm */
        vma->vm_flags |= VM_IO;


        /* Page caching disabled completely for memory mapping to work properly*/
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        while (vsize > 0) {
                page = vmalloc_to_pfn((void *)pos);
                if (remap_pfn_range(vma, start, page, PAGE_SIZE, vma->vm_page_prot))
                        return -EAGAIN;

                start += PAGE_SIZE;
                pos += PAGE_SIZE;
                if (vsize > PAGE_SIZE)
                        vsize -= PAGE_SIZE;
                else
                        vsize = 0;
        }

        vma->vm_ops = &roach_remap_vm_ops;
        roach_vma_open(vma);

        return 0;
}

static int roach_mem_release(struct inode *inode, struct file *filp)
{
        struct fpga_device *rdev_mem = filp->private_data;
        int status = 0;

        printk(KERN_NOTICE "roach release mem called\n");

        mutex_lock(&rdev_mem->mutex);

        rdev_mem->is_mem_open--;

        mutex_unlock(&rdev_mem->mutex);
        return status;
}

/* ROACH2 Subdevices 
 * dev/roach/config - configuring FPGA with bitstream
 * dev/roach/mem    - reading,writing and memory-mapped FPGA interface
 */


static struct file_operations r2config_fops = {
        .owner          = THIS_MODULE,
        .open           = roach_config_open,
        .write          = roach_config_write,
        .release        = roach_config_release,
};

static struct file_operations r2mem_fops = {
        .owner          = THIS_MODULE,
        .open           = roach_mem_open,
        .read           = roach_mem_read,
        .write          = roach_mem_write,
        .llseek         = roach_mem_llseek,
        .release        = roach_mem_release,
        .mmap           = roach_mem_mmap,
};

/*
 * Setup ROACH2 sub character devices
 */
static int roach_setup_cdevs(struct device *dev, struct fpga_device *fdev)
{
        int err; 
        fdev->devt = MKDEV(roach_major, 0);
        cdev_init(fdev->rdev, &r2config_fops);
        fdev->rdev[0].owner = THIS_MODULE;
        err = cdev_add(fdev->rdev, fdev->devt, 1);
        if (err) {
                dev_err(dev, "cdev_add() failed");
                goto out;
        }
        fdev->devt = MKDEV(roach_major, 1);
        cdev_init(fdev->rdev + 1, &r2mem_fops);
        fdev->rdev[1].owner = THIS_MODULE;
        err = cdev_add(fdev->rdev + 1, fdev->devt, 1);
        if (err) {
                dev_err(dev, "cdev_add() failed");
                goto out_free_cdev;
        }
        dev_info(dev, "char devices successfully registered");
        return 0;
out_free_cdev:
        cdev_del(fdev->rdev);
out:
        return err;
}

static int r2fpga_setup(struct device *dev, const struct resource *res)
{
        dev_t devt;
        struct fpga_device *fdev = NULL;
        struct device_node *of_node;
        struct resource res1;
        int retval = 0;

        dev_info(dev, "ROACH2 FPGA driver");

        mutex_lock(&roach2_mutex);

        devt = MKDEV(roach_major, roach_minor);

        /* Allocate private data */
        fdev = kzalloc(sizeof(struct fpga_device), GFP_KERNEL);
        if (!fdev) {
                dev_err(dev, "Unable to allocate device private data");
                retval = -ENOMEM;
                goto out_free_mutex;
        }

        memset(fdev, 0, sizeof(struct fpga_device));


        dev_set_drvdata(dev, (void *)fdev);


        /* Get the physical address of the FPGA registers */
        fdev->fpga_addr = res->start;
        fdev->fpga_size  = resource_size(res);

        
        if (!request_mem_region(fdev->fpga_addr,
                                fdev->fpga_size, DRIVER_NAME)) {
                dev_err(dev, "Could't lock FPGA memory region at %Lx",
                                (unsigned long long) fdev->fpga_addr);
                retval = -EBUSY;
                goto out_free_fpga_device;
        }

        
        dev_info(dev,"check parameters,addr:0x%llx,size:0x%llx",
        (unsigned long long)fdev->fpga_addr, (unsigned long long)fdev->fpga_size);

        fdev->devt = devt;
        fdev->dev = dev;

        fdev->fpga_virt = ioremap(fdev->fpga_addr, fdev->fpga_size);
        if (!fdev->fpga_virt) {
                dev_err(dev, "fpga ioremap() failed");
                goto out_free_fpga_mem;
        }

        dev_info(dev, "ioremap %llx to %p with size %llx",
                        (unsigned long long) fdev->fpga_addr,
                        fdev->fpga_virt,
                        (unsigned long long) fdev->fpga_size);

        of_node = of_find_node_by_name(NULL, "smap");
        if (of_node == NULL) {
                dev_err(dev, "Unable to find smap node");
                retval = -ENODEV;
                goto out_free_fpga;
        }

        /* TODO: Ideally this should be under of_probe, ask MARC */
        /* Get the physical address of the SMAP registers */
        retval = of_address_to_resource(of_node, 0, &res1);
        if (retval) {
                dev_err(dev, "Unable to find SMAP physical address");
                retval = -ENODEV;
                goto out_free_fpga;
        }

        fdev->smap_addr = res1.start;
        fdev->smap_size = resource_size(&res1);

        if (!request_mem_region(fdev->smap_addr,
                                fdev->smap_size, DRIVER_NAME)) {
                dev_err(dev, "Could't lock SMAP memory region at %Lx",
                                (unsigned long long) fdev->smap_addr);
                retval = -EBUSY;
                goto out_free_fpga;
        }

        fdev->smap_virt = ioremap(fdev->smap_addr, fdev->smap_size);
        if (!fdev->smap_virt) {
                dev_err(dev, "smap ioremap() failed\n");
                goto out_free_smap_mem;
        }
        dev_info(dev, "ioremap %llx to %p with size %llx",
                        (unsigned long long) fdev->smap_addr,
                        fdev->smap_virt,
                        (unsigned long long) fdev->smap_size);

        mutex_init(&fdev->mutex);
        fdev->is_config_open = 0;
        fdev->is_mem_open = 0;

        /*Setting up the two char devs*/

        devt = MKDEV(roach_major, 0);

        retval = roach_setup_cdevs(dev, fdev);
        if(retval)
                goto out_free_smap;

        fdev->tx_buf = (void *) __get_free_page(GFP_KERNEL);
        if (!fdev->tx_buf) {
                dev_err(dev, "fdev: failed getting memory for roach TX buf");
                retval = -ENOMEM;
                goto out_free_cdev;
        }

        fdev->rx_buf = (void *) __get_free_page(GFP_KERNEL);
        if (!fdev->rx_buf) {
                dev_err(dev, "fdev: failed getting memory for roach RX buf");
                retval = -ENOMEM;
                goto out_free_tx;
        }

        /*TODO: Check whether this step needed*/
        dev_set_drvdata(dev, fdev);

        retval = 0;
        goto out_free_mutex;

out_free_tx:
        free_page((int)fdev->tx_buf);
        fdev->tx_buf = NULL;
out_free_cdev:
        cdev_del(fdev->rdev);
        cdev_del(fdev->rdev + 1);
out_free_smap:
        iounmap(fdev->smap_virt);
out_free_smap_mem:
        release_mem_region(fdev->smap_addr, fdev->smap_size);
out_free_fpga:
        iounmap(fdev->fpga_virt);
out_free_fpga_mem:
        release_mem_region(fdev->fpga_addr, fdev->fpga_size);
out_free_fpga_device:
        kfree(fdev);
out_free_mutex:
        mutex_unlock(&roach2_mutex);
        return retval;

}

static int r2fpga_remove(struct device *dev)
{
        struct fpga_device *drvdata;

        drvdata = (struct fpga_device *)dev_get_drvdata(dev);

        if (!drvdata)
                return 0;

        cdev_del(drvdata->rdev);
        cdev_del(drvdata->rdev + 1);

        iounmap(drvdata->smap_virt);
        release_mem_region(drvdata->smap_addr, drvdata->smap_size);
        iounmap(drvdata->fpga_virt);
        release_mem_region(drvdata->fpga_addr, drvdata->fpga_size);
        if (drvdata->tx_buf) {
                free_page((int)drvdata->tx_buf);
                drvdata->tx_buf = NULL;
        }
        if (drvdata->rx_buf) {
                free_page((int)drvdata->rx_buf);
                drvdata->rx_buf = NULL;
        }
        kfree(drvdata);
        dev_set_drvdata(dev, NULL);
        dev_info(dev, "ROACH2 FPGA driver removed");

        return 0;               /* success */
}


#ifdef CONFIG_OF
static int r2fpga_of_probe(struct platform_device *op)
{
        struct resource res;
        const char *family;
        int rc;

        rc = of_address_to_resource(op->dev.of_node, 0, &res);
        if (rc) {
                dev_err(&op->dev, "Unable to find FPGA physical address");
                return rc;
        }
        family = of_get_property(op->dev.of_node, "xlnx,family", NULL);

        if (family) {
                if (!strcmp(family, "virtex5")) {
                        printk(KERN_INFO "Virtex 5 family\n");
                }
        }
        
        return r2fpga_setup(&op->dev, &res);
}
#else
static inline int r2fpga_of_probe(struct platform_device *op)
{
        return -EINVAL;
}
#endif /* CONFIG_OF */


static const struct of_device_id r2fpga_of_match[];

static int r2fpga_drv_probe(struct platform_device *pdev)
{
        const struct of_device_id *match;
        struct resource *res;

        match = of_match_device(r2fpga_of_match, &pdev->dev);
        if (match)
                return r2fpga_of_probe(pdev);

        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!res)
                return -ENODEV;

        return r2fpga_setup(&pdev->dev, res);

}

static int r2fpga_drv_remove(struct platform_device *pdev)
{
        return r2fpga_remove(&pdev->dev);

}

#ifdef CONFIG_OF
/* Match table for device tree binding */
static const struct of_device_id r2fpga_of_match[] = {
        { 
        .type = "fpga", 
        .compatible = "kat,roach2-fpga", 
        },
        {},
};
MODULE_DEVICE_TABLE(of, r2fpga_of_match);
#else
#define r2fpga_of_match NULL
#endif

static struct platform_driver r2fpga_platform_driver = {
        .probe          = r2fpga_drv_probe,
        .remove         = r2fpga_drv_remove,
        .driver         = {
                .name           = DRIVER_NAME,
                .of_match_table = r2fpga_of_match,
                .owner          = THIS_MODULE,
        },
};

/*
 * Module Init / Exit
 */

static int __init r2fpga_drv_init(void)
{
        dev_t devt = 0;
        int retval = 0;


        printk(KERN_INFO DRIVER_NAME ": ROACH2 FPGA Access driver\n");

        if (roach_major) {
                retval = register_chrdev_region(devt, 1, DRIVER_NAME);
        } else {
                retval = alloc_chrdev_region(&devt, roach_minor, ROACH2_NR_DEVS , DRIVER_NAME);
                roach_major = MAJOR(devt);
        }

        if (retval < 0) {
                printk(KERN_WARNING "roach2: Unable to get major %d\n", roach_major);
                goto out;
        }

        if (roach_major == 0)
                roach_major = retval;

        printk(KERN_INFO "roach2:  major %d\n", roach_major);

        devt = MKDEV(roach_major, 0);

        retval = platform_driver_register(&r2fpga_platform_driver);
        if (retval < 0)
                goto fail;

        return 0;
fail:
        unregister_chrdev_region(devt, ROACH2_NR_DEVS);
out:
        return retval;
}

static void __exit r2fpga_drv_exit(void)
{
        dev_t devt = MKDEV(roach_major, 0);

        platform_driver_unregister(&r2fpga_platform_driver);

        unregister_chrdev_region(devt, ROACH2_NR_DEVS);

        DBG("unloaded");

}

module_init(r2fpga_drv_init);
module_exit(r2fpga_drv_exit);

MODULE_AUTHOR("Shanly Rajan <shanly@ska.ac.za>");
MODULE_DESCRIPTION("FPGA Access Driver for ROACH2");
MODULE_LICENSE("GPL");


