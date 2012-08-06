/*
 * Driver for PandaDAQ data acquisition platform
 *
 * Copyright (C) 2010 Ben Gamari
 *
 * Author: Ben Gamari <bgamari.foss@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ioport.h>
#include <plat/gpmc.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/spi/spi.h>
#include "mux.h"

#include "pandadaq.h"

#define DRIVER_NAME "pandadaq"

#define PANDADAQ_WINDOW_SIZE SZ_128K

struct pandadaq_data {
        struct device *dev;
        struct cdev cdev;
        unsigned long phys_base;
        void *base;
};

static struct pandadaq_data *pandadaq;

static int __init pandadaq_init_gpmc(struct pandadaq_data *pdaq)
{
	int ret;
	uint32_t l;
	struct gpmc_timings t;

	memset(&t, 0, sizeof(t));
	t.sync_clk = 10*1000;
	t.cs_on = 1;
	t.adv_on = 1;
		
	/* Read timings */
	t.adv_rd_off = 1;
	t.oe_on = 1;
	t.access = 1;
	t.oe_off = 1;
	t.cs_rd_off = 1;
	t.rd_cycle = 1;
		
	/* Write timings */
	t.adv_wr_off = 1;
	t.we_on = 1;
	t.we_off = 1;
	t.cs_wr_off = 1;
	t.wr_cycle = 1;
	
	ret = gpmc_cs_request(0, (1<<16)-1, &pdaq->phys_base);
	if (ret)
		return ret;
	
	gpmc_cs_configure(0, GPMC_CONFIG_DEV_TYPE, GPMC_DEVICETYPE_NOR);
	gpmc_cs_configure(0, GPMC_CONFIG_DEV_SIZE, 1); /* 16-bits */

	l = gpmc_cs_read_reg(0, GPMC_CS_CONFIG1);
	l |= GPMC_CONFIG1_FCLK_DIV4;
	l |= GPMC_CONFIG1_READTYPE_SYNC;
	l |= GPMC_CONFIG1_WRITETYPE_SYNC;
        l &= ~GPMC_CONFIG1_WAIT_READ_MON;
	gpmc_cs_write_reg(0, GPMC_CS_CONFIG1, l);
	
	ret = gpmc_cs_set_timings(0, &t);
	if (ret)
		goto out_free_cs;
	return 0;
	
out_free_cs:
	gpmc_cs_free(0);

	return ret;
}

static int pandadaq_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
        return VM_FAULT_SIGBUS; /* Prevent mremap */
}

static const struct vm_operations_struct pandadaq_vmops = {
        .fault       = pandadaq_fault,
};

static int pandadaq_mmap(struct file *filp, struct vm_area_struct *vma)
{
        struct pandadaq_data *pdaq = filp->private_data;
        size_t vsize = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
        unsigned long physical = (unsigned long) pdaq->phys_base + offset;
        size_t psize = PANDADAQ_WINDOW_SIZE - offset;

        if (vsize > psize)
                return -EINVAL;
    
	if (remap_pfn_range(vma,
                            vma->vm_start,
                            physical,
                            vsize,
                            vma->vm_page_prot)) {
		return -EAGAIN;
        }

        vma->vm_ops = &pandadaq_vmops;
	return 0;
}

static int pandadaq_open(struct inode * inode, struct file * filp)
{
        filp->private_data = pandadaq;
        return 0;
}

static inline unsigned long size_inside_page(unsigned long start,
					     unsigned long size)
{
	unsigned long sz;

	sz = PAGE_SIZE - (start & (PAGE_SIZE - 1));

	return min(sz, size);
}

static ssize_t pandadaq_read(struct file *filp, char __user *buf,
                             size_t count, loff_t *ppos)
{
        struct pandadaq_data *pdaq = filp->private_data;
        unsigned long p = *ppos;
        ssize_t read, sz;
        char *ptr;

        read = 0;

        while (count > 0) {
                unsigned long remaining;

                sz = size_inside_page(p, count);

                if (p > PANDADAQ_WINDOW_SIZE)
                        return -EFAULT;

                ptr = xlate_dev_kmem_ptr(p + pdaq->base);
                if (!ptr)
                        return -EFAULT;
                
                remaining = copy_to_user(buf, ptr, sz);
                if (remaining)
                        return -EFAULT;

                buf += sz;
                p += sz;
                count -= sz;
                read += sz;
        }

        *ppos += read;
        return read;
}

ssize_t pandadaq_write(struct file *filp, const char __user *buf,
                       size_t count, loff_t *ppos)
{
        struct pandadaq_data *pdaq = filp->private_data;
        unsigned long p = *ppos;
        ssize_t written, sz;
        unsigned long copied;
        void *ptr;

        written = 0;
        while (count > 0) {
                sz = size_inside_page(p, count);

                if (p > PANDADAQ_WINDOW_SIZE)
                        return -EFAULT;

		ptr = xlate_dev_kmem_ptr(p + pdaq->base);
                if (!ptr) {
                        if (written)
                                break;
                        return -EFAULT;
                }
                
                copied = copy_from_user(ptr, buf, sz);
                if (copied) {
                        written += sz - copied;
                        if (written)
                                break;
                        return -EFAULT;
                }

                buf += sz;
                p += sz;
                count -= sz;
                written += sz;
        }

        *ppos += written;
        return written;
}

static void __init panda_config_mcspi1_mux(void)
{
        // NOTE: Clock pins need to be in input mode
	omap_mux_init_signal("mcspi1_clk", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi1_cs0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcspi1_cs1", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcspi1_cs2", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcspi1_cs3", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcspi1_simo", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("mcspi1_somi", OMAP_PIN_INPUT_PULLUP);
}

static void __init panda_config_gpmc_mux(void)
{
	omap_mux_init_signal("gpmc_clk", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpmc_nadc_ale", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_noe", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_nwe", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_nbe0_cle", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_nbe1", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_wait0", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpmc_wait1", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpmc_cs0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_cs1", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_ad0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_ad1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_ad2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_ad3", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_ad4", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_ad5", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_ad6", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("gpmc_ad7", OMAP_PIN_INPUT_PULLUP);
}

static struct spi_board_info panda_mcspi_board_info[] = {
	// spi 1.0
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, // 48 Mbps
		.bus_num	= 1,
		.chip_select	= 0,
		.mode = SPI_MODE_1,
	},

	// spi 1.1
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, // 48 Mbps
		.bus_num	= 1,
		.chip_select	= 1,
		.mode = SPI_MODE_1,
	},

	// spi 1.2
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, // 48 Mbps
		.bus_num	= 1,
		.chip_select	= 2,
		.mode = SPI_MODE_1,
	},

	// spi 1.3
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, // 48 Mbps
		.bus_num	= 1,
		.chip_select	= 3,
		.mode = SPI_MODE_1,
	},
};

void __init pandadaq_init_mux() {
	panda_config_mcspi1_mux();
	panda_config_gpmc_mux();
	spi_register_board_info(panda_mcspi_board_info,
			ARRAY_SIZE(panda_mcspi_board_info));
}

static struct class *pandadaq_class;

static const struct file_operations pandadaq_fops = {
        .read         = pandadaq_read,
        .write        = pandadaq_write,
	.mmap         = pandadaq_mmap,
        .open         = pandadaq_open,
};

/**
 * pandadaq_init - Initialize PandaDAQ GPMC resources
 * @pdata: Platform data with base address set
 * @return status of the operation
 */
int __init pandadaq_probe(struct platform_device *pdev)
{
        int ret;
        struct device *dev;
        unsigned int *addr;

        pandadaq = kzalloc(sizeof(*pandadaq), GFP_KERNEL);
        if (!pandadaq) {
                ret = -ENOMEM;
                goto err;
        }
        
        pandadaq->dev = &pdev->dev;
        platform_set_drvdata(pdev, pandadaq);
        
        pandadaq_init_mux();
	ret = pandadaq_init_gpmc(pandadaq);
        if (!ret) {
                printk(KERN_ERR "pandadaq: Failed to configure GPMC: %d\n", ret);
                goto err;
        }
        printk(KERN_ERR "pandadaq: Window at %08lx\n", pandadaq->phys_base);

        addr = ioremap(pandadaq->phys_base, PANDADAQ_WINDOW_SIZE);
        if (!addr) {
                ret = -ENOMEM;
                goto err;
        }

        pandadaq->base = addr;
        printk(KERN_ERR "pandadaq: Window mapped at %08lx\n", pandadaq->base);

	pandadaq_class = class_create(THIS_MODULE, "pandadaq");
	if (IS_ERR(pandadaq_class))
		return PTR_ERR(pandadaq_class);
	
        cdev_init(&pandadaq->cdev, &pandadaq_fops);
        ret = cdev_add(&pandadaq->cdev, MKDEV(202, 128), 1);
        if (ret) {
                printk(KERN_ERR "pandadaq: Failed to create char device: %d\n", ret);
                goto err;
        }
	
	dev = device_create(pandadaq_class, NULL, 0,
		      NULL, "pandadaq");
	if (IS_ERR(dev)) {
		printk(KERN_ERR "pandadaq: Failed to register device: %ld\n", PTR_ERR(dev));
		return PTR_ERR(dev);
	}

	return 0;

err:
        pandadaq = NULL;
        return ret;
}

static int __devexit pandadaq_remove(struct platform_device *pdev)
{
        struct pandadaq_data *pdaq = platform_get_drvdata(pdev);
        kfree(pdaq);

        return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id pandadaq_match[] = {
        {
                .compatible = "pandadaq",
        },
        { },
};
MODULE_DEVICE_TABLE(of, pandadaq_match);
#endif

static struct platform_driver pandadaq_driver = {
        .probe = pandadaq_probe,
        .remove = __exit_p(pandadaq_remove),
        .driver = {
                .name = "pandadaq",
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(pandadaq_match),
        },
};

module_platform_driver(pandadaq_driver);

MODULE_DESCRIPTION("PandaDAQ driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("B Gamari");
MODULE_ALIAS("platform:pandadaq");

