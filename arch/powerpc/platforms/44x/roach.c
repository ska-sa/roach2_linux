/*
 * Roach support
 *
 * 2012 Marc Welz for SKA/SA 
 * 2013 Shanly Rajan for SKA/SA 
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; version 2 of the License.
 *
 */

#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm/ppc4xx.h>
#include <asm/prom.h>
#include <asm/time.h>
#include <asm/udbg.h>
#include <asm/uic.h>

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

static __initdata struct of_device_id roach_of_bus[] = {
        { .compatible = "ibm,plb4", },
        { .compatible = "ibm,opb", },
        { .compatible = "ibm,ebc", },
        { .compatible = "simple-bus", },
        {},
};

static int __init roach_device_probe(void)
{
        of_platform_bus_probe(NULL, roach_of_bus, NULL);

        return 0;
}
machine_device_initcall(roach, roach_device_probe);

static int __init roach_probe(void)
{
        unsigned long root = of_get_flat_dt_root();

	if (of_flat_dt_is_compatible(root, "kat,roach")) {

		printk(KERN_INFO "roach: claiming matching platform");

		pci_set_flags(PCI_REASSIGN_ALL_RSRC);
		return 1;
	}

        return 0;
}

static void roach_power_off(void)
{
	//struct device_node *np, *child;
	int pin;

	pin = (-1);

        printk(KERN_INFO "roach: performing hardware power down");
        printk(KERN_INFO "roach: No software routine yet written for powerdown");

}

define_machine(roach) {
        .name = "PowerPC 44x Platform",
                .probe = roach_probe,
                .progress = udbg_progress,
                .init_IRQ = uic_init_tree,
                .get_irq = uic_get_irq,
                .restart = ppc4xx_reset_system,
                .power_off = roach_power_off,
                .calibrate_decr = generic_calibrate_decr,
};
