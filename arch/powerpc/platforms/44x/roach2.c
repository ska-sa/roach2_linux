/*
 * Roach2 support
 *
 * 2012 Marc Welz for SKA/SA 
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

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

static __initdata struct of_device_id roach2_of_bus[] = {
	{ .compatible = "ibm,plb4", },
	{ .compatible = "ibm,opb", },
	{ .compatible = "ibm,ebc", },
	{ .compatible = "simple-bus", },
	{},
};

static int __init roach2_device_probe(void)
{
	of_platform_bus_probe(NULL, roach2_of_bus, NULL);

	return 0;
}
machine_device_initcall(roach2, roach2_device_probe);

static int __init roach2_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (of_flat_dt_is_compatible(root, "kat,roach2")) {
		pci_set_flags(PCI_REASSIGN_ALL_RSRC);
		return 1;
	}

	return 0;
}

static void roach2_power_off(void)
{
	struct device_node *np, *child;
	int pin;

	pin = (-1);

	printk(KERN_INFO "attempting hardware power down");

	np = of_find_compatible_node(NULL, NULL, "gpio-pins");
	if (!np) {
		printk(KERN_ERR "unable to locate gpio interface");
		return;
	}

	for_each_child_of_node(np, child)
		if (strcmp(child->name, "powerdown") == 0) {
			pin = of_get_gpio(child, 0);
		}

	of_node_put(np);

	if(pin < 0){
		printk(KERN_WARNING "no gpio for power circuit found");
		return;
	}

	printk(KERN_INFO "power off: about to drive power pin low");

	gpio_set_value(pin, 0);
}

define_machine(roach2) {
	.name = "PowerPC 44x Platform",
	.probe = roach2_probe,
	.progress = udbg_progress,
	.init_IRQ = uic_init_tree,
	.get_irq = uic_get_irq,
	.restart = ppc4xx_reset_system,
	.power_off = roach2_power_off,
	.calibrate_decr = generic_calibrate_decr,
};
