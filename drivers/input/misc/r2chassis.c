#include <linux/of_platform.h>
#include <linux/of_gpio.h>


#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <linux/of_irq.h>

MODULE_AUTHOR("Shanly Rajan <shanly.rajan@ska.ac.za>");
MODULE_DESCRIPTION("ROACH2 chassis gpio driver");
MODULE_LICENSE("GPL");

unsigned char r2case_keycode[1] = {
        KEY_POWER
};

static struct input_dev *r2case;
int r2chassis_irq;
unsigned char data;
int green_pin, red_pin = (-1);

static int r2case_setup_gpios(void)
{
        struct device_node *np, *child;                                                                                                

        np = of_find_compatible_node(NULL, NULL, "gpio-pins");                                                                         
        if (!np) { 
                printk(KERN_ERR "unable to locate gpio interface\n");                                                                    
                return -1;
        }

        for_each_child_of_node(np, child)                                                                                              
                if (strcmp(child->name, "cpu_rdy") == 0) {                                                                           
                        green_pin = of_get_gpio(child, 0);                                                                                   
                }
                else if (strcmp(child->name, "cpu_err") == 0) {                                                                           
                        red_pin = of_get_gpio(child, 0);                                                                                   
                }

        if(green_pin == -1 || red_pin == -1){
                printk(KERN_ERR "unable to locate gpio pins,cpu_rdy and cpu_err\n");                                                                    
                return -1;
        }

        of_node_put(np);

        return 0;
}


/*
 * Toggle LED  
 */

static void  r2case_toggle_leds(int code, int value)
{

        int pin = (-1);
        int actual;

      //  printk(KERN_DEBUG "attempting led toggle\n");                                                                            

        if (code == LED_MISC) { 
                pin = green_pin;
                actual = value;
                printk(KERN_DEBUG "About to toggle cpu_rdy pin\n");                                                                   
        }
        else { /*code = LED_SUSPEND)*/
                pin = red_pin;
                actual =  value;
                printk(KERN_DEBUG "About to toggle cpu_err pin\n");                                                                   
        }

        gpio_set_value(pin, actual);                      

}

/*
 * r2case_event() handles events from the input module.
 */

static int r2case_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{

        printk(KERN_DEBUG "%s(): Got type %x, code %x, value %x\n", __func__, type, code, value); 
        switch (type) {
                case EV_LED:
                        switch (code) {
                                case LED_MISC:
                                case LED_SUSPEND:
                                        r2case_toggle_leds(code, value);
                                        break;
                                default:
                                        printk(KERN_DEBUG "%s(): Unsupported Code: code %x, value %x\n", __func__, code, value);
                        }
                        return 0;
                default:
                        printk(KERN_ERR "%s(): Unsupported event: Got unknown type %x, code %x, value %x\n", __func__, type, code, value);
        }

        return -1;
}

/*
 * r2chassis_interrupt(). Here takes place processing of data received from
 * the keyboard into events.
 */


static irqreturn_t r2chassis_interrupt(int irq,  void *data)
{

//        printk(KERN_INFO "%s: irq %d\n", __func__, irq);


        input_report_key(r2case, r2case_keycode[0], 1);
        input_report_key(r2case, r2case_keycode[0], 0);

        input_sync(r2case);


        return IRQ_HANDLED;
}


static int r2case_probe(struct platform_device *ofdev)
{
        
        int i,err;

        printk(KERN_DEBUG "%s:node name:%s\n", __func__, ofdev->dev.of_node->full_name);


        r2case = input_allocate_device();
        if(!r2case){
                printk(KERN_ERR "%s(): Failed to allocate input device\n", __func__);
                return -ENOMEM;
        }

        memset(r2case, 0, sizeof(struct input_dev *));


        /*
         * Setting up the input device capabilities
         */
        r2case->name = "roach2chassis";
        r2case->event = r2case_event;

        r2case->evbit[0]  = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_MSC) | BIT_MASK(EV_LED);
        r2case->ledbit[0] = BIT_MASK(LED_SUSPEND) | BIT_MASK(LED_MISC);
        r2case->sndbit[0] = BIT_MASK(SND_CLICK);

        r2case->keycode = r2case_keycode;
        r2case->keycodesize = sizeof(unsigned char);
        r2case->keycodemax = ARRAY_SIZE(r2case_keycode);
        for (i = 0; i < 1; i++) {
                set_bit(r2case_keycode[i], r2case->keybit);
        }

        __clear_bit(KEY_RESERVED, r2case->keybit);

        err = input_register_device(r2case);
        if(err){
                printk(KERN_ERR "%s(): Failed to register input device\n", __func__);
                goto err_free_dev;
        }

        r2chassis_irq = irq_of_parse_and_map(ofdev->dev.of_node, 0);
        if(r2chassis_irq == NO_IRQ){
                printk(KERN_ERR "%s:failed to map interrupts!\n", __func__);
                goto err_free_device;
        }
        
        err = request_irq(r2chassis_irq, r2chassis_interrupt, 0, "R2KBD", NULL);
        if (err){
                printk(KERN_ERR "%s:irq request failed!\n", __func__);
                goto err_free_device;
        }

        err = r2case_setup_gpios();
        if (err){
                printk(KERN_ERR "%s:irq request failed!\n", __func__);
                goto err_free_irq;
        }

        printk(KERN_DEBUG "%s: init successful\n", __func__);

        return 0;

err_free_irq:
        free_irq(r2chassis_irq, r2chassis_interrupt);
err_free_device:
        input_unregister_device(r2case);
err_free_dev:
        input_free_device(r2case);
        return err;

}

static int r2case_remove(struct platform_device *ofdev)
{
        free_irq(r2chassis_irq, r2chassis_interrupt);
        input_unregister_device(r2case);
        input_free_device(r2case);
        return 0;
}

static const struct of_device_id r2case_platform_match[] =
{
        {
                .compatible     = "kat,r2chassis",
        },
        {},
};

MODULE_DEVICE_TABLE(of, r2case_platform_match);

static struct platform_driver r2case_of_driver = {
        .driver = {
                .name = "r2chassis",
                .owner = THIS_MODULE,
                .of_match_table = r2case_platform_match,
        },
        .probe = r2case_probe,
        .remove = r2case_remove,
};

int __init r2case_init(void)
{
        return platform_driver_register(&r2case_of_driver);
}

void r2case_exit(void)
{
        platform_driver_unregister(&r2case_of_driver);
}


module_init(r2case_init);
module_exit(r2case_exit);

MODULE_AUTHOR("Shanly Rajan <shanly.rajan@ska.ac.za>");
MODULE_DESCRIPTION("ROACH2 Chassis Input Device Driver");
MODULE_LICENSE("GPL");

