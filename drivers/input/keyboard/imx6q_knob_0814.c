#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/uaccess.h> 
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/input.h>

#include <linux/workqueue.h>  
#include <linux/gpio.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach/arch.h>

struct knob_desc{
	int irq;
	char *name;
	unsigned char clockwise_bit_val;
	unsigned char anticw_bit_val;
	unsigned int clockwise_pulse_val;
	unsigned int anticlockwise_pulse_val;
	int cwpin;
	int acwpin;
	int cwpin_irq;
	int acwpin_irq;

	struct delayed_work knob_dwq; 
	struct workqueue_struct *test_wq;
	spinlock_t     spinlock;
	struct input_dev *knobs_dev;
	struct timer_list knobs_timer;
	//volatile unsigned char knob_clockwise_status;
	//volatile unsigned char knob_anticlockwise_status;

};

static struct knob_desc *imx6q_knob; 


static void knobs_timer_function(unsigned long data)  
{  
 
        
	pr_info("wq_func \r\n");  
 	//input_report_key(imx6q_knob->knobs_dev, KEY_ENTER, 1);
	//input_report_key(imx6q_knob->knobs_dev, KEY_TAB, 1);
	input_report_key(imx6q_knob->knobs_dev, BTN_TOUCH, 1);
	input_report_abs(imx6q_knob->knobs_dev, ABS_X, 2);
	input_report_abs(imx6q_knob->knobs_dev, ABS_Y, 3);
        input_sync(imx6q_knob->knobs_dev);

	mod_timer(&imx6q_knob->knobs_timer, jiffies + (HZ));

}  



void wq_func(struct work_struct *work);  
 
void wq_func(struct work_struct *work)  
{  
	
    #if 0
    input_report_key(imx6q_knob->knobs_dev,BTN_TOUCH, 0);  

    input_report_abs(imx6q_knob->knobs_dev,ABS_PRESSURE, 0);

    input_sync(imx6q_knob->knobs_dev);  

    input_report_abs(imx6q_knob->knobs_dev,ABS_X, 1); 
    input_report_key(imx6q_knob->knobs_dev, BTN_LEFT, 2); 
    //input_report_abs(imx6q_knob->knobs_dev,ABS_Y, 3);
    //input_report_abs(imx6q_knob->knobs_dev,ABS_PRESSURE, 2);
    //input_event(imx6q_knob->knobs_dev,EV_ABS,ABS_X,1); 

    pr_info("wq_func \r\n"); 
    input_sync(imx6q_knob->knobs_dev);

    //schedule_delayed_work(&imx6q_knob->knob_dwq, HZ);
    #endif


}  

static int knobs_init(void)
{
	
	imx6q_knob  = (struct knob_desc*)kmalloc(sizeof(struct knob_desc), GFP_KERNEL);  
	struct input_dev *input = input_allocate_device();
        if (!input || !imx6q_knob) {
                pr_info("failed to allocate state\n");
		return 1;
        }
	input->name = "knob";
        input->phys = "knobs/input2";

        input->id.bustype = BUS_HOST;
        input->id.vendor = 0x0001;
        input->id.product = 0x0001;
        input->id.version = 0x0100;
#if 1
	/* 能产生哪类事件 */
	//input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS); 
	//input->keybit[BIT_WORD(BTN_TOUCH)] =BIT_MASK(BTN_TOUCH); 

	set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, 0,0x3FF, 16, 0);  
        input_set_abs_params(input, ABS_Y, 0,0x3FF, 16, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 300, 0, 0);


	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	

#endif

#if 0
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_REP, input->evbit);
	set_bit(KEY_ENTER, input->keybit);
	set_bit(KEY_TAB, input->keybit);
#endif	
 

	/* 注册 */
	imx6q_knob->knobs_dev = input;
	input_register_device(imx6q_knob->knobs_dev);

	init_timer(&imx6q_knob->knobs_timer);  
        imx6q_knob->knobs_timer.function = knobs_timer_function;  
        add_timer(&imx6q_knob->knobs_timer);  

	mod_timer(&imx6q_knob->knobs_timer, jiffies + (HZ/20)); 


	imx6q_knob->test_wq = create_workqueue("test_wq"); 
     	if (!imx6q_knob->test_wq) {  
        	printk(KERN_ERR "No memory for workqueue\n");  
        	return 1;     
    	} 
    spin_lock_init(&imx6q_knob->spinlock);
    INIT_DELAYED_WORK(&imx6q_knob->knob_dwq, wq_func);  
 //   schedule_delayed_work(&imx6q_knob->knob_dwq, HZ/20);  
    return 0;
	
}

static void knobs_exit(void)
{
	printk("knobs_exit!\n");
	input_unregister_device(imx6q_knob->knobs_dev);
	//input_free_device(imx6q_knob->knobs_dev);
	del_timer(&imx6q_knob->knobs_timer);  
	kfree(imx6q_knob);  	
}

module_init(knobs_init);

module_exit(knobs_exit);

MODULE_LICENSE("GPL");



