/*******************EPT**********************/
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


#define KNOB_CW_COUNT  	ABS_X
#define KNOB_ACW_COUNT 	ABS_Y
#define KNOB_SPEED	ABS_WHEEL
#define KNOB_KEY	BTN_MISC

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
	//volatile unsigned char knob_rdy_status;
	//volatile unsigned char knob_counter;
	volatile unsigned char knob_clockwise_status;
	volatile unsigned char knob_anticlockwise_status;

};
 
static struct knob_desc *imx6q_knob; 


void wq_func(struct work_struct *work);  
 
void wq_func(struct work_struct *work)  
{  
    //struct knob_desc *imx6q_knob;
    //imx6q_knob = container_of(work, struct knob_desc, work);

    struct delayed_work *delay_work = to_delayed_work(work);
    //imx6q_knob = container_of(delay_work, struct knob_desc, knob_dwq);
          
    pr_info("wq_func \r\n"); 
     
    //spin_lock(&imx6q_knob->spinlock);
    //input_event(imx6q_knob->knobs_dev, EV_ABS, KNOB_CW_COUNT , imx6q_knob->clockwise_pulse_val);
    //input_event(imx6q_knob->knobs_dev, EV_ABS, imx6q_knob->direction_code_val , imx6q_knob->anticlockwise_pulse_val);

    //input_report_abs(imx6q_knob->knobs_dev, KNOB_CW_COUNT,  imx6q_knob->clockwise_pulse_val);
    //input_report_abs(imx6q_knob->knobs_dev, KNOB_ACW_COUNT,  imx6q_knob->anticlockwise_pulse_val);

    //input_report_abs(imx6q_knob->knobs_dev, KNOB_SPEED,  0x0000001D);
    input_report_abs(imx6q_knob->knobs_dev,ABS_X, 0x0000001D);  

    //input_report_key(imx6q_knob->knobs_dev, KNOB_KEY, 1);
    
    input_sync(imx6q_knob->knobs_dev);

    

 
    imx6q_knob->clockwise_pulse_val = 0;
    imx6q_knob->anticlockwise_pulse_val = 0;
    schedule_delayed_work(&imx6q_knob->knob_dwq, HZ/20);


} 

static irqreturn_t knob_clockwise_irq(int irq, void *dev_id)
{
	//struct knob_desc * imx6q_knob = (struct knob_desc *)dev_id;

	//unsigned temp ;
	//printk("KNOB_CW_INT interrupt happend!\n");
	imx6q_knob->knob_clockwise_status = true;

	//udelay(1);
	//printk("clockwise value is %d !\n", gpio_get_value_cansleep(KNOB_CW_INT_PIN));
	//printk("anti-clockwise value is %d !\n", gpio_get_value_cansleep(KNOB_ACW_INT_PIN));

	if((true == imx6q_knob->knob_clockwise_status ) && (true == imx6q_knob->knob_anticlockwise_status))
	{
		imx6q_knob->clockwise_pulse_val++;
		imx6q_knob->knob_clockwise_status = false;
		imx6q_knob->knob_anticlockwise_status = false;
	}

	return IRQ_HANDLED;

}

static irqreturn_t knob_anticw_irq(int irq, void *dev_id)
{
	//struct knob_desc * imx6q_knob = (struct knob_desc *)dev_id;

	//printk("KNOB_ACW_INT interrupt happend!\n"); 
	imx6q_knob->knob_anticlockwise_status = true;

	//udelay(1);
	//printk("clockwise value is %d !\n", gpio_get_value_cansleep(KNOB_CW_INT_PIN));
	//printk("anti-clockwise value is %d !\n", gpio_get_value_cansleep(KNOB_ACW_INT_PIN));

	if((true == imx6q_knob->knob_clockwise_status ) && (true == imx6q_knob->knob_anticlockwise_status))
	{
		imx6q_knob->anticlockwise_pulse_val++;
		imx6q_knob->knob_clockwise_status = false;
		imx6q_knob->knob_anticlockwise_status = false;
	}

	return IRQ_HANDLED;
}

static int imx6q_knob_remove(struct platform_device *pdev)

{
	struct knob_desc *imx6q_knob;
    imx6q_knob = (struct knob_desc *)platform_get_drvdata(pdev);

	free_irq(imx6q_knob->cwpin_irq, imx6q_knob);
	free_irq(imx6q_knob->acwpin_irq, imx6q_knob);
	input_unregister_device(imx6q_knob->knobs_dev);
	input_free_device(imx6q_knob->knobs_dev);	
        gpio_free(imx6q_knob->cwpin); 
	gpio_free(imx6q_knob->acwpin);
	return 0;
}

static void create_input_device(struct platform_device *pdev, struct knob_desc *imx6q_knob)
{
	/*分配一个input_dev结构体 */
	struct input_dev *input = input_allocate_device();
        if (!input) {
                pr_info("failed to allocate state\n");
		return;
        }

        input->name = "imx6q_knob";
        //input->phys = "gpio-keys/input0";
        //input->dev.parent = &pdev->dev;
        //input->open = gpio_keys_open;
        //input->close = gpio_keys_close;

        input->id.bustype = BUS_HOST;
        input->id.vendor = 0x0001;
        input->id.product = 0x0001;
        input->id.version = 0x0100;


	/* 产生事件类型*/
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	//set_bit(EV_SYNC, input->evbit);

	/* 能产生这类操作里的哪些事件*/
	//set_bit(KNOB_KEY, input->keybit);
	//set_bit(KNOB_SPEED, input->absbit);
	input->keybit[BIT_WORD(BTN_TOUCH)] =BIT_MASK(BTN_TOUCH);  
	//input_set_abs_params(input, KNOB_SPEED, 0, 0x3FF, 0, 0);
	input_set_abs_params(input, ABS_X, 0,0x3FF, 0, 0); 


	

	/* 注册 */
	imx6q_knob->knobs_dev = input;
	input_set_drvdata(imx6q_knob->knobs_dev, imx6q_knob);

	input_register_device(imx6q_knob->knobs_dev);
}

static int imx6q_knob_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int error;
	struct knob_desc *imx6q_knob;
	imx6q_knob = (struct knob_desc*) kmalloc(sizeof(struct knob_desc), GFP_KERNEL);

	imx6q_knob->cwpin = of_get_named_gpio(dev->of_node, "cwpin", 0);
	pr_info("imx6q_knob->cwpin : %d\r\n", imx6q_knob->cwpin);
	if(imx6q_knob->cwpin == -1){
		pr_info("imx6q_knob->cwpin error: %d\r\n", imx6q_knob->cwpin);
		return;
	}
	gpio_request(imx6q_knob->cwpin,"knob_irq_1");
	gpio_direction_input(imx6q_knob->cwpin); 

	
	
	imx6q_knob->acwpin =  of_get_named_gpio(dev->of_node, "acwpin", 0);
	pr_info("imx6q_knob->acwpin : %d\r\n", imx6q_knob->acwpin );
	if(imx6q_knob->acwpin == -1){
		pr_info("imx6q_knob->acwpin error: %d\r\n", imx6q_knob->acwpin );
		return;
	}
	gpio_request(imx6q_knob->acwpin,"knob_irq_2");
	gpio_direction_input(imx6q_knob->acwpin); 

	imx6q_knob->acwpin_irq = gpio_to_irq(imx6q_knob->acwpin);
	imx6q_knob->cwpin_irq = gpio_to_irq(imx6q_knob->cwpin);
	pr_info("imx6q_knob->cwpin_irq : %d\r\n", imx6q_knob->cwpin_irq );
	pr_info("imx6q_knob->acwpin_irq : %d\r\n", imx6q_knob->acwpin_irq );

	error = request_irq(imx6q_knob->cwpin_irq, knob_clockwise_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "knob_clockwise_irq", imx6q_knob);
	if (error) {
		printk("failed to request IRQ1 \n");
	}
	
	error = request_irq(imx6q_knob->acwpin_irq, knob_anticw_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "knob_anticw_irq", imx6q_knob);
	if (error) {
		printk("failed to request IRQ2 \n");
	}
	platform_set_drvdata(pdev, imx6q_knob);

	create_input_device(pdev, imx6q_knob);
	//device_init_wakeup(&pdev->dev, 0);

	imx6q_knob->test_wq = create_workqueue("test_wq");  
     	if (!imx6q_knob->test_wq) {  
        	printk(KERN_ERR "No memory for workqueue\n");  
        	return 1;     
    	} 

    spin_lock_init(&imx6q_knob->spinlock);
    INIT_DELAYED_WORK(&imx6q_knob->knob_dwq, wq_func);  
    schedule_delayed_work(&imx6q_knob->knob_dwq, HZ/20);  
    return 0;
}

static struct of_device_id device_table[] = {
		{.compatible = "imx6q,knobirq"},
	};
	static struct platform_driver imx6q_knob_driver=
        {
        .probe = imx6q_knob_probe,
        .remove = imx6q_knob_remove,
	.driver={
		.name = "imx6q_knob-driver",
		.of_match_table =  device_table,
	},
        };

static int knobs_init(void)
{
        return platform_driver_register(&imx6q_knob_driver);  	
}

static void knobs_exit(void)
{
	platform_driver_unregister(&imx6q_knob_driver);
}

module_init(knobs_init);

module_exit(knobs_exit);

MODULE_LICENSE("GPL");



