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



//#define EPT_TEST
#define USE_OWNER_WORKQUEUE

#define PERIOD (HZ/5)

#define KNOB_CW_COUNT  	ABS_X
#define KNOB_ACW_COUNT 	ABS_Y
#define KNOB_SPEED	ABS_WHEEL
#define KNOB_KEY	BTN_MISC

struct knob_data {
	int speed;
	unsigned char clockwise_bit_val;
	unsigned char anticw_bit_val;
	unsigned char btn_val;
	unsigned int clockwise_pulse_val;
	unsigned int anticlockwise_pulse_val;
	
	//volatile unsigned char knob_rdy_status;
	//volatile unsigned char knob_counter;
	volatile unsigned char knob_clockwise_status;
	volatile unsigned char knob_anticlockwise_status;
};

struct knob_device{
	spinlock_t spinlock;
	int btnpin, cwpin, acwpin;
	int btn_irq, cwpin_irq, acwpin_irq;
	struct knob_data data;
	
	struct delayed_work knob_work; 
	struct input_dev *input_dev;
#if defined(USE_OWNER_WORKQUEUE)
	struct workqueue_struct *workqueue;
#endif
};

void knob_reset_data(struct knob_device *knob)
{
	struct knob_data* data = &knob->data;
	//spin_lock(&knob->spinlock);
	data->clockwise_pulse_val = 0;
    data->anticlockwise_pulse_val = 0;
	data->knob_clockwise_status = false;
	data->knob_anticlockwise_status = false;
	data->btn_val = 0;
	//spin_unlock(&knob->spinlock);
}

void knob_report_func(struct work_struct *work)  
{  
	struct knob_data* data;
    struct knob_device *knob;
    struct delayed_work *delay_work = to_delayed_work(work);
    knob = container_of(delay_work, struct knob_device, knob_work);
	data = &knob->data;
        
    //pr_info("knob_report_func %d\r\n", knob->speed++); 




	data->speed = data->clockwise_pulse_val - data->anticlockwise_pulse_val;
	
    input_report_abs(knob->input_dev, KNOB_CW_COUNT,  data->clockwise_pulse_val);
    input_report_abs(knob->input_dev, KNOB_ACW_COUNT,  data->anticlockwise_pulse_val);
    input_report_abs(knob->input_dev, KNOB_SPEED,  data->speed % 1024);
    input_report_key(knob->input_dev, KNOB_KEY, data->btn_val);
    //input_report_key(knob->input_dev, KNOB_KEY, 0);
 
    input_sync(knob->input_dev);
	
#if defined(EPT_TEST)
	data->speed++;
#else
	knob_reset_data(knob);
#endif	
	
#if defined(USE_OWNER_WORKQUEUE)
	queue_delayed_work(knob->workqueue, &knob->knob_work, PERIOD);
#else
    schedule_delayed_work(&knob->knob_work, PERIOD);
#endif
} 

	//printk("KNOB_CW_INT!\n");
	
static irqreturn_t knob_clockwise_irq(int irq, void *dev_id)
{
	
	struct knob_device * knob = (struct knob_device *)dev_id;
	struct knob_data * data = &knob->data;
	//printk("cpin  %d !\n", gpio_get_value(knob->cwpin));
	//printk("anpin  %d !\n", gpio_get_value(knob->acwpin));
#if 1
	if(((0 == gpio_get_value(knob->cwpin)) && (gpio_get_value(knob->acwpin) > 0)) || ((0 == gpio_get_value(knob->acwpin)) && (gpio_get_value(knob->cwpin) > 0)) )
	{
		data->clockwise_pulse_val++;
		//data->anticlockwise_pulse_val = 0;	
	}

	else if(((0 != gpio_get_value(knob->cwpin)) && (gpio_get_value(knob->acwpin) != 0)) || ((0 == gpio_get_value(knob->acwpin)) && (gpio_get_value(knob->cwpin) == 0))) 
	{
		data->anticlockwise_pulse_val++;
		//data->clockwise_pulse_val = 0;
	}
#endif
	return IRQ_HANDLED;
}


static irqreturn_t knob_anticw_irq(int irq, void *dev_id)
{
#if 0	
	struct knob_device * knob = (struct knob_device *)dev_id;
	struct knob_data * data = &knob->data;
	if(((0 == gpio_get_value(knob->cwpin)) && (gpio_get_value(knob->acwpin) > 0)) || ((0 == gpio_get_value(knob->acwpin)) && (gpio_get_value(knob->cwpin) > 0)) )
	{
		data->anticlockwise_pulse_val++;
		data->clockwise_pulse_val = 0;
	}
	
#endif
	return IRQ_HANDLED;
}


static irqreturn_t knob_btn_irq(int irq, void *dev_id)
{
	struct knob_device * knob = (struct knob_device *)dev_id;
	struct knob_data * data = &knob->data;

	//printk("KNOB_BTN_INT interrupt happend!\n");
	//data->btn_val = gpio_get_value(knob->btnpin);
	data->btn_val = 1;
	return IRQ_HANDLED;
}

static int knob_release_irq(struct knob_device* knob)
{
	//free_irq(knob->irq, knob);
	free_irq(knob->cwpin_irq, knob);
	free_irq(knob->acwpin_irq, knob);
	free_irq(knob->btn_irq, knob);
	//gpio_free(knob->pin);
    gpio_free(knob->cwpin); 
	gpio_free(knob->acwpin);
	gpio_free(knob->btnpin);
	return 0;
}
static int knob_init_irq(struct knob_device* knob, struct device *dev)
{
	int err;
	// for cwpin
	knob->cwpin = of_get_named_gpio(dev->of_node, "cwpin", 0);
	pr_info("knob->cwpin : %d\r\n", knob->cwpin);
	if(knob->cwpin == -1){
		pr_info("knob->cwpin error: %d\r\n", knob->cwpin);
		return -1;
	}
	gpio_request(knob->cwpin,"knob_irq_1");
	gpio_direction_input(knob->cwpin); 

	// for acwpin
	knob->acwpin =  of_get_named_gpio(dev->of_node, "acwpin", 0);
	pr_info("knob->acwpin : %d\r\n", knob->acwpin );
	if(knob->acwpin == -1){
		pr_info("knob->acwpin error: %d\r\n", knob->acwpin );
		return -1;
	}
	gpio_request(knob->acwpin,"knob_irq_2");
	gpio_direction_input(knob->acwpin); 
	
	// for keypin
	knob->btnpin = of_get_named_gpio(dev->of_node, "btnpin", 0);
	pr_info("knob->btnpin : %d\r\n", knob->btnpin);
	if(knob->btnpin == -1){
		pr_info("knob->btnpin error: %d\r\n", knob->btnpin);
		return -1;
	}
	gpio_request(knob->btnpin,"knob_irq_3");
	gpio_direction_input(knob->btnpin); 
	
	// irq
	knob->acwpin_irq = gpio_to_irq(knob->acwpin);
	knob->cwpin_irq = gpio_to_irq(knob->cwpin);
	knob->btn_irq = gpio_to_irq(knob->btnpin);
	pr_info("knob->cwpin_irq : %d\r\n", knob->cwpin_irq );
	pr_info("knob->acwpin_irq : %d\r\n", knob->acwpin_irq );
	pr_info("knob->btn_irq : %d\r\n", knob->btn_irq );
	
	//err = request_irq(knob->cwpin_irq, knob_clockwise_irq, IRQF_TRIGGER_RISING , "knob_clockwise_irq", knob);
	err = request_irq(knob->cwpin_irq, knob_clockwise_irq,  IRQF_TRIGGER_FALLING, "knob_clockwise_irq", knob);
	if (err) {
		pr_info("failed to request IRQ1 \n");
		return -1;
	}
	
	//err = request_irq(knob->acwpin_irq, knob_anticw_irq, IRQF_TRIGGER_RISING , "knob_anticw_irq", knob);
	err = request_irq(knob->acwpin_irq, knob_anticw_irq, IRQF_TRIGGER_FALLING, "knob_anticw_irq", knob);

	if (err) {
		pr_info("failed to request IRQ2 \n");
		return -1;
	}
	
	err = request_irq(knob->btn_irq, knob_btn_irq, IRQF_TRIGGER_FALLING, "knob_btn_irq", knob);
	if (err) {
		pr_info("failed to request IRQ3 \n");
		return -1;
	}
	
	return 0;

}

static int create_input_device(struct platform_device *pdev, struct knob_device *knob)
{
	int err;
	struct input_dev *input = input_allocate_device();
	if (!input) {
		pr_info("failed to input_allocate_device\n");
		err = -ENOMEM;
		return err;
	}

	input->name = "knob";
	input->phys = "knob/input0";
	input->dev.parent = &pdev->dev;
	//input->open = knob_open;
	//input->close = knob_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;


	/* 产生事件类型*/
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_REP, input->evbit);

	/* 能产生这类操作里的哪些事件*/
	set_bit(KNOB_KEY, input->keybit);

	input_set_abs_params(input, KNOB_SPEED, -0x3FF, 0x3FF, 0, 0);
	input_set_abs_params(input, KNOB_CW_COUNT, 0, 0x3FF, 0, 0);
	input_set_abs_params(input, KNOB_ACW_COUNT, 0, 0x3FF, 0, 0);

	/* 注册 */
	knob->input_dev = input;
	input_set_drvdata(knob->input_dev, knob);

	err = input_register_device(input);
	if (err){
		pr_info("failed to input_register_device\n");	
		return err;
	}
	return 0;
}


static int imx6q_knob_probe(struct platform_device *pdev)
{
	int err;
	struct knob_device *knob;
	knob = (struct knob_device*) kmalloc(sizeof(struct knob_device), GFP_KERNEL);

	spin_lock_init(&knob->spinlock);

	err = create_input_device(pdev, knob);
	if(err){
		pr_info("failed to create input device\n");
		return err;
	}
	
    INIT_DELAYED_WORK(&knob->knob_work, knob_report_func);  
#if defined(USE_OWNER_WORKQUEUE)
	knob->workqueue = create_singlethread_workqueue("knob-workqueue");
	queue_delayed_work(knob->workqueue, &knob->knob_work, PERIOD);
#else
    schedule_delayed_work(&knob->knob_work, PERIOD);  
#endif
	
#if !defined(EPT_TEST)
	err = knob_init_irq(knob, &pdev->dev);
	if(err){
		pr_info("failed to knob_init_irq\n");
		return err;		
	}
#endif
	platform_set_drvdata(pdev, knob);
    return 0;
}

static int imx6q_knob_remove(struct platform_device *pdev)
{
	struct knob_device *knob;
 	knob = (struct knob_device *)platform_get_drvdata(pdev);
	
#if !defined(EPT_TEST)
	knob_release_irq(knob);
#endif

	cancel_delayed_work(&knob->knob_work);
#if defined(USE_OWNER_WORKQUEUE)
	destroy_workqueue(knob->workqueue);
#endif

	input_unregister_device(knob->input_dev);
	input_free_device(knob->input_dev);	
	kfree(knob);
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
		.name = "knob-driver",
		.of_match_table =  device_table,
	},
};

#if defined(EPT_TEST)		
static struct platform_device knob_device = {
        .name   = "knob-driver",
};
#endif

static int knobs_init(void)
{
#if defined(EPT_TEST)	
	platform_device_register(&knob_device);
#endif
	return platform_driver_register(&imx6q_knob_driver);  	
}

static void knobs_exit(void)
{
#if defined(EPT_TEST)		
	platform_device_unregister(&knob_device);
#endif
	platform_driver_unregister(&imx6q_knob_driver);
}

module_init(knobs_init);

module_exit(knobs_exit);

MODULE_LICENSE("GPL");



