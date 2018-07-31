#include <linux/of.h>
#include <linux/module.h>  
#include <linux/fs.h>  
#include <linux/cdev.h>  
#include <linux/device.h>  
#include <linux/platform_device.h>  
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>

//static int major = 250;  
//static int minor=0;  
//static dev_t devno;  
//static struct class *cls;  
//static struct device *test_device;
//static struct resource *res;  

static int imx6q_led_probe(struct platform_device *pdev)
{
	int ret = -1;
   	int audiopinctrl;
	enum of_gpio_flags flag;
	
	struct device_node *led_node = pdev->dev.of_node;
 	//printk("now we are beginning to probe the du_control_pin operation!!!\n");
 	audiopinctrl = of_get_named_gpio_flags(led_node, "audio_paenable_ctrl", 0, &flag);
	//printk("get gpio id successful\n");

	if(!gpio_is_valid(audiopinctrl)){ 

 	 printk("invalid led-gpios: %d\n",audiopinctrl) ;

  	return -1 ;

	}
	//gpio_free(audiopinctrl);

	if(gpio_request(audiopinctrl,"audio_paenable_ctrl")){
	printk("gpio %d request failed!\n",audiopinctrl);

	return ret ;
	} 
	printk("we are now using gpio %d request!\n",audiopinctrl);
	//printk("gpio_direction_output successful process1!\n");
        gpio_direction_output(audiopinctrl,1);
	//printk("gpio_direction_output successful process2!\n");
	

         return 0;  
}
	static int imx6q_led_remove(struct platform_device *pdev)

	{
	 //gpio_free(audiopinctrl);
       	 return 0;
	}

	static struct of_device_id led_table[] = {
		{.compatible = "imx6q,audiopinctrl"},
	};
	static struct platform_driver imx6q_led_driver=
        {
        .probe = imx6q_led_probe,
        .remove = imx6q_led_remove,
	.driver={
		.name = "rcardupinctrl-driver",
		.of_match_table =  led_table,
	},
        };
	static int imx6q_led_init(void)  
	{  
   	 printk("imx6q_audiopinctrl_init!\n");  
   	 return platform_driver_register(&imx6q_led_driver);  
	}  
  
	static void imx6q_led_exit(void)  
	{  
       	printk("imx6q_audiopinctrl_exit");  
        platform_driver_unregister(&imx6q_led_driver);  
        return;  
	}



MODULE_LICENSE("GPL");  
module_init(imx6q_led_init);  
module_exit(imx6q_led_exit);   
