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


static int imx6q_phyrst_probe(struct platform_device *pdev)
{
	int ret = -1;
   	int phypinctrl;
	enum of_gpio_flags flag;
	
	struct device_node *led_node = pdev->dev.of_node;
 	printk("Harry******************now we are beginning to probe the phy_control_pin operation!!!\n");
 	phypinctrl = of_get_named_gpio_flags(led_node, "phyrst_pinctrl", 0, &flag);
	//printk("get gpio id successful\n");

	if(!gpio_is_valid(phypinctrl)){ 

 	 printk("invalid led-gpios: %d\n",phypinctrl) ;

  	return -1 ;

	}
	//gpio_free(dupinctrl);

	if(gpio_request(phypinctrl,"phyrst_pinctrl")){
	printk("gpio %d request failed!\n",phypinctrl);

	return ret ;
	} 
	printk("Harry******************we are now using gpio %d request!\n",phypinctrl);
	printk("Harry******************phy gpio_direction_output successful process1!\n");
        gpio_direction_output(phypinctrl,1);
	udelay(1000);
	gpio_direction_output(phypinctrl,0);
	udelay(10);
	gpio_direction_output(phypinctrl,1);
	printk("Harry******************phy gpio_direction_output successful process2!\n");
         return 0;  
}
	static int imx6q_phyrst_remove(struct platform_device *pdev)

	{
	 //gpio_free(dupinctrl);
       	 return 0;
	}

	static struct of_device_id phyrstpin_table[] = {
		{.compatible = "imx6q,phyrstpinctrl"},
	};
	static struct platform_driver imx6q_phyrst_driver=
        {
        .probe = imx6q_phyrst_probe,
        .remove = imx6q_phyrst_remove,
	.driver={
		.name = "phyrstpinctrl-driver",
		.of_match_table =  phyrstpin_table,
	},
        };
	static int imx6q_phyrst_init(void)  
	{  
   	 printk("imx6q_phyrst_init!\n");  
   	 return platform_driver_register(&imx6q_phyrst_driver);  
	}  
  
	static void imx6q_phyrst_exit(void)  
	{  
       	printk("imx6q_phyrst_exit");  
        platform_driver_unregister(&imx6q_phyrst_driver);  
        return;  
	}



MODULE_LICENSE("GPL");  
module_init(imx6q_phyrst_init);  
module_exit(imx6q_phyrst_exit);   