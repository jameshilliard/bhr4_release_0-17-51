/*
 * IXP4XX EHCI Host Controller Driver
 *
 * Author: Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * Based on "ehci-fsl.c" by Randy Vinson <rvinson@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/platform_device.h>
#include <mach/gpio_alloc.h>
#include <linux/miscdevice.h>

#ifdef CONFIG_USB_SUSPEND
#include <linux/pm_runtime.h>
/* for runtime PM */
struct platform_device *glo_pdev=NULL;
#endif
/*#define PM_RTC_AUTOMATION*/

#ifdef PM_RTC_AUTOMATION
#include <mach/rtc-g2.h>

/* need to change drivers/rtc/rtc-cs75xx.c */
extern int g2_rtc_writel(unsigned int addr, unsigned int val, unsigned int bitmask);
#endif

#define USBLED_START			0
#define USBLED_STOP			1

#if defined(GPIO_USB_STORAGE_LED_0) || defined(GPIO_USB_STORAGE_LED_1)
extern spinlock_t usbled_lock;
extern unsigned long usbled_flags;

extern int usbled_id[2];
extern int usbled_start[2];
extern int usbled_status[2];
extern int usbled_count[2];
extern int usbled_idle_count[2];

#ifdef GPIO_USB_STORAGE_LED_0
static int cs752x_usbled_open_0(struct inode *inode, struct file *file);
static int cs752x_usbled_release_0(struct inode *inode, struct file *file);
static long cs752x_usbled_ioctl_0(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations cs752x_usbled_fops_0 =
{
	.owner          = THIS_MODULE,
	.open           = cs752x_usbled_open_0,
	.release        = cs752x_usbled_release_0,
	.unlocked_ioctl = cs752x_usbled_ioctl_0,
};

static struct miscdevice cs752x_usbled_miscdev_0 =
{
	.minor = 255,
	.name = "usbled0",
	.fops = &cs752x_usbled_fops_0
};
#endif

#ifdef GPIO_USB_STORAGE_LED_1
static int cs752x_usbled_open_1(struct inode *inode, struct file *file);
static int cs752x_usbled_release_1(struct inode *inode, struct file *file);
static long cs752x_usbled_ioctl_1(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations cs752x_usbled_fops_1 =
{
	.owner          = THIS_MODULE,
	.open           = cs752x_usbled_open_1,
	.release        = cs752x_usbled_release_1,
	.unlocked_ioctl = cs752x_usbled_ioctl_1,
};

static struct miscdevice cs752x_usbled_miscdev_1 =
{
	.minor = 255,
	.name = "usbled1",
	.fops = &cs752x_usbled_fops_1
};
#endif
#endif

static const char hcd_name[] = "cs752x_ehci";
#define software_override_sysopmode_en	0x80000000
#define software_override_sysopmode	0x40000000
#define usb_port_reset_1		0x00008000
#define usb_port_reset_0		0x00004000
#define usb_phy1_por			0x00002000
#define usb_phy0_por			0x00001000
#define Hst_ss_ena_incr4		0x02000000
#define Hst_ss_ena_incr8		0x04000000
#define Hst_ss_ena_incr16		0x08000000
#define usb_por				0x20000000
#define Hst_app_start_clk		0x00000002

#define COMMONONN			0x00000040

static int cs752x_ehci_init(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;

#ifndef CONFIG_CORTINA_FPGA
{
	unsigned int temp;

#if defined(CONFIG_CORTINA_CUSTOM_BOARD)


/* custom board patch start HERE: */

/* Review contents of included template below. After review, apply patch to replace 
 * everything between the start HERE: comment above to the and end HERE: comment below
 * including the start HERE: and end HERE: lines themselves. 
 *
 * This patch should also remove the warning below and also change inclusion path to be a location 
 * within YOUR own custom_board/my_board_name tree which will not be overwritten by
 * future Cortina releases.   
 *
 * WARNING: Do NOT remove or change the CONFIG_CORTINA_CUSTOM_BOARD pre-processor definition name above.
 * Cortina will only support custom board builds which use the CONFIG_CORTINA_CUSTOM_BOARD definition.
 */

#warning CUSTOM_BOARD_REVIEW_ME
#include <mach/custom_board/template/usb-ehci/cfg_gpio_usb_vbus_power.h>

/* custom board patch end HERE: */

#else /* CONFIG_CORTINA_CUSTOM_BOARD else */

	/* USB VBUS power enable */
#ifdef GPIO_USB_VBUS_POWER_0
	if (gpio_request(GPIO_USB_VBUS_POWER_0, "GPIO_USB_VBUS_POWER_0")) {
		printk("can't request GPIO_USB_VBUS_POWER_0 %d\n", GPIO_USB_VBUS_POWER_0);
		return -ENODEV;
	}


#if defined(CONFIG_CORTINA_REFERENCE) || defined(CONFIG_CORTINA_REFERENCE_B) || defined(CONFIG_CORTINA_REFERENCE_S) || defined(CONFIG_CORTINA_PON) || defined(CONFIG_CORTINA_WAN) || defined(CONFIG_CORTINA_BHR) || defined(CONFIG_CORTINA_REFERENCE_Q)
	gpio_direction_output(GPIO_USB_VBUS_POWER_0, 0);
	gpio_set_value(GPIO_USB_VBUS_POWER_0, 1);
#else
	gpio_direction_output(GPIO_USB_VBUS_POWER_0, 0);
	gpio_set_value(GPIO_USB_VBUS_POWER_0, 0);
#endif /* (CONFIG_CORTINA_REFERENCE) || defined(CONFIG_CORTINA_REFERENCE_B)...defined(CONFIG_CORTINA_REFERENCE_Q) endif */
#endif /* GPIO_USB_VBUS_POWER_0 endif */


#ifdef GPIO_USB_VBUS_POWER_1
	if (gpio_request(GPIO_USB_VBUS_POWER_1, "GPIO_USB_VBUS_POWER_1")) {
		printk("can't request GPIO_USB_VBUS_POWER_1 %d\n", GPIO_USB_VBUS_POWER_1);
		return -ENODEV;
	}
#if defined(CONFIG_CORTINA_REFERENCE) || defined(CONFIG_CORTINA_REFERENCE_B) || defined(CONFIG_CORTINA_REFERENCE_S) || defined(CONFIG_CORTINA_PON) || defined(CONFIG_CORTINA_WAN) || defined(CONFIG_CORTINA_BHR) || defined(CONFIG_CORTINA_REFERENCE_Q)
	gpio_direction_output(GPIO_USB_VBUS_POWER_1, 0);
	gpio_set_value(GPIO_USB_VBUS_POWER_1, 1);
#else
	gpio_direction_output(GPIO_USB_VBUS_POWER_1, 0);
	gpio_set_value(GPIO_USB_VBUS_POWER_1, 0);
#endif /* defined(CONFIG_CORTINA_REFERENCE) || defined(CONFIG_CORTINA_REFERENCE_B)...defined(CONFIG_CORTINA_REFERENCE_Q) endif */
#endif /* GPIO_USB_VBUS_POWER_1 endif */
#endif /* CONFIG_CORTINA_CUSTOM_BOARD endif */

#if defined(GPIO_USB_STORAGE_LED_0) || defined(GPIO_USB_STORAGE_LED_1)
{
	int ret;

#ifdef GPIO_USB_STORAGE_LED_0
	if (gpio_request(GPIO_USB_STORAGE_LED_0, "GPIO_USB_STORAGE_LED_0")) {
		printk("can't request GPIO_USB_STORAGE_LED_0 %d\n", GPIO_USB_STORAGE_LED_0);
		return -ENODEV;
	}

	gpio_direction_output(GPIO_USB_STORAGE_LED_0, 0);
	gpio_set_value(GPIO_USB_STORAGE_LED_0, 0);

	ret=misc_register(&cs752x_usbled_miscdev_0);
	if (ret)
		printk("%s:misc_register(cs752x_usbled_miscdev_0) fail:%x\n", __FUNCTION__, ret);
#endif

#ifdef GPIO_USB_STORAGE_LED_1
	if (gpio_request(GPIO_USB_STORAGE_LED_1, "GPIO_USB_STORAGE_LED_1")) {
		printk("can't request GPIO_USB_STORAGE_LED_1 %d\n", GPIO_USB_STORAGE_LED_1);
		return -ENODEV;
	}
	gpio_direction_output(GPIO_USB_STORAGE_LED_1, 0);
	gpio_set_value(GPIO_USB_STORAGE_LED_1, 0);

	ret=misc_register(&cs752x_usbled_miscdev_1);
	if (ret)
		printk("%s:misc_register(cs752x_usbled_miscdev_1) fail:%x\n", __FUNCTION__, ret);
#endif
}
#endif /* defined(GPIO_USB_STORAGE_LED_0) || defined(GPIO_USB_STORAGE_LED_1) endif */

	/* disconnect threshold adjustment (bit 31, 30) */
	/* HS transmitter pre-emphasis enable (bit 18) */
	temp = readl(GLOBAL_GLOBAL_USBPHY0_REG0);	/* 0xf00000a0 */
	temp |= 0xc0040000;
	writel(temp, GLOBAL_GLOBAL_USBPHY0_REG0);

	temp = readl(GLOBAL_GLOBAL_USBPHY1_REG0);	/* 0xf00000a4 */
	temp |= 0xc0040000;
	writel(temp, GLOBAL_GLOBAL_USBPHY1_REG0);

#ifndef CONFIG_CORTINA_DISABLE_USB_PHY0_CLOCK
	/* keep COMMONONN 0 */
	temp = readl(GLOBAL_GLOBAL_USBPHY0_REG0);	/* 0xf00000a0 */
	temp &= ~COMMONONN;
	writel(temp, GLOBAL_GLOBAL_USBPHY0_REG0);

	temp = readl(GLOBAL_GLOBAL_USBPHY1_REG0);	/* 0xf00000a4 */
	temp &= ~COMMONONN;
	writel(temp, GLOBAL_GLOBAL_USBPHY1_REG0);
#endif

	/* crystal 12MHz */
/*
	temp = readl(GLOBAL_GLOBAL_USBPHY0_REG0);
	temp &= ~0x00000780;
	writel(temp, GLOBAL_GLOBAL_USBPHY0_REG0);

	temp = readl(GLOBAL_GLOBAL_USBPHY1_REG0);
	temp &= ~0x00000780;
	writel(temp, GLOBAL_GLOBAL_USBPHY1_REG0);
*/

	/* oscillator 12MHz */
/*
	temp = readl(GLOBAL_GLOBAL_USBPHY0_REG0);
	temp &= ~0x00000780;
	temp |= 0x00000200;
	writel(temp, GLOBAL_GLOBAL_USBPHY0_REG0);

	temp = readl(GLOBAL_GLOBAL_USBPHY1_REG0);
	temp &= ~0x00000780;
	temp |= 0x00000200;
	writel(temp, GLOBAL_GLOBAL_USBPHY1_REG0);
*/

	/* oscillator 24MHz */
/*
	temp = readl(GLOBAL_GLOBAL_USBPHY0_REG0);
	temp &= ~0x00000780;
	temp |= 0x00000280;
	writel(temp, GLOBAL_GLOBAL_USBPHY0_REG0);

	temp = readl(GLOBAL_GLOBAL_USBPHY1_REG0);
	temp &= ~0x00000780;
	temp |= 0x00000280;
	writel(temp, GLOBAL_GLOBAL_USBPHY1_REG0);
*/

	/* oscillator 48MHz */
/*
	temp = readl(GLOBAL_GLOBAL_USBPHY0_REG0);
	temp &= ~0x00000780;
	temp |= 0x00000300;
	writel(temp, GLOBAL_GLOBAL_USBPHY0_REG0);

	temp = readl(GLOBAL_GLOBAL_USBPHY1_REG0);
	temp &= ~0x00000780;
	temp |= 0x00000300;
	writel(temp, GLOBAL_GLOBAL_USBPHY1_REG0);
*/
}
#endif /* #ifndef CONFIG_CORTINA_FPGA endif */

	ehci->big_endian_desc = 1;
	ehci->big_endian_mmio = 1;

	ehci->caps = hcd->regs;
	ehci->regs = hcd->regs + HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	hcd->has_tt = 1;
	ehci_reset(ehci);

	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci_port_power(ehci, 1);

	return retval;
}

static const struct hc_driver cs752x_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "cs752x EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,
	.reset			= cs752x_ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#if defined(CONFIG_PM)
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

static int cs752x_ehci_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	const struct hc_driver *driver = &cs752x_ehci_hc_driver;
	struct resource *res;
	int irq;
	int retval;
	unsigned int temp;

	if (usb_disabled())
		return -ENODEV;

#ifndef CONFIG_USB_GADGET_SNPS_DWC_OTG
	/* Add for cs752x Global register init */
	/* ================================================================== */
	/* 1. Setting the port1 in host mode before POR */
	temp = readl(GLOBAL_GLOBAL_USB_REG0);		/* 0xf000008c */
	temp |= software_override_sysopmode_en;
	temp |= software_override_sysopmode;
	writel(temp, GLOBAL_GLOBAL_USB_REG0);

#ifndef CONFIG_CORTINA_FPGA
	/* 2. De-assert USB PHY reset */
	temp = readl(GLOBAL_PHY_CONTROL);		/* 0xf0000014 */
	temp &= ~(usb_phy1_por | usb_phy0_por);
	writel(temp, GLOBAL_PHY_CONTROL);
#endif

	/* 3. Wait for USB PHY Clocks come up */
	udelay(1000);
	printk("%s-host: wait for USB PHY Clocks come up \n", __func__);

	/* 5. De-assert USB Controller POR */
	/* Stone add for USB Power on reset */
	writel(0x0, GLOBAL_GLOBAL_USB_REG1);		/* 0xf0000090 */

	/* 4. Set the USB Host Strap pins before de-asserting the USB Host Reset */
	temp = readl(GLOBAL_GLOBAL_USB_REG0);		/* 0xf000008c */
	/* For 2 host test */
	/*temp |= 0xce000000;*/
	temp |= 0x0e000000;
  	writel(temp, GLOBAL_GLOBAL_USB_REG0);
#else	/* CONFIG_USB_GADGET_SNPS_DWC_OTG */
	/* Add for cs752x Global register init */
	/* ================================================================== */
	/* 1. Setting the OTG in Device mode before POR */
	temp = readl(GLOBAL_GLOBAL_USB_REG0);		/* 0xf000008c */
	temp |= software_override_sysopmode_en;
	temp &= ~software_override_sysopmode;
	writel(temp, GLOBAL_GLOBAL_USB_REG0);

#ifndef CONFIG_CORTINA_FPGA
	/* 2. De-assert USB PHY reset */
	temp = readl(GLOBAL_PHY_CONTROL);		/* 0xf0000014 */
	temp &= ~(usb_phy1_por | usb_phy0_por);
	writel(temp, GLOBAL_PHY_CONTROL);
#endif

	/* 3. Wait for USB PHY Clocks come up */
	udelay(1000);
	printk("%s-gadget: wait for USB PHY Clocks come up \n", __func__);

	/* 4. Set the USB Host Strap pins before de-asserting the USB Host Reset */
	temp = readl(GLOBAL_GLOBAL_USB_REG0);		/* 0xf000008c */
	temp |= Hst_ss_ena_incr4;
	temp |= Hst_ss_ena_incr8;
	temp |= Hst_ss_ena_incr16;
	writel(temp, GLOBAL_GLOBAL_USB_REG0);

	/* 5. De-assert USB Controller POR */
	temp = readl(GLOBAL_GLOBAL_USB_REG1);		/* 0xf0000090 */
	temp &= ~usb_por;
	writel(temp, GLOBAL_GLOBAL_USB_REG1);
	printk("%s: cs752x driver init global register !!!! \n", __func__);
#endif	/* CONFIG_USB_GADGET_SNPS_DWC_OTG */

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto fail_request_resource;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto fail_request_resource;
	}

	hcd->regs = ioremap_nocache(hcd->rsrc_start, hcd->rsrc_len);
	printk("==>%s ehci hcd->regs 0x%p\n", __func__, hcd->regs);
	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto fail_ioremap;
	}

	device_init_wakeup(&pdev->dev, 1);

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;

#ifdef CONFIG_USB_SUSPEND
	/* Probed interfaces are initially active.  They are
	 * runtime-PM-enabled only if the driver has autosuspend support.
	 * They are sensitive to their children's power states.
	 */
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	glo_pdev = pdev;
#endif

	return retval;

fail_add_hcd:
	device_init_wakeup(&pdev->dev, 0);
	iounmap(hcd->regs);

fail_ioremap:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

fail_request_resource:
	usb_put_hcd(hcd);

fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);

	return retval;
}

static int cs752x_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	/* ehci_shutdown(hcd); */

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

#if defined(GPIO_USB_STORAGE_LED_0) || defined(GPIO_USB_STORAGE_LED_1)
#ifdef GPIO_USB_STORAGE_LED_0
	misc_deregister(&cs752x_usbled_miscdev_0);
	gpio_free(GPIO_USB_STORAGE_LED_0);
#endif

#ifdef GPIO_USB_STORAGE_LED_1
	misc_deregister(&cs752x_usbled_miscdev_1);
	gpio_free(GPIO_USB_STORAGE_LED_1);
#endif
#endif

#ifdef GPIO_USB_VBUS_POWER_0
	gpio_free(GPIO_USB_VBUS_POWER_0);
#endif

#ifdef GPIO_USB_VBUS_POWER_1
	gpio_free(GPIO_USB_VBUS_POWER_1);
#endif

	return 0;
}

#ifdef	CONFIG_PM
	#ifdef CONFIG_USB_SUSPEND
int cs752x_ehci_suspend(struct device *dev)
	#else
static int cs752x_ehci_suspend(struct platform_device *pdev, pm_message_t state)
	#endif
{
	unsigned int temp;

#ifdef CONFIG_USB_SUSPEND
	struct platform_device *pdev;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct device *tdev;

	if(!dev)
		tdev = &glo_pdev->dev;
	else
		tdev = dev;

	pdev = to_platform_device(tdev);
	hcd = platform_get_drvdata(pdev);
	ehci = hcd_to_ehci(hcd);

#else
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
#endif

	printk("%s:in\n", __FUNCTION__);

#ifdef CONFIG_CORTINA_DISABLE_USB_PHY0_CLOCK
	temp = ehci_readl(ehci, &ehci->regs->port_status[0]);
	if (temp & PORT_OWNER) {printk("%s:ohci suspend work around\n", __FUNCTION__);
		temp = readl(GLOBAL_GLOBAL_USB_REG0);		/* 0xf000008c */
		temp &= ~Hst_app_start_clk;
		writel(temp, GLOBAL_GLOBAL_USB_REG0);
		msleep(20);
	}
#endif

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(hcd->irq);

#ifdef PM_RTC_AUTOMATION
	enable_irq_wake(IRQ_RTC_ALM);
#endif

	printk("%s:out\n", __FUNCTION__);
	return 0;
}

	#ifdef CONFIG_USB_SUSPEND
int cs752x_ehci_resume(struct device *dev)
	#else
static int cs752x_ehci_resume(struct platform_device *pdev)
	#endif
{
	unsigned int temp;
#ifdef CONFIG_USB_SUSPEND
	struct platform_device *pdev;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct device *tdev;

	if(!dev)
		tdev = &glo_pdev->dev;
	else
		tdev = dev;

	pdev = to_platform_device(tdev);
	hcd = platform_get_drvdata(pdev);
	ehci = hcd_to_ehci(hcd);

#else
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
#endif

	printk("%s:in\n", __FUNCTION__);

#ifdef PM_RTC_AUTOMATION
	disable_irq_wake(IRQ_RTC_ALM);
	g2_rtc_writel(G2_RTC_WKUPPEND, 0, 0xff);
	g2_rtc_writel(G2_RTC_RTCIM, 0x43, 0xff);
#endif

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(hcd->irq);

#ifdef CONFIG_CORTINA_DISABLE_USB_PHY0_CLOCK
	temp = ehci_readl(ehci, &ehci->regs->port_status[0]);
	if (temp & PORT_OWNER) {
		printk("%s:ohci resume work around\n", __FUNCTION__);
		temp = readl(GLOBAL_GLOBAL_USB_REG0);		/* 0xf000008c */
		temp |= Hst_app_start_clk;
		writel(temp, GLOBAL_GLOBAL_USB_REG0);
		msleep(20);
	} else {
		if ((temp & (PORT_PE | PORT_SUSPEND)) == (PORT_PE | PORT_SUSPEND)) {
			printk("%s:ehci resume work around\n", __FUNCTION__);
			temp |= PORT_RESUME;
			ehci_writel(ehci, temp, &ehci->regs->port_status[0]);
			msleep(20);
		}
	}
#endif

	printk("%s:out\n", __FUNCTION__);
	return 0;
}
#endif

#if defined(GPIO_USB_STORAGE_LED_0) || defined(GPIO_USB_STORAGE_LED_1)
static long cs752x_usbled_ioctl(struct file *file, unsigned int cmd, unsigned long arg, int usb_id)
{
	int status;

	switch (cmd) {
	case USBLED_START :
		spin_lock(&usbled_lock);

		usbled_status[usb_id] = 1;
		gpio_set_value(usbled_id[usb_id], usbled_status[usb_id]);

		usbled_start[usb_id] = 1;

		spin_unlock(&usbled_lock);
		break;

	case USBLED_STOP :
		spin_lock(&usbled_lock);

		usbled_status[usb_id] = 0;
		gpio_set_value(usbled_id[usb_id], usbled_status[usb_id]);

		usbled_start[usb_id] = 0;

		spin_unlock(&usbled_lock);
		break;

	default :
		return -1;
	}

	return 0;
}

#ifdef GPIO_USB_STORAGE_LED_0
static int cs752x_usbled_open_0(struct inode *inode, struct file *file)
{
	return 0;
}

static int cs752x_usbled_release_0(struct inode *inode, struct file *file)
{
	return 0;
}

static long cs752x_usbled_ioctl_0(struct file *file, unsigned int cmd, unsigned long arg)
{
	int usb_id;
	int ret;

	usb_id = 0;
	ret=cs752x_usbled_ioctl(file, cmd, arg, usb_id);

	return ret;
}
#endif

#ifdef GPIO_USB_STORAGE_LED_1
static int cs752x_usbled_open_1(struct inode *inode, struct file *file)
{
	return 0;
}

static int cs752x_usbled_release_1(struct inode *inode, struct file *file)
{
	return 0;
}

static long cs752x_usbled_ioctl_1(struct file *file, unsigned int cmd, unsigned long arg)
{
	int usb_id;
	int ret;

	usb_id = 1;
	ret=cs752x_usbled_ioctl(file, cmd, arg, usb_id);

	return ret;
}
#endif
#endif

MODULE_ALIAS("platform:cs752x_ehci");

#ifdef CONFIG_USB_SUSPEND
static int cs752x_ehci_runtime_suspend(struct device *dev)
{
	return 0;

}

static int cs752x_ehci_runtime_resume(struct device *dev)
{
	return 0;
}

static int cs752x_ehci_runtime_idle(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops cs752x_ehci_pm_ops = {
#if 0
	.runtime_suspend =	cs752x_ehci_runtime_suspend,
	.runtime_resume =	cs752x_ehci_runtime_resume,
	.runtime_idle =		cs752x_ehci_runtime_idle,
#endif
	.suspend = cs752x_ehci_suspend,
	.resume = cs752x_ehci_resume,
};

static struct platform_driver cs752x_ehci_driver = {
	.probe = cs752x_ehci_probe,
	.remove = cs752x_ehci_remove,
	.driver = {
		.name = "cs752x_ehci",
		.pm =		&cs752x_ehci_pm_ops,
	},
};

#else  //CONFIG_USB_SUSPEND

static struct platform_driver cs752x_ehci_driver = {
	.probe = cs752x_ehci_probe,
	.remove = cs752x_ehci_remove,
#ifdef	CONFIG_PM
	.suspend = cs752x_ehci_suspend,
	.resume = cs752x_ehci_resume,
#endif
	.driver = {
		.name = "cs752x_ehci",
	},
};

#endif
