/*
  spike.c
 
  Copyright Scott Ellis, 2010
 
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/moduleparam.h>
#include <linux/hrtimer.h>

#define SPI_BUFF_SIZE	16
#define USER_BUFF_SIZE	128

#define SPI_BUS 1
#define SPI_BUS_CS1 1
#define SPI_BUS_SPEED 1000000

#define DEFAULT_WRITE_FREQUENCY 100
static int write_frequency = DEFAULT_WRITE_FREQUENCY;
module_param(write_frequency, int, S_IRUGO);
MODULE_PARM_DESC(write_frequency, "Spike write frequency in Hz");


const char this_driver_name[] = "spike";

struct spike_control {
	struct spi_message msg;
	struct spi_transfer transfer;
	u32 busy;
	u32 spi_callbacks;
	u32 busy_counter;
	u8 *tx_buff; 
};

static struct spike_control spike_ctl;

struct spike_dev {
	struct semaphore spi_sem;
	struct semaphore fop_sem;
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct spi_device *spi_device;
	struct hrtimer timer;
	u32 timer_period_ns;
	u32 running;
	char *user_buff;
};

static struct spike_dev spike_dev;


static void spike_completion_handler(void *arg)
{	
	spike_ctl.spi_callbacks++;
	spike_ctl.busy = 0;
}

static int spike_queue_spi_write(void)
{
	int status;

	if (down_interruptible(&spike_dev.spi_sem))
		return -ERESTARTSYS;

	if (!spike_dev.spi_device) {
		up(&spike_dev.spi_sem);
		return -ENODEV;
	}

	spi_message_init(&spike_ctl.msg);

	/* this gets called when the spi_message completes */
	spike_ctl.msg.complete = spike_completion_handler;
	spike_ctl.msg.context = NULL;

	/* write some toggling bit patterns, doesn't really matter */	
	spike_ctl.tx_buff[0] = 0xAA;
	spike_ctl.tx_buff[1] = 0x55;

	spike_ctl.transfer.tx_buf = spike_ctl.tx_buff;
	spike_ctl.transfer.rx_buf = NULL;
	spike_ctl.transfer.len = 2;

	spi_message_add_tail(&spike_ctl.transfer, &spike_ctl.msg);

	/* spi_async returns immediately */
	status = spi_async(spike_dev.spi_device, &spike_ctl.msg);
	
	/* update the busy flag if spi_async() was good */
	if (status == 0)
		spike_ctl.busy = 1;
	
	up(&spike_dev.spi_sem);

	return status;	
}

static enum hrtimer_restart spike_timer_callback(struct hrtimer *timer)
{
	if (!spike_dev.running) {
		return HRTIMER_NORESTART;
	}

	/* busy means the previous message has not completed */
	if (spike_ctl.busy) {
		spike_ctl.busy_counter++;
	}
	else if (spike_queue_spi_write() != 0) {
		return HRTIMER_NORESTART;
	}

	hrtimer_forward_now(&spike_dev.timer, 
		ktime_set(0, spike_dev.timer_period_ns));
	
	return HRTIMER_RESTART;
}

static ssize_t spike_read(struct file *filp, char __user *buff, size_t count,
			loff_t *offp)
{
	size_t len;
	ssize_t status = 0;

	if (!buff) 
		return -EFAULT;

	if (*offp > 0) 
		return 0;

	if (down_interruptible(&spike_dev.fop_sem)) 
		return -ERESTARTSYS;

	sprintf(spike_dev.user_buff, 
			"%s|%u|%u\n",
			spike_dev.running ? "Running" : "Stopped",
			spike_ctl.spi_callbacks,
			spike_ctl.busy_counter);
		
	len = strlen(spike_dev.user_buff);
 
	if (len < count) 
		count = len;

	if (copy_to_user(buff, spike_dev.user_buff, count))  {
		printk(KERN_ALERT "spike_read(): copy_to_user() failed\n");
		status = -EFAULT;
	} else {
		*offp += count;
		status = count;
	}

	up(&spike_dev.fop_sem);

	return status;	
}

/*
 * We accept two commands 'start' or 'stop' and ignore anything else.
 */
static ssize_t spike_write(struct file *filp, const char __user *buff,
		size_t count, loff_t *f_pos)
{
	size_t len;	
	ssize_t status = 0;

	if (down_interruptible(&spike_dev.fop_sem))
		return -ERESTARTSYS;

	memset(spike_dev.user_buff, 0, 16);
	len = count > 8 ? 8 : count;

	if (copy_from_user(spike_dev.user_buff, buff, len)) {
		status = -EFAULT;
		goto spike_write_done;
	}

	/* we'll act as if we looked at all the data */
	status = count;

	/* but we only care about the first 5 characters */
	if (!strnicmp(spike_dev.user_buff, "start", 5)) {
		if (spike_dev.running) {
			printk(KERN_ALERT "already running\n");
			goto spike_write_done;
		}

		if (spike_ctl.busy) {
			printk(KERN_ALERT "waiting on a spi transaction\n");
			goto spike_write_done;
		}

		spike_ctl.spi_callbacks = 0;		
		spike_ctl.busy_counter = 0;

		hrtimer_start(&spike_dev.timer, 
				ktime_set(0, spike_dev.timer_period_ns),
        	               	HRTIMER_MODE_REL);

		spike_dev.running = 1; 
	} 
	else if (!strnicmp(spike_dev.user_buff, "stop", 4)) {
		hrtimer_cancel(&spike_dev.timer);
		spike_dev.running = 0;
	}

spike_write_done:

	up(&spike_dev.fop_sem);

	return status;
}

static int spike_open(struct inode *inode, struct file *filp)
{	
	int status = 0;

	if (down_interruptible(&spike_dev.fop_sem)) 
		return -ERESTARTSYS;

	if (!spike_dev.user_buff) {
		spike_dev.user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);
		if (!spike_dev.user_buff) 
			status = -ENOMEM;
	}	

	up(&spike_dev.fop_sem);

	return status;
}

static int spike_probe(struct spi_device *spi_device)
{
	if (down_interruptible(&spike_dev.spi_sem))
		return -EBUSY;

	spike_dev.spi_device = spi_device;

	up(&spike_dev.spi_sem);

	return 0;
}

static int spike_remove(struct spi_device *spi_device)
{
	if (spike_dev.running) {
		hrtimer_cancel(&spike_dev.timer);
		spike_dev.running = 0;
	}

	if (down_interruptible(&spike_dev.spi_sem))
		return -EBUSY;
	
	spike_dev.spi_device = NULL;

	up(&spike_dev.spi_sem);

	return 0;
}

static int __init add_spike_device_to_bus(void)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	struct device *pdev;
	char buff[64];
	int status = 0;

	spi_master = spi_busnum_to_master(SPI_BUS);
	if (!spi_master) {
		printk(KERN_ALERT "spi_busnum_to_master(%d) returned NULL\n",
			SPI_BUS);
		printk(KERN_ALERT "Missing modprobe omap2_mcspi?\n");
		return -1;
	}

	spi_device = spi_alloc_device(spi_master);
	if (!spi_device) {
		put_device(&spi_master->dev);
		status = -1;
		printk(KERN_ALERT "spi_alloc_device() failed\n");
		return -1;
	}

	spi_device->chip_select = SPI_BUS_CS1;

	/* Check whether this SPI bus.cs is already claimed */
	snprintf(buff, sizeof(buff), "%s.%u", 
			dev_name(&spi_device->master->dev),
			spi_device->chip_select);

	pdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buff);
 	if (pdev) {
		/* We are not going to use this spi_device, so free it */ 
		spi_dev_put(spi_device);
		
		/* 
		 * There is already a device configured for this bus.cs  
		 * It is okay if it us, otherwise complain and fail.
		 */
		if (pdev->driver && pdev->driver->name && 
				strcmp(this_driver_name, pdev->driver->name)) {
			printk(KERN_ALERT 
				"Driver [%s] already registered for %s\n",
				pdev->driver->name, buff);
			status = -1;
		} 
	} else {
		spi_device->max_speed_hz = SPI_BUS_SPEED;
		spi_device->mode = SPI_MODE_0;
		spi_device->bits_per_word = 8;
		spi_device->irq = -1;
		spi_device->controller_state = NULL;
		spi_device->controller_data = NULL;
		strlcpy(spi_device->modalias, this_driver_name, SPI_NAME_SIZE);
		status = spi_add_device(spi_device);
		
		if (status < 0) {	
			spi_dev_put(spi_device);
			printk(KERN_ALERT "spi_add_device() failed: %d\n", 
				status);		
		}				
	}

	put_device(&spi_master->dev);

	return status;
}

static struct spi_driver spike_driver = {
	.driver = {
		.name =	this_driver_name,
		.owner = THIS_MODULE,
	},
	.probe = spike_probe,
	.remove = __devexit_p(spike_remove),	
};

static int __init spike_init_spi(void)
{
	int error;

	spike_ctl.tx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
	if (!spike_ctl.tx_buff) {
		error = -ENOMEM;
		goto spike_init_error;
	}

	error = spi_register_driver(&spike_driver);
	if (error < 0) {
		printk(KERN_ALERT "spi_register_driver() failed %d\n", error);
		goto spike_init_error;
	}

	error = add_spike_device_to_bus();
	if (error < 0) {
		printk(KERN_ALERT "add_spike_to_bus() failed\n");
		spi_unregister_driver(&spike_driver);
		goto spike_init_error;	
	}

	return 0;

spike_init_error:

	if (spike_ctl.tx_buff) {
		kfree(spike_ctl.tx_buff);
		spike_ctl.tx_buff = 0;
	}

	return error;
}

static const struct file_operations spike_fops = {
	.owner =	THIS_MODULE,
	.read = 	spike_read,
	.write = 	spike_write,
	.open =		spike_open,	
};

static int __init spike_init_cdev(void)
{
	int error;

	spike_dev.devt = MKDEV(0, 0);

	error = alloc_chrdev_region(&spike_dev.devt, 0, 1, this_driver_name);
	if (error < 0) {
		printk(KERN_ALERT "alloc_chrdev_region() failed: %d \n", 
			error);
		return -1;
	}

	cdev_init(&spike_dev.cdev, &spike_fops);
	spike_dev.cdev.owner = THIS_MODULE;
	
	error = cdev_add(&spike_dev.cdev, spike_dev.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: %d\n", error);
		unregister_chrdev_region(spike_dev.devt, 1);
		return -1;
	}	

	return 0;
}

static int __init spike_init_class(void)
{
	spike_dev.class = class_create(THIS_MODULE, this_driver_name);

	if (!spike_dev.class) {
		printk(KERN_ALERT "class_create() failed\n");
		return -1;
	}

	if (!device_create(spike_dev.class, NULL, spike_dev.devt, NULL, 	
			this_driver_name)) {
		printk(KERN_ALERT "device_create(..., %s) failed\n",
			this_driver_name);
		class_destroy(spike_dev.class);
		return -1;
	}

	return 0;
}

static int __init spike_init(void)
{
	memset(&spike_dev, 0, sizeof(spike_dev));
	memset(&spike_ctl, 0, sizeof(spike_ctl));

	sema_init(&spike_dev.spi_sem, 1);
	sema_init(&spike_dev.fop_sem, 1);
	
	if (spike_init_cdev() < 0) 
		goto fail_1;
	
	if (spike_init_class() < 0)  
		goto fail_2;

	if (spike_init_spi() < 0) 
		goto fail_3;

	/* enforce some range to the write frequency */
	if (write_frequency < 1 || write_frequency > 1000) {
		printk(KERN_ALERT "write_frequency reset to %d", 
			DEFAULT_WRITE_FREQUENCY);

		write_frequency = DEFAULT_WRITE_FREQUENCY;
	}


	spike_dev.timer_period_ns = 1000000000 / write_frequency; 
	hrtimer_init(&spike_dev.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	spike_dev.timer.function = spike_timer_callback; 

	return 0;

fail_3:
	device_destroy(spike_dev.class, spike_dev.devt);
	class_destroy(spike_dev.class);

fail_2:
	cdev_del(&spike_dev.cdev);
	unregister_chrdev_region(spike_dev.devt, 1);

fail_1:
	return -1;
}
module_init(spike_init);

static void __exit spike_exit(void)
{
	spi_unregister_driver(&spike_driver);

	device_destroy(spike_dev.class, spike_dev.devt);
	class_destroy(spike_dev.class);

	cdev_del(&spike_dev.cdev);
	unregister_chrdev_region(spike_dev.devt, 1);

	if (spike_ctl.tx_buff)
		kfree(spike_ctl.tx_buff);

	if (spike_dev.user_buff)
		kfree(spike_dev.user_buff);
}
module_exit(spike_exit);

MODULE_AUTHOR("Scott Ellis");
MODULE_DESCRIPTION("spike module - an example SPI driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3");

