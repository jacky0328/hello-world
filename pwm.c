/*
* Usage:
 *        Load driver:  insmod ./dht11km.ko <optional variables>
 *                              i.e.    sudo insmod ./pwm.ko gpio_pin=2
 *                Set up device file to read from (i.e.):
 *                                      sudo mknod /dev/pwm c 80 0
 *                                      mknod /dev/myfile c <driverno> 0        - to set the output to your ow

*/


#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include <linux/irq.h>
#include <linux/fcntl.h>
#include <linux/spinlock.h>

#include <linux/fs.h>
#include <asm/uaccess.h>        // for put_user

// include RPi harware specific constants
#include <mach/platform.h>

#define INTERRUPT_GPIO0  79 

#define PWM_DRIVER_NAME "pwm"

#define SUCCESS 0



// set GPIO pin g as input
#define GPIO_DIR_INPUT(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
// set GPIO pin g as output
#define GPIO_DIR_OUTPUT(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
// get logical value from gpio pin g
#define GPIO_READ_PIN(g) (*(gpio+13) & (1<<(g))) && 1
// sets   bits which are 1 ignores bits which are 0
#define GPIO_SET_PIN(g) *(gpio+7) = 1<<g;
// clears bits which are 1 ignores bits which are 0
#define GPIO_CLEAR_PIN(g) *(gpio+10) = 1<<g;
// Clear GPIO interrupt on the pin we use
#define GPIO_INT_CLEAR(g) *(gpio+16) = (*(gpio+16) | (1<<g));
// GPREN0 GPIO Pin Rising Edge Detect Enable/Disable
#define GPIO_INT_RISING(g,v) *(gpio+19) = v ? (*(gpio+19) | (1<<g)) : (*(gpio+19) ^ (1<<g))
// GPFEN0 GPIO Pin Falling Edge Detect Enable/Disable
#define GPIO_INT_FALLING(g,v) *(gpio+22) = v ? (*(gpio+22) | (1<<g)) : (*(gpio+22) ^ (1<<g))


static spinlock_t lock;

// Forward declarations
static int read_pwm(struct inode *, struct file *);
static int close_pwm(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static void clear_interrupts(void);
static int setup_interrupts(void);



static struct timeval lasttv = {0, 0};

static int Device_Open = 0;                             // Is device open?  Used to prevent multiple access to device
static char msg[80];                               // The msg the device will give when asked
static char *msg_Ptr;

static int gpio_pin = 4;                //Default GPIO pin
static int driverno = 80;               //Default driver number

static int pwm_data =0;

//Operations that can be performed on the device
static struct file_operations fops = {
        .read = device_read,
        .open = read_pwm,
        .release = close_pwm
};

// Possible valid GPIO pins
int valid_gpio_pins[] = { 0, 1, 4, 8, 7, 9, 10, 11, 14, 15, 17, 18, 21, 22, 23, 24, 25 };

volatile unsigned *gpio;


// IRQ handler - where the timing takes place
static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs)
{
        struct timeval tv;
        long deltv;
        int signal;

        // use the GPIO signal level
        signal = GPIO_READ_PIN(gpio_pin);

        /* reset interrupt */
        GPIO_INT_CLEAR(gpio_pin);

        if(signal==1) // positive rising
            do_gettimeofday(&lasttv);
        else
        {          // negative rising
            do_gettimeofday(&tv);
                // get time since last interrupt in microseconds
            deltv = tv.tv_sec-lasttv.tv_sec;
            pwm_data = (unsigned int) (deltv*1000000 + (tv.tv_usec - lasttv.tv_usec));
        } 
        return IRQ_HANDLED;
}


static int read_pwm(struct inode *inode, struct file *file)
{

        if (Device_Open)
                return -EBUSY;

        try_module_get(THIS_MODULE);            //Increase use count

        Device_Open++;

        // Take data low for min 18mS to start up DHT11

        GPIO_DIR_INPUT(gpio_pin);   // Change to read

        // Set up interrupts
        setup_interrupts();
 
        mdelay(20);
        sprintf(msg, "pwm : %d\n", pwm_data);
        msg_Ptr = msg;



        return SUCCESS;
}



static ssize_t device_read(struct file *filp,   // see include/linux/fs.h
                           char *buffer,        // buffer to fill with data
                           size_t length,       // length of the buffer
                           loff_t * offset)
{
        // Number of bytes actually written to the buffer
        int bytes_read = 0;
        // If we're at the end of the message, return 0 signifying end of file

        // Actually put the data into the buffer
                // put_user which copies data from the kernel data segment to the user data segment.
      while (length && *msg_Ptr) {

                // put_user which copies data from the kernel data segment to the user data segment.
                put_user(*(msg_Ptr++), buffer++);

                length--;
                bytes_read++;
        }

        printk("pwm : %d\n",pwm_data);

        // Return the number of bytes put into the buffer
        return bytes_read;
}



// Called when a process closes the device file.
static int close_pwm(struct inode *inode, struct file *file)
{
        // Decrement the usage count, or else once you opened the file, you'll never get get rid of the module.
        module_put(THIS_MODULE);
        Device_Open--;

        clear_interrupts();
        //printk(KERN_INFO PWM_DRIVER_NAME ": Device release (close_pwm)\n");

        return 0;
}


// Clear the GPIO edge detect interrupts
static void clear_interrupts(void)
{
        unsigned long flags;

        spin_lock_irqsave(&lock, flags);

        // GPREN0 GPIO Pin Rising Edge Detect Disable
        GPIO_INT_RISING(gpio_pin, 0);

        // GPFEN0 GPIO Pin Falling Edge Detect Disable
        GPIO_INT_FALLING(gpio_pin, 0);

        spin_unlock_irqrestore(&lock, flags);

        free_irq(INTERRUPT_GPIO0, (void *) gpio);
}

// Initialise GPIO memory
static int init_port(void)
{
        /*
        // reserve GPIO memory region.
        if (request_mem_region(GPIO_BASE, SZ_4K, DHT11_DRIVER_NAME) == NULL) {
                printk(KERN_ERR PWM_DRIVER_NAME ": unable to obtain GPIO I/O memory address\n");
                return -EBUSY;
        }
        */

        // remap the GPIO memory
        if ((gpio = ioremap_nocache(GPIO_BASE, SZ_4K)) == NULL) {
                printk(KERN_ERR PWM_DRIVER_NAME ": failed to map GPIO I/O memory\n");
                return -EBUSY;
        }

        return 0;
}


static int __init pwm_init_module(void)
{
        int result;
        int i;


        printk("GPIO_PIN : %d\n",gpio_pin);

        // check for valid gpio pin number
        result = 0;
        for(i = 0; (i < ARRAY_SIZE(valid_gpio_pins)) && (result != 1); i++) {
                if(gpio_pin == valid_gpio_pins[i])
                        result++;
        }

        if (result != 1) {
                result = -EINVAL;
                printk(KERN_ERR PWM_DRIVER_NAME ": invalid GPIO pin specified!\n");
                goto exit_rpi;
        }

    result = register_chrdev(driverno, PWM_DRIVER_NAME, &fops);

        if (result < 0) {
          printk(KERN_ALERT PWM_DRIVER_NAME "Registering pwm driver failed with %d\n", result);
          return result;
        }

        printk(KERN_INFO PWM_DRIVER_NAME ": driver registered!\n");

        result = init_port();
        if (result < 0)
                goto exit_rpi;

        return 0;

exit_rpi:

        return result;
}

static void __exit pwm_exit_module(void)
{
        // release mapped memory and allocated region
        if(gpio != NULL) {
                iounmap(gpio);
        //      release_mem_region(GPIO_BASE, SZ_4K);
                printk(PWM_DRIVER_NAME ": cleaned up resources\n");
        }

        // Unregister the driver
        unregister_chrdev(driverno, PWM_DRIVER_NAME);
        printk(PWM_DRIVER_NAME ": cleaned up module\n");
}


static int setup_interrupts(void)
{
        int result;
        unsigned long flags;

        result = request_irq(INTERRUPT_GPIO0, (irq_handler_t) irq_handler, IRQF_SHARED, PWM_DRIVER_NAME, (void*) gpio);

        switch (result) {
        case -EBUSY:
                printk(KERN_ERR PWM_DRIVER_NAME ": IRQ %d is busy\n", INTERRUPT_GPIO0);
                return -EBUSY;
        case -EINVAL:
                printk(KERN_ERR PWM_DRIVER_NAME ": Bad irq number or handler\n");
                return -EINVAL;
        default:
                printk(KERN_INFO PWM_DRIVER_NAME      ": Interrupt %04x obtained\n", INTERRUPT_GPIO0);
                break;
        };

        spin_lock_irqsave(&lock, flags);

        // GPREN0 GPIO Pin Rising Edge Detect Enable
        GPIO_INT_RISING(gpio_pin, 1);
        // GPFEN0 GPIO Pin Falling Edge Detect Enable
        GPIO_INT_FALLING(gpio_pin, 1);

        // clear interrupt flag
        GPIO_INT_CLEAR(gpio_pin);

        spin_unlock_irqrestore(&lock, flags);

        return 0;
}



module_init(pwm_init_module);
module_exit(pwm_exit_module);

MODULE_DESCRIPTION("PWM read value by Raspberry Pi GPIO.");
MODULE_AUTHOR("Jacky chi");
MODULE_LICENSE("GPL");

// Command line paramaters for gpio pin and driver major number
module_param(gpio_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_pin, "GPIO pin to use");
module_param(driverno, int, S_IRUGO);
MODULE_PARM_DESC(driverno, "Driver handler major value");
