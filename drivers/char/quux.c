/*
 *  quux.c: Creates a read-only char device that says how many times
 *  you've read from the dev file, and runs the LEDs.
 */

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>	/* for get_user, put_user */

MODULE_AUTHOR("Will Ware");
MODULE_DESCRIPTION("Char driver example");
MODULE_LICENSE("GPL");

static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);

#define SUCCESS 0
#define DEVICE_NAME "quux"	/* Dev name as it appears in /proc/devices   */
#define BUF_LEN 80		/* Max length of the message from the device */

/*
 * Global variables are declared as static, so are global within the file.
 */

static int Major;		/* Major number assigned to our device driver */
static int Device_Open = 0;	/* Is device open?
				 * Used to prevent multiple access to device */
static char msg[BUF_LEN];	/* The msg the device will give when asked */
static char *msg_Ptr;
static int read_counter = 0;

static struct file_operations fops = {
    .read = device_read,
    .write = device_write,
    .open = device_open,
    .release = device_release
};

/*
 * This function is called when the module is loaded
 */
static int __init quux_init(void)
{
    Major = register_chrdev(0, DEVICE_NAME, &fops);

    if (Major < 0) {
        printk(KERN_ALERT "Registering char device failed with %d\n", Major);
        return Major;
    }

    printk(KERN_INFO "quux major=%d\n", Major);
    printk(KERN_INFO "Try: mknod /dev/%s c %d 0\n", DEVICE_NAME, Major);

    // LeopardBoard LEDs
    // these functions return zero when things go well
    // this is the only device that will want to talk to the LEDs, so I'm
    // not bothering to check return values as I should.
    gpio_request(57, "LED1");
    gpio_direction_output(57, 0);
    gpio_request(58, "LED2");
    gpio_direction_output(58, 0);

    return SUCCESS;
}

/*
 * This function is called when the module is unloaded
 */
static void __exit quux_exit(void)
{
    /*
     * Unregister the device
     */
    unregister_chrdev(Major, DEVICE_NAME);
    gpio_free(57);
    gpio_free(58);
}

/*
 * Methods
 */

/*
 * Called when a process tries to open the device file, like
 * "cat /dev/mycharfile"
 */
static int device_open(struct inode *inode, struct file *file)
{
    if (Device_Open)
        return -EBUSY;

    Device_Open++;
    if (read_counter == 0)
        sprintf(msg, "Kernel module says: Hello world!\n");
    else if (read_counter == 1)
        sprintf(msg, "I already said Hello world!\n");
    else
        sprintf(msg, "I already said %d times Hello world!\n", read_counter);
    msg_Ptr = msg;
    try_module_get(THIS_MODULE);

    return SUCCESS;
}

/*
 * Called when a process closes the device file.
 */
static int device_release(struct inode *inode, struct file *file)
{
    Device_Open--;		/* We're now ready for our next caller */

    /*
     * Decrement the usage count, or else once you opened the file, you'll
     * never get get rid of the module.
     */
    module_put(THIS_MODULE);

    return 0;
}

/*
 * Called when a process, which already opened the dev file, attempts to
 * read from it.
 */
static ssize_t device_read(struct file *filp,	/* see include/linux/fs.h   */
			   char *buffer,	/* buffer to fill with data */
			   size_t length,	/* length of the buffer     */
			   loff_t * offset)
{
    /*
     * Number of bytes actually written to the buffer
     */
    int bytes_read = 0;

    /*
     * If we're at the end of the message,
     * return 0 signifying end of file
     */
    if (*msg_Ptr == 0)
        return 0;

    gpio_set_value(57, read_counter & 1);
    gpio_set_value(58, read_counter & 2);
    read_counter++;

    /*
     * Actually put the data into the buffer
     */
    while (length && *msg_Ptr) {

        /*
         * The buffer is in the user data segment, not the kernel
         * segment so "*" assignment won't work.  We have to use
         * put_user which copies data from the kernel data segment to
         * the user data segment.
         */
        put_user(*(msg_Ptr++), buffer++);

        length--;
        bytes_read++;
    }

    /*
     * Most read functions return the number of bytes put into the buffer
     */
    return bytes_read;
}

/* 
 * This function is called when somebody tries to write into our device file.
 * This is cribbed and modified from
 * http://www.linuxtopia.org/online_books/Linux_Kernel_Module_Programming_Guide/x872.html
 */
static ssize_t
device_write(struct file *file,
	     const char __user * buffer, size_t length, loff_t * offset)
{
	char x;

#ifdef DEBUG
	printk(KERN_INFO "device_write(%p,%s,%d)", file, buffer, length);
#endif

	// get the first byte of the text coming in
	get_user(x, buffer);

	// use the two low bits to set the LEDs
	gpio_set_value(57, x & 1);
	gpio_set_value(58, x & 2);

	/* 
	 * Return the number of input characters used 
	 */
	return length;
}

module_init(quux_init);
module_exit(quux_exit);
