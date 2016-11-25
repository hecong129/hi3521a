#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#define MINI_TCH 1

#define INF(fmt...) \
	{ printk(KERN_INFO "%s %d",__FUNCTION__,__LINE__);\
		printk(KERN_INFO fmt);}

#define DBG(fmt...) \
	{ printk(KERN_DEBUG "%s %d",__FUNCTION__,__LINE__);\
		printk(KERN_INFO fmt);}

/* I2C Driver Code */

#define GPIO_SCL_BASE				0x121B0000
#define GPIO_SDA_BASE				0x121B0000
#define SCL_SHIFT_NUM   			3
#define SDA_SHIFT_NUM   			2
#define SCL							(0x1 << SCL_SHIFT_NUM)   
#define SDA							(0x1 << SDA_SHIFT_NUM)    
#define GPIO_I2C_SCL_REG    		IO_ADDRESS(GPIO_SCL_BASE + (0x1<<(SCL_SHIFT_NUM+2)))  
#define GPIO_I2C_SDA_REG    		IO_ADDRESS(GPIO_SDA_BASE + (0x1<<(SDA_SHIFT_NUM+2)))  
#define GPIO_SCL_DIR				IO_ADDRESS( GPIO_SCL_BASE + 0x400)
#define GPIO_SDA_DIR				IO_ADDRESS( GPIO_SDA_BASE + 0x400)
#define HW_REG(reg)         		*((volatile unsigned int *)(reg))
#define DELAY(us)           		time_delay_us(us)

spinlock_t gpioi2c_lock;

void hi35xx_wr(unsigned int phy_reg, unsigned int val)
{
	*((volatile unsigned int *)IO_ADDRESS(phy_reg)) = (val);
}

unsigned int  hi35xx_rd(unsigned int phy_reg)
{
	return *((volatile unsigned int *)IO_ADDRESS(phy_reg));
}

static void i2c_clr(unsigned char whichline)
{
	unsigned char regvalue;
	
	if(whichline == SCL)
	{
		regvalue = HW_REG(GPIO_SCL_DIR);
		regvalue |= SCL;
		HW_REG(GPIO_SCL_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SCL_REG) = 0;
		return;
	}
	else if(whichline == SDA)
	{
		regvalue = HW_REG(GPIO_SDA_DIR);
		regvalue |= SDA;
		HW_REG(GPIO_SDA_DIR) = regvalue;
		HW_REG(GPIO_I2C_SDA_REG) = 0;
		return;
	}
	else if(whichline == (SDA|SCL))
	{
		regvalue = HW_REG(GPIO_SCL_DIR);
		regvalue |= SCL;
		HW_REG(GPIO_SCL_DIR) = regvalue;		
		HW_REG(GPIO_I2C_SCL_REG) = 0;
		
		regvalue = HW_REG(GPIO_SDA_DIR);
		regvalue |= SDA;
		HW_REG(GPIO_SDA_DIR) = regvalue;		
		HW_REG(GPIO_I2C_SDA_REG) = 0;
		return;
	}
	else
	{
		printk("Error input.\n");
		return;
	}
	
}

static void  i2c_set(unsigned char whichline)
{
	unsigned char regvalue;
	
	if(whichline == SCL)
	{
		regvalue = HW_REG(GPIO_SCL_DIR);
		regvalue |= SCL;
		HW_REG(GPIO_SCL_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SCL_REG) = SCL;
		return;
	}
	else if(whichline == SDA)
	{
		regvalue = HW_REG(GPIO_SDA_DIR);
		regvalue |= SDA;
		HW_REG(GPIO_SDA_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SDA_REG) = SDA;
		return;
	}
	else if(whichline == (SDA|SCL))
	{
		regvalue = HW_REG(GPIO_SCL_DIR);
		regvalue |= SCL;
		HW_REG(GPIO_SCL_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SCL_REG) = SCL;
		
		regvalue = HW_REG(GPIO_SDA_DIR);
		regvalue |= SDA;
		HW_REG(GPIO_SDA_DIR) = regvalue;
		
		HW_REG(GPIO_I2C_SDA_REG) = SDA;
		return;
	}
	else
	{
		printk("Error input.\n");
		return;
	}
}

void time_delay_us(unsigned int usec)
{
	udelay(1);
}

static unsigned char i2c_data_read(void)
{
	unsigned char regvalue;
	
	regvalue = HW_REG(GPIO_SDA_DIR);
	regvalue &= (~SDA);
	HW_REG(GPIO_SDA_DIR) = regvalue;
	DELAY(1);
		
	regvalue = HW_REG(GPIO_I2C_SDA_REG);
	if((regvalue&SDA) != 0)
		return 1;
	else
		return 0;
}

static void i2c_start_bit(void)
{
        DELAY(1);
       	i2c_set(SDA | SCL);
       	DELAY(1);
        i2c_clr(SDA);
        DELAY(1);
}

static void i2c_stop_bit(void)
{
        /* clock the ack */
        DELAY(1);
        i2c_set(SCL);
        DELAY(1); 
        i2c_clr(SCL);  

        /* actual stop bit */
        DELAY(1);
        i2c_clr(SDA);
        DELAY(1);
        i2c_set(SCL);
        DELAY(1);
        i2c_set(SDA);
        DELAY(1);
}

static void i2c_send_byte(unsigned char c)
{
    int i;
    local_irq_disable();
    for (i=0; i<8; i++)
    {
        DELAY(1);
        i2c_clr(SCL);
        DELAY(1);

        if (c & (1<<(7-i)))
            i2c_set(SDA);
        else
            i2c_clr(SDA);

        DELAY(1);
        i2c_set(SCL);
        DELAY(1);
        i2c_clr(SCL);
    }
    DELAY(1);
	// i2c_set(SDA);
    local_irq_enable();
}

static unsigned char i2c_receive_byte(void)
{
    int j=0;
    int i;
    unsigned char regvalue;

    local_irq_disable();
    for (i=0; i<8; i++)
    {
        DELAY(1);
        i2c_clr(SCL);
        DELAY(1);
        i2c_set(SCL);
        
        regvalue = HW_REG(GPIO_SDA_DIR);
        regvalue &= (~SDA);
        HW_REG(GPIO_SDA_DIR) = regvalue;
        DELAY(1);
        
        if (i2c_data_read())
            j+=(1<<(7-i));

        DELAY(1);
        i2c_clr(SCL);
    }
    local_irq_enable();
    DELAY(1);
	// i2c_clr(SDA);
	// DELAY(1);

    return j;
}

static int i2c_receive_ack(void)
{
    int nack;
    unsigned char regvalue;
    
    DELAY(1);
    
    regvalue = HW_REG(GPIO_SDA_DIR);
    regvalue &= (~SDA);
    HW_REG(GPIO_SDA_DIR) = regvalue;
    
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    i2c_set(SCL);
    DELAY(1);
    
    nack = i2c_data_read();

    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
	//  i2c_set(SDA);
	//  DELAY(1);

    if (nack == 0)
        return 1; 

    return 0;
}

void i2c_send_ack(int ack)
{
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    ack? i2c_set(SDA) : i2c_clr(SDA);
    DELAY(1);
    i2c_set(SCL);
    DELAY(1);
    i2c_clr(SCL);
    DELAY(1);
    i2c_clr(SDA);
    DELAY(1);
}

unsigned char gpio_i2ctp_read(unsigned char devaddress, unsigned char addressh,unsigned char addressl)
{

	int rxdata;
	int i;
	spin_lock(&gpioi2c_lock);
	devaddress &= 0xFE;
	for(i = 0;i<16;i++)
	{
		i2c_set(SCL);  DELAY(2);
		i2c_clr(SCL);  DELAY(2);
	}

	i2c_start_bit();
	i2c_send_byte(devaddress);
	i2c_receive_ack();

	i2c_send_byte(addressh);
    i2c_receive_ack();

	i2c_send_byte(addressl);
	i2c_receive_ack(); 

	i2c_start_bit();
	devaddress |= 0x01;
	i2c_send_byte( devaddress );
	i2c_receive_ack();
	rxdata = i2c_receive_byte();
	
	i2c_send_ack(1);
	i2c_stop_bit();

	spin_unlock(&gpioi2c_lock);
	return rxdata;
}


static void gpio_i2ctp_write(unsigned char devaddress, unsigned char addressh,unsigned char addressl, unsigned char data)
{
	int i;
    spin_lock(&gpioi2c_lock);
    devaddress &= 0xFE;
	for(i = 0;i<16;i++)
	{
		i2c_set(SCL);  DELAY(2);
		i2c_clr(SCL);  DELAY(2);
	}

    i2c_start_bit();
    i2c_send_byte(devaddress);

    i2c_receive_ack();
    i2c_send_byte(addressh);

    i2c_receive_ack();
	i2c_send_byte(addressl);

    i2c_receive_ack();
    i2c_send_byte(data); 

    i2c_stop_bit();

	spin_unlock(&gpioi2c_lock);
} 

void gpio_i2ctp_read_buf(unsigned char devaddress, unsigned char addressh,unsigned char addressl,unsigned char *buf,int size)
{
	int i = 0;
	spin_lock(&gpioi2c_lock);
	devaddress &= 0xFE;
	for(i = 0;i < 16; i++)
	{
		i2c_set(SCL);  DELAY(2);
		i2c_clr(SCL);  DELAY(2);
	}
	i = 0;
	i2c_start_bit();
	i2c_send_byte(devaddress);
	i2c_receive_ack();

	i2c_send_byte(addressh);
    i2c_receive_ack();


	i2c_send_byte(addressl);
	i2c_receive_ack(); 


	i2c_start_bit();
	devaddress |= 0x01;
	i2c_send_byte( devaddress );
	i2c_receive_ack();
	do
	{
		buf[i] = i2c_receive_byte();
	
		size--;
		if(size)
			i2c_send_ack(0);
		else
			i2c_send_ack(1);
		i++;
	}while(size);

	
	i2c_stop_bit();

	spin_unlock(&gpioi2c_lock);
}


void gpio_i2ctp_write_buf(unsigned char devaddress, unsigned char addressh,unsigned char addressl,unsigned char *buf,int size)
{
	int i= 0;

    spin_lock(&gpioi2c_lock);
	
    devaddress &= 0xFE;
	for(i = 0;i<16;i++)
	{
		i2c_set(SCL);  DELAY(2);
		i2c_clr(SCL);  DELAY(2);
	}
	
	i = 0;
    i2c_start_bit();
    i2c_send_byte(devaddress);

    i2c_receive_ack();
    i2c_send_byte(addressh);

    i2c_receive_ack();
	i2c_send_byte(addressl);

	do{
    	i2c_receive_ack();
    	i2c_send_byte(buf[i++]); 
	}while(--size);

    i2c_stop_bit();

	spin_unlock(&gpioi2c_lock);
}

static const u8 gt911_table[186] = 
{
									   0x51,0x00,0x04,0x58,0x02,0x05,0x0D,0x00,0x01,
	0x0A,0x28,0x0F,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x87,0x28,0x0A,0x3C,0x3A,0x0C,0x08,0x00,0x00,0x00,0x22,0x02,0x2D,0x00,
	0x01,0x00,0x00,0x00,0x03,0x64,0x32,0x00,0x00,0x00,0x23,0x64,0x94,0xC5,0x02,0x07,
	0x00,0x00,0x04,0xB5,0x27,0x00,0x96,0x30,0x00,0x7E,0x3B,0x00,0x69,0x49,0x00,0x5A,
	0x5A,0x00,0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x14,0x12,0x10,0x0E,0x0C,0x0A,0x08,0x06,0x04,
	0x02,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x1D,0x1E,0x1F,0x20,
	0x21,0x22,0x24,0x26,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xAD,
	0x01
};

// Reserved for far work 
static struct Tp_data
{
	int gpio; 
	int irq;
	int rep;
	int res;
} tp_data;

static struct platform_device gt911_device = {
	.name	= "gt911",
	.id		= -1,
	.dev	= {
		.platform_data = &tp_data,
	}
};

struct goodix_ts_data {
	struct platform_device *dev;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
};

#define GOODIX_MAX_HEIGHT		4096
#define GOODIX_MAX_WIDTH		4096
#define GOODIX_INT_TRIGGER		1
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_MAX_CONTACTS		10

#define GOODIX_CONFIG_MAX_LENGTH	240

/* Register defines */
#define GOODIX_READ_COOR_ADDR		0x814E
#define GOODIX_REG_CONFIG_DATA		0x8047
#define GOODIX_REG_VERSION		0x8140

#define RESOLUTION_LOC		1
#define TRIGGER_LOC		6
#define DEVICE_ADDR  0xBA

static const unsigned long goodix_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};

static int goodix_i2c_read(u8 addr,
				u16 reg, u8 *buf, int len)
{
	gpio_i2ctp_read_buf(addr, reg>>8, reg&0xff, buf, len);
	return 0;
}

static int goodix_ts_read_input_report(struct goodix_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;

	error = goodix_i2c_read(DEVICE_ADDR, GOODIX_READ_COOR_ADDR, data,
				GOODIX_CONTACT_SIZE + 1);
	if (error) {
		INF("I2C transfer error: %d\n", error);
		return error;
	}

	touch_num = data[0] & 0x0f;
	if (touch_num > GOODIX_MAX_CONTACTS)
	{	
		INF("I2C io error: \n");
		return -EPROTO;
	}
	if (touch_num > 1) {
		data += 1 + GOODIX_CONTACT_SIZE;
		error = goodix_i2c_read(DEVICE_ADDR,
					GOODIX_READ_COOR_ADDR +
						1 + GOODIX_CONTACT_SIZE,
					data,
					GOODIX_CONTACT_SIZE * (touch_num - 1));
		if (error)
			return error;
	}

	return touch_num;
}

static void goodix_ts_report_touch(struct goodix_ts_data *ts, u8 *coor_data)
{
	int id = coor_data[0] & 0x0F;
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	int input_w = get_unaligned_le16(&coor_data[5]);

#if MINI_TCH
	//	printk("pre x:%4X y:%4X w;%4X\n",input_x, input_y, input_w);
	input_report_abs(ts->input_dev, ABS_X, input_x);
	input_report_abs(ts->input_dev, ABS_Y, input_y);
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_sync(ts->input_dev);
#else	
	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
#endif

}

/**
 * goodix_process_events - Process incoming events
 *
 * @ts: our goodix_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void goodix_process_events(struct goodix_ts_data *ts)
{
	u8  point_data[1 + GOODIX_CONTACT_SIZE * GOODIX_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
	{	
		INF("irq:%d\n",touch_num);
		return;
	}
	//printk("irq:%d\n",touch_num);
#if MINI_TCH
	if(touch_num) //press
		goodix_ts_report_touch(ts,
				&point_data[1 + GOODIX_CONTACT_SIZE * 0]);
	else //realse
	{	
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_sync(ts->input_dev);
	}	
#else	
	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch(ts,
				&point_data[1 + GOODIX_CONTACT_SIZE * i]);
	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
#endif

}

/**
 * goodix_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	static u8 end_cmd[] = {
		GOODIX_READ_COOR_ADDR >> 8,
		GOODIX_READ_COOR_ADDR & 0xff,
		0
	};
	DBG("irq \n");
	struct goodix_ts_data *ts = dev_id;
	goodix_process_events(ts);
	gpio_i2ctp_write(DEVICE_ADDR, end_cmd[0],end_cmd[1],end_cmd[2]);
	hi35xx_wr(0x121B041C,~0);
	return IRQ_HANDLED;
}

/**
 * goodix_read_config - Read the embedded configuration of the panel
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static void goodix_read_config(struct goodix_ts_data *ts)
{
	u8 config[GOODIX_CONFIG_MAX_LENGTH];
	int error;

	error = goodix_i2c_read(DEVICE_ADDR, GOODIX_REG_CONFIG_DATA,
			      config,
			   GOODIX_CONFIG_MAX_LENGTH);
	if (error) {
		INF("Error reading config (%d), using defaults\n",error);
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
		ts->int_trigger_type = GOODIX_INT_TRIGGER;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	if (!ts->abs_x_max || !ts->abs_y_max) {
		INF("Invalid config, using defaults\n");
		ts->abs_x_max = GOODIX_MAX_WIDTH;
		ts->abs_y_max = GOODIX_MAX_HEIGHT;
	}
}


static int goodix_read_version(u8 addr, u16 *version)
{
	int error;
	u8 buf[6];
	int i,sum;

	error = goodix_i2c_read(DEVICE_ADDR,GOODIX_REG_VERSION, buf, sizeof(buf));
	if (error) {
		INF("read version failed: %d\n", error);
		return error;
	}

	if (version)
		*version = get_unaligned_le16(&buf[4]);
	
	sum = 0;
	for(i=0; i<6; i++)
	{
		sum=sum+buf[i];
	}
	if((sum==0) || (sum== 0xff*6))
	{
		INF("GET IC VERSION Error : check i2c io\n");
	}	

	INF("IC VERSION: %6ph\n", buf);

	return 0;
}

/**
 * goodix_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our goodix_ts_data pointer
 *
 * Must be called during probe
 */
static int goodix_request_input_dev(struct goodix_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->dev->dev);
	if (!ts->input_dev) {
		INF("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) |
				  BIT_MASK(EV_KEY) |
				  BIT_MASK(EV_ABS) |
				  BIT_MASK(EV_REP);

	__set_bit(EV_REP,ts->input_dev->evbit);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

#if MINI_TCH 

	input_set_abs_params(ts->input_dev, ABS_X, 0, 0x3FFF, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, 0x3FFF, 0, 0);
#else	
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
				ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
				ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_mt_init_slots(ts->input_dev, GOODIX_MAX_CONTACTS,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#endif
	ts->input_dev->name = "Goodix GT911 TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = 0x1001;
	ts->input_dev->id.version = 10427;

	error = input_register_device(ts->input_dev);
	if (error) {
		INF("Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

void goodix_i2c_init(void)	
{	
    *((volatile unsigned int *)IO_ADDRESS(0x120F0000 + 0x0F0)) = 0x1;//GPIO6_2
    *((volatile unsigned int *)IO_ADDRESS(0x120F0000 + 0x0F4)) = 0x1;//GPIO6_3
	i2c_set(SCL | SDA); 
    spin_lock_init(&gpioi2c_lock);
}	

#define GPIO_RESET_BASE  0x121B0000
#define GPIO_INT_BASE  0x121B0000

void goodix_gpio_config(void)
{
	u32 reg = 0;
#if 0
	hi35xx_wr(0x121B0000 + (1 << 7),0);	
	reg = hi35xx_rd(0x121B0400);
	reg |= (1 << 7);
	hi35xx_wr(0x121B0400,reg);
	hi35xx_wr(0x121B0000 + (1 << 9),0); 
	msleep(2);
	hi35xx_wr(0x121B0000 + (1 << 7),(1 <<5));
	msleep(40);
	reg = hi35xx_rd(0x121B0400);
	reg &=~(1 << 7);
	hi35xx_wr(0x121B0400,reg);
	msleep(60);
#else
	hi35xx_wr(GPIO_RESET_BASE + (1 << 7),0);	
	
	reg = hi35xx_rd(GPIO_INT_BASE + 0x400);
	reg |= (1 << 7);
	hi35xx_wr(GPIO_INT_BASE+0x400,reg);
	
	hi35xx_wr(GPIO_INT_BASE + (1 << 9),0); 
	msleep(2);
	
	hi35xx_wr(GPIO_RESET_BASE + (1 << 7),(1 <<5));
	msleep(40);

	reg = hi35xx_rd(GPIO_INT_BASE +0x400);
	reg &=~(1 << 7);
	hi35xx_wr(GPIO_INT_BASE+0x400,reg);
	msleep(60);
#endif	
}

void goodix_write_config(void)
{
	int i = 0;
	u16 reg = GOODIX_REG_CONFIG_DATA;

	for(i=0; i<sizeof(gt911_table); i++)
	{
		gpio_i2ctp_write(DEVICE_ADDR, reg>>8,reg&0xff,gt911_table[i]);
		reg++;
	}
}	


void goodix_int_config()
{
	u32 reg = 0;

#if  1 //rising
	 
	reg = hi35xx_rd(0x121B0404);
	reg &=~(1 << 7);
	hi35xx_wr(0x121B0404,reg);
		
	reg = hi35xx_rd(0x121B040C);
	reg &= ~(1 << 7);
	hi35xx_wr(0x121B040C,reg);
	
	reg = hi35xx_rd(0x121B0408);
	reg &=~(1 << 7);
	hi35xx_wr(0x121B0408,reg);

	reg = hi35xx_rd(0x121B0410);
	reg |= (1 << 7);
	hi35xx_wr(0x121B0410,reg);
	
	hi35xx_wr(0x121B041C,~0);
#else
	
	reg = hi35xx_rd(0x121B0404);
	reg &=~(1 << 7);
	hi35xx_wr(0x121B0404,reg);
		
	reg = hi35xx_rd(0x121B040C);
	reg |= (1 << 7);
	hi35xx_wr(0x121B040C,reg);
	
	reg = hi35xx_rd(0x121B0408);
	reg &=~(1 << 7);
	hi35xx_wr(0x121B0408,reg);

	reg = hi35xx_rd(0x121B0410);
	reg |= (1 << 7);
	hi35xx_wr(0x121B0410,reg);
	
#endif	
	
}	

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_data *ts;

	ts = container_of(pdev,struct goodix_ts_data, dev);
	input_unregister_device(ts->input_dev);	
	devm_free_irq(&gt911_device.dev,90,ts);
	devm_kfree(&pdev->dev, ts);
}

static int goodix_ts_probe(struct platform_device *pdev)
{
	struct goodix_ts_data *ts;
	
	int error;
	u16 version_info;
	u32 reg =0;
	struct Tp_data *pdata = pdev->dev.platform_data;

	INF("probe here.\n");
	ts = devm_kzalloc(&pdev->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
	{	
		INF("devm kzamlloc error \n");
		return -ENOMEM;
	}
	ts->dev = pdev;

	platform_set_drvdata(pdev, pdata);

	goodix_i2c_init();

	goodix_gpio_config();

	//product version  id 
	error = goodix_read_version(DEVICE_ADDR, &version_info);
	if (error) {
		INF("Read version failed.\n");
		return error;
	}
	
	goodix_write_config();
	
	goodix_read_config(ts);

	error = goodix_request_input_dev(ts);
	if (error)
	{	
		INF("request input error.\n");
		return error;
	}

	if(devm_request_threaded_irq(&gt911_device.dev,90,NULL,&goodix_ts_irq_handler, goodix_irq_flags[0] | IRQF_ONESHOT ,"gt911",ts))
	{
		INF("request_irq error!\n");
		return -1;
	}
	DBG("request_irq ok!\n");
	
	goodix_int_config();
		
	return 0;
}

static struct platform_driver gt911_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.driver   = {
		.name = "gt911",
		.owner = THIS_MODULE,
	},
};

static int __init gt911_init(void)
{
	int ret;

	INF("init here.\n");
	ret = platform_device_register(&gt911_device);
	if (ret) {
		INF("device register fail.\n");
		return ret;
	}
	
	DBG("device register ok.\n");
	
	ret = platform_driver_register(&gt911_driver);
	if (ret) {
		INF("device driver fail.\n");
		return ret;
	}
	
	DBG("device driver ok.\n");
	
	return 0;
}

static void __exit gt911_exit(void)
{
	INF( "exit here .\n");
	platform_driver_unregister(&gt911_driver);
	platform_device_unregister(&gt911_device);
}

module_init(gt911_init);
module_exit(gt911_exit);

MODULE_AUTHOR("hecong cong.he@qualvision.cn");
MODULE_DESCRIPTION("GT911 Touch Screen Driver");
MODULE_LICENSE("GPL");

