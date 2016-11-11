#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/sched.h> 
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>

#include <linux/kernel.h>

#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/gfp.h>
#include <asm/irq.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/init.h>

#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_data/atmel.h>

#include "gpio_i2c.h" 
static DECLARE_WAIT_QUEUE_HEAD(interrupt_wait);
static int condition = 0;
spinlock_t gpioi2c_lock;

/*
change log :
1. change the base address 
2. change time_delay_us dly amplify 155/25

hi3531 skt :  
I2C_SCL  -- GPIO12_5 
I2C_SDA  -- GPIO12_4
GPIO12 base addr : 0x20210000


*/


#define GPIO_SCL_BASE 			0x121B0000
#define GPIO_SDA_BASE 			0x121B0000

#define SCL_SHIFT_NUM   			3
#define SDA_SHIFT_NUM   			2
#define SCL                 				(0x1 << SCL_SHIFT_NUM)   
#define SDA                 				(0x1 << SDA_SHIFT_NUM)    

#define GPIO_I2C_SCL_REG    		IO_ADDRESS(GPIO_SCL_BASE + (0x1<<(SCL_SHIFT_NUM+2)))  
#define GPIO_I2C_SDA_REG    		IO_ADDRESS(GPIO_SDA_BASE + (0x1<<(SDA_SHIFT_NUM+2)))  

#define GPIO_SCL_DIR 			IO_ADDRESS( GPIO_SCL_BASE + 0x400)
#define GPIO_SDA_DIR 			IO_ADDRESS( GPIO_SDA_BASE + 0x400)


#define HW_REG(reg)         			*((volatile unsigned int *)(reg))
#define DELAY(us)           			time_delay_us(us)


void hi35xx_wr(unsigned int phy_reg, unsigned int val)
{
	*((volatile unsigned int *)IO_ADDRESS(phy_reg)) = (val);
}


unsigned int  hi35xx_rd(unsigned int phy_reg)
{
	return *((volatile unsigned int *)IO_ADDRESS(phy_reg));
}




/* 
 * I2C by GPIO simulated  clear 0 routine.
 *
 * @param whichline: GPIO control line
 *
 */
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

/* 
 * I2C by GPIO simulated  set 1 routine.
 *
 * @param whichline: GPIO control line
 *
 */
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

/*
 *  delays for a specified number of micro seconds rountine.
 *
 *  @param usec: number of micro seconds to pause for
 *
 */
 // FPGA  APB :  25M
 // ASIC  APB : 155M
 //  翻转5倍
 void time_delay_us(unsigned int usec)
{
	udelay(1);
}

/* 
 * I2C by GPIO simulated  read data routine.
 *
 * @return value: a bit for read 
 *
 */
 
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



/*
 * sends a start bit via I2C rountine.
 *
 */
static void i2c_start_bit(void)
{
        DELAY(1);
       	i2c_set(SDA | SCL);
       	DELAY(1);
        i2c_clr(SDA);
        DELAY(1);
}

/*
 * sends a stop bit via I2C rountine.
 *
 */
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

/*
 * sends a character over I2C rountine.
 *
 * @param  c: character to send
 *
 */
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

/*  receives a character from I2C rountine.
 *
 *  @return value: character received
 *
 */
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

/*  receives an acknowledge from I2C rountine.
 *
 *  @return value: 0--Ack received; 1--Nack received
 *          
 */
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

    // i2c_receive_ack();//add by hyping for tw2815
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

    // i2c_receive_ack();//add by hyping for tw2815
    i2c_stop_bit();

   spin_unlock(&gpioi2c_lock);
}


//int gpioi2c_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
int gpioi2c_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
   	unsigned int val;
	unsigned char device_addr, reg_addrh,reg_addrl;
	unsigned int buf[5];
	i2c_trans_buffer i2c_buf;
	int size;

	switch(cmd)
	{
		case GPIO_I2C_READ_BYTE:
			if (copy_from_user(&buf[0], (unsigned int __user *)arg, sizeof(buf)))
			{
				printk("I2C read  copy from user error!\n");
				return -1;
			}

			device_addr	= (unsigned char)(buf[0] & 0xFF);
			reg_addrh 		= (unsigned char)((buf[1] >> 8) & 0xFF);
			reg_addrl		= (unsigned char)(buf[1] & 0xFF);
			buf[2] = 0;
			buf[2] = gpio_i2ctp_read((unsigned char)device_addr,(unsigned char) reg_addrh,(unsigned char)reg_addrl);
#if 0
			if(reg_addrh == 0x81 && (reg_addrl & 0xFC) == 0x50)
			{
				wait_event_interruptible(interrupt_wait,condition);
  				condition = 0;
			}
#endif
//			printk("r:device_addr=  0x%x reg_addr= 0x%x,buf[0]=0x%0x\n", device_addr, ((reg_addrh<<8) | reg_addrl),buf[2]);
			if (copy_to_user((unsigned int __user *)arg, buf, sizeof(buf)))
			{
				printk("I2C read  copy to user error!\n");
				return -1;
			}
			break;
		
		case GPIO_I2C_WRITE_BYTE:			
			if (copy_from_user(&buf[0], (unsigned int __user *)arg, sizeof(buf)))
			{
				printk("I2C write  copy from user error!\n");
				return -1;
			}
			
			device_addr	= (unsigned char)(buf[0] & 0xFF);
			reg_addrh 		= (unsigned char)((buf[1] >> 8) & 0xFF);
			reg_addrl		= (unsigned char)(buf[1] & 0xFF);
			val				= (unsigned char)(buf[2] & 0xFF);
			gpio_i2ctp_write((unsigned char)device_addr, (unsigned char)reg_addrh,(unsigned char)reg_addrl, (unsigned char)val);
			break;		
		case GPIO_I2C_READ_BUF:
			if (copy_from_user(&i2c_buf, (unsigned int __user *)arg, sizeof(i2c_trans_buffer)))
			{
				printk("I2C read_buf  copy from user error!\n");
				return -1;
			}
		
			device_addr 	= (unsigned char)(i2c_buf.SlaveAddr & 0xFF);
			reg_addrh		= (unsigned char)((i2c_buf.SubAddr >> 8)  & 0xFF);
			reg_addrl		= (unsigned char)(i2c_buf.SubAddr & 0xFF);
			size			= i2c_buf.data_size;

			gpio_i2ctp_read_buf((unsigned char)device_addr,(unsigned char) reg_addrh,(unsigned char)reg_addrl,i2c_buf.buf,size);
		//	for(i=0;i<size;i++)
		//		printk("device_addr=0x%x\tdevice_addr=0x%x\tbuf[%d]=0x%x\n",device_addr,i2c_buf.SubAddr,i,i2c_buf.buf[i]);
#if 0
			if(reg_addrh == 0x81 && (reg_addrl & 0xFC) == 0x50)
			{
				wait_event_interruptible(interrupt_wait,condition);
				condition = 0;
			}
#endif

			if (copy_to_user((unsigned int __user *)arg, &i2c_buf, sizeof(i2c_trans_buffer)))
			{
				printk("I2C read  copy to user error!\n");
				return -1;
			}
	

			break;
		case GPIO_I2C_WRITE_BUF:			
			if (copy_from_user(&i2c_buf, (unsigned int __user *)arg, sizeof(i2c_trans_buffer)))
			{
				printk("I2C write  copy from user error!\n");
				return -1;
			}
			
			device_addr 	= (unsigned char)(i2c_buf.SlaveAddr & 0xFF);
			reg_addrh		= (unsigned char)((i2c_buf.SubAddr >> 8)  & 0xFF);
			reg_addrl		= (unsigned char)(i2c_buf.SubAddr & 0xFF);
			size			= i2c_buf.data_size;
			gpio_i2ctp_write_buf((unsigned char)device_addr, (unsigned char)reg_addrh,(unsigned char)reg_addrl, i2c_buf.buf,size);
			break;		
		default:
			return -1;
	}
    return 0;
}
#if 0
static irqreturn_t i2c_tp_irq(int irq, void *data)
{ 
	//int *id = (int *)data;
	//printk("now is high?%d\n",(HW_REG(0x121B03FC) & (1<<5)));
	/*if(*id!= 9527)
	{
		return 0;
	}*/
	printk("\nrequest_irq !\n");
	condition = 1; 
	wake_up_interruptible(&interrupt_wait);

	return 0;
}
#endif


int gpioi2c_open(struct inode * inode, struct file * file)
{
	int reg;
	static int i = 0;

#if 1
	if(!i)
	{
	    hi35xx_wr(0x121B0000 + (1 << 7),0);	//Reset 0
	

		reg = hi35xx_rd(0x121B0400);
		reg |= (1 << 7);
		hi35xx_wr(0x121B0400,reg);
		hi35xx_wr(0x121B0000 + (1 << 9),0);//Int 0 
		msleep(2);

		hi35xx_wr(0x121B0000 + (1 << 7),(1 <<5));//Reset 1
		msleep(40);

	
		reg = hi35xx_rd(0x121B0400);	//Int 改为输入
		reg &=~(1 << 7);
		hi35xx_wr(0x121B0400,reg);
		msleep(60);
		printk("\n----01----gpioi2c_open !\n");

	/************************************************************/
#if 1//中断配置
		reg = hi35xx_rd(0x121B0404);
		reg &=~(1 << 7);
		hi35xx_wr(0x121B0404,reg);
		
		
		reg = hi35xx_rd(0x121B040C);
		reg |= (1 << 7);
		hi35xx_wr(0x121B040C,reg);
		
		reg = hi35xx_rd(0x121B0408);
		reg &=~(1 << 7);
		hi35xx_wr(0x121B0408,reg);

		hi35xx_wr(0x121B041C,~0);
		
		reg = hi35xx_rd(0x121B0410);
		reg |= (1 << 7);
		hi35xx_wr(0x121B0410,reg);
#endif
		i = 1;
	}

#endif
    return 0;
}
int gpioi2c_close(struct inode * inode, struct file * file)
{
    return 0;
}
#if 1
static irqreturn_t i2c_tp_irq(int irq, void *data)
{ 
	printk("\nrequest_irq !\n");
		
	gpio_i2ctp_write(0xBB, 0x81,0x4e,0);

	hi35xx_wr(0x121B041C,~0);
	return 0;
}
#endif
static struct file_operations gpioi2c_fops = {
    .owner      = THIS_MODULE,
//	.ioctl      = gpioi2c_ioctl,
    .unlocked_ioctl = gpioi2c_ioctl,
    .open       = gpioi2c_open,
    .release    = gpioi2c_close
};


static struct miscdevice gpioi2c_dev = {
   .minor		= MISC_DYNAMIC_MINOR,
   .name		= "gpioi2cTP",
   .fops  		= &gpioi2c_fops,
};



static int __init gpio_i2c_init(void)
{
    int ret;
	int reg;
	u8 test[20]={0};
	int i=0;
	
    ret = misc_register(&gpioi2c_dev);
	printk("I2C read_buf  copy from user ...\n");
	
    if(0 != ret)
    	return -1;

    *((volatile unsigned int *)IO_ADDRESS(0x120F0000 + 0x0F0)) = 0x1;//GPIO6_2
    *((volatile unsigned int *)IO_ADDRESS(0x120F0000 + 0x0F4)) = 0x1;//GPIO6_3
     
	 i2c_set(SCL | SDA); 

    spin_lock_init(&gpioi2c_lock);
	
	// GPIO6_5 Reset
	// GPIO6_7 Int
	 	    hi35xx_wr(0x121B0000 + (1 << 7),0);	//Reset 0
	

		reg = hi35xx_rd(0x121B0400);
		reg |= (1 << 7);
		hi35xx_wr(0x121B0400,reg);
		hi35xx_wr(0x121B0000 + (1 << 9),0);//Int 0 
		msleep(2);

		hi35xx_wr(0x121B0000 + (1 << 7),(1 <<5));//Reset 1
		msleep(40);

	
		reg = hi35xx_rd(0x121B0400);	//Int 改为输入
		reg &=~(1 << 7);
		hi35xx_wr(0x121B0400,reg);
		msleep(60);
		printk("\n----01----gpioi2c_open !\n");



		gpio_i2ctp_write(0xbB, 0x80,0x4d,0xd);
		msleep(700);
		printk("\n----02----gpio setting !\n");
	
		for( i=0 ; i <8; i++)
		{	
		
		
		test[0] = gpio_i2ctp_read(0xBB, 0x80,0x4d);
		printk(" 804d = %2X\n",test[0]);
		
		test[0] = gpio_i2ctp_read(0xBB, 0x81,0x40);
		printk(" 8140 = %2X\n",test[0]);

		test[0] = gpio_i2ctp_read(0xbB, 0x81,0x4e);
		printk(" 814E = %2X\n",test[0]);
		if( test[0] & 0x80)
		{	
			gpio_i2ctp_read_buf(0xbB, 0x81,0x50,test,4);
			printk("All %2X %2X %2X %2X \n", test[0], test[1], test[2], test[3]);
			gpio_i2ctp_write(0xbB, 0x81,0x4e,0);
		}
		else
		{
			printk("ALL : NULL\n");
		}	
		msleep(700);
		}
#if 1	
	if(devm_request_irq(gpioi2c_dev.this_device,90,&i2c_tp_irq,0 | IRQF_ONESHOT ,"gpioi2cTP",NULL))
	{
		printk("request_irq error!\n");
		//disable_irq(90);
		//free_irq(90,NULL);
	}
		printk("request_irq is OK!\n");
		//enable_irq(90);
		reg = hi35xx_rd(0x121B0404);
		reg &=~(1 << 7);
		hi35xx_wr(0x121B0404,reg);
		
		
		reg = hi35xx_rd(0x121B040C);
		reg |= (1 << 7);
		hi35xx_wr(0x121B040C,reg);
		
		reg = hi35xx_rd(0x121B0408);
		reg &=~(1 << 7);
		hi35xx_wr(0x121B0408,reg);

		hi35xx_wr(0x121B041C,~0);
		
		reg = hi35xx_rd(0x121B0410);
		reg |= (1 << 7);
		hi35xx_wr(0x121B0410,reg);

		for( i=0 ; i <8; i++)
		{	
		
		
		test[0] = gpio_i2ctp_read(0xBB, 0x80,0x4d);
		printk(" 804d = %2X\n",test[0]);
		
		test[0] = gpio_i2ctp_read(0xBB, 0x81,0x40);
		printk(" 8140 = %2X\n",test[0]);

		test[0]=gpio_i2ctp_read(0xbB, 0x81,0x4e);
		printk(" 814E %2X\n",test[0]);
		if((test[0] & 0x80 ) == 0)
		{
			printk("no data");
		}
		else
		{	
			gpio_i2ctp_read_buf(0xbB, 0x81,0x50,test,4);
			printk("All %2X %2X %2X %2X \n", test[0], test[1], test[2], test[3]);
			
			gpio_i2ctp_write(0xbB, 0x81,0x4e,0);
		}
		msleep(998);
		}
	
#endif
	
	
	printk("gpio_i2c_tp init is OK!\n");
    return 0;    
}

static void __exit gpio_i2c_exit(void)
{
    misc_deregister(&gpioi2c_dev);
	printk("gpio_i2c_tp exit is OK!\n");
}


module_init(gpio_i2c_init);
module_exit(gpio_i2c_exit);

#ifdef MODULE
//#include <linux/compile.h>
#endif
//MODULE_INFO(build, UTS_VERSION);
MODULE_LICENSE("GPL");





