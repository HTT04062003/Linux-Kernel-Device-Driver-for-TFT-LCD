#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/delay.h> 
#include "tft_driver.h"
#define DRIVER_NAME "tft_spi_driver"

#define GPIO_PIN_LED 17 // Ch      n GPIO    ^qi      ^au khi      ^cn LED c         a TFT
#define GPIO_PIN_A0 18  // Ch      n GPIO A0 c         a TFT
#define PORTRAIT 0
#define LANDSCAPE 1
//



static int major_number;
static uint8_t orientation;

static struct spi_device *tft_spi_device;

static struct gpio_desc *gpio_desc_led;
static struct gpio_desc *gpio_desc_a0;
//
void A0_LOW(void){
     gpiod_direction_output(gpio_desc_a0, 0);
}
void A0_HIGH(void){
     gpiod_direction_output(gpio_desc_a0, 1);
}
//
void TFT_LED_ON(void){
   gpiod_direction_output(gpio_desc_led, 1);
}
void TFT_LED_OFF(void){
   gpiod_direction_output(gpio_desc_led, 0);
}
static int tft_spi_write_byte(struct spi_device *spi,  u8 data)
{
    u8 tx_buf[1];
    int ret;

    tx_buf[0] = data;
    
    ret = spi_write(spi, tx_buf, 1);
    if (ret < 0) {
        printk(KERN_ERR "Failed to write to MAX7219\n");
    }

    return ret;
}
//
static void tft_command( uint8_t cmd){
   int ret;
   A0_LOW();
   ret = tft_spi_write_byte(tft_spi_device, cmd); 
   A0_HIGH();
}
//
static int TFT_Init(void){
     int ret;
           tft_command(0x01);//reset
	    msleep(5);
	    //ret = tft_spi_write_byte(spi, 0x11);//Exit sleep
	    //if (ret < 0) return ret;
	    tft_command(0x11);//Exit sleep
	    msleep(5);
	    //
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xB1);//Frame Rate Control (In normal mode/ Full colors)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    //
	    ret = tft_spi_write_byte(tft_spi_device, 0x01);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2C);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2D);
	    if (ret < 0) return ret;
	    //
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xB2);//Frame Rate Control (In Idle mode/ 8-colors)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x01);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2C);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2D);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xB3);//Frame Rate Control (In Partial mode/ full colors)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x01);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2C);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2D);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x01);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2C);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2D);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xB4);//Display Inversion Control
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x07);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xC0);//Power Control 1
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0xA2);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x02);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x84);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xC1);//Power Control 2
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0xC5);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xC2);//Power Control 3 (in Normal mode/ Full colors)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x0A);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xC3);//Power Control 4 (in Idle mode/ 8-colors)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x8A);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2A);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xC4);//Power Control 5 (in Partial mode/ full-colors)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x8A);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0xEE);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xC5);//VCOM Control 1
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x0E);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0x20);//Display Inversion Off
	    if (ret < 0) return ret;
	    A0_HIGH();
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0x36);//Memory Data Access Control 
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0xA0);//RGB mode + che do man hinh ngang
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0x3A);//Interface Pixel Format
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x05);//16-bit/pixel
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0x2A);//Column address set
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 160);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0x2B);//Row address set
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 128);
	    if (ret < 0) return ret;
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xE0);//Gamm adjustment (+ polarity)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x02);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x1C);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x07);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x12);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x37);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x32);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x29);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2D);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x29);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x25);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2B);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x39);
	    if (ret < 0) return ret;
	    
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x01);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x03);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x10);
	    if (ret < 0) return ret;
	    
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0xE1);//Gamma adjustment(- polarity)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    ret = tft_spi_write_byte(tft_spi_device, 0x01);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x1D);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x07);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x06);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2E);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2C);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x29);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2D);
	    if (ret < 0) return ret;
	    
	    ret = tft_spi_write_byte(tft_spi_device, 0x2E);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x2E);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x37);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x3F);
	    if (ret < 0) return ret;
	    
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x00);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x02);
	    if (ret < 0) return ret;
	    ret = tft_spi_write_byte(tft_spi_device, 0x10);
	    if (ret < 0) return ret;
	    
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0x13);//Partial off (Normal)
	    if (ret < 0) return ret;
	    A0_HIGH();
	    
	    A0_LOW();
	    ret = tft_spi_write_byte(tft_spi_device, 0x29);//Display on
	    if (ret < 0) return ret;
	    A0_HIGH();
	    return 0;
}

static void tft_setWindow(uint8_t x, uint8_t y, uint8_t w, uint8_t h){
   int ret;
   
   
           tft_command(0x2A);
           ret = tft_spi_write_byte(tft_spi_device, 0x00); 
           ret = tft_spi_write_byte(tft_spi_device, x + 0); 
           ret = tft_spi_write_byte(tft_spi_device, 0x00); 
           ret = tft_spi_write_byte(tft_spi_device, x + w); 

           tft_command(0x2B);
           ret = tft_spi_write_byte(tft_spi_device, 0x00); 
           ret = tft_spi_write_byte(tft_spi_device, y + 0); 
           ret = tft_spi_write_byte(tft_spi_device, 0x00); 
           ret = tft_spi_write_byte(tft_spi_device, y + h ); 
   
           tft_command(0x2C);
}
void tft_set_orientation(uint8_t orient){
    int ret;
    tft_command(0x36);
    if(orientation == PORTRAIT){
       ret = tft_spi_write_byte(tft_spi_device, 0x00); //Normal mode
    }
    if(orientation == LANDSCAPE){
       ret = tft_spi_write_byte(tft_spi_device, 0xA0); //Lanscape mode
    }
}
static void draw( uint16_t color){
     int ret;
     uint8_t low;
     uint8_t high;
     low = (uint8_t) color;
     high = color>>8;
     ret = tft_spi_write_byte(tft_spi_device, high);
     ret = tft_spi_write_byte(tft_spi_device, low);
}
static void drawPixel( uint8_t x, uint8_t y, uint16_t color){
     tft_setWindow(x, y, 1, 1);
     draw(color);
}
static void tft_clear(void){
    for(uint32_t i = 0; i <= 160; i++){
        for(uint32_t j = 0; j<=160; j++){
                int ret;
                tft_setWindow(i, j, 1, 1);
                ret = tft_spi_write_byte(tft_spi_device, 0x00);
                ret = tft_spi_write_byte(tft_spi_device, 0x00);
            }
         }
}
static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
    uint32_t i, b, j;

    tft_setWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++)
    {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++)
        {
            if((b << j) & 0x8000)
            {
                drawPixel(x + j, y + i, color);
            }
            else
            {
                drawPixel(x + j, y + i, bgcolor);
            }
        }
    }
}
static void tft_write_string(uint16_t x, uint16_t y, const uint8_t* str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	

    while(*str)
    {
        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }
    
}
static long tft_spi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct tft_gpio_config gpio_cfg;
    tft_window params;
    pixel px;
    uint8_t byte;
    tft_write_string_params write_str;
    switch (cmd) {
    case TFT_IOCTL_RESET:

        break;
    
    case TFT_IOCTL_SET_GPIO:
        if (copy_from_user(&gpio_cfg, (void __user *)arg, sizeof(struct tft_gpio_config))) {
            ret = -EFAULT;
            break;
        }

        // Open LED GPIO
        gpio_desc_led = gpio_to_desc(gpio_cfg.gpio_led);
        if (!gpio_desc_led) {
            ret = -EINVAL;
            break;
        }
        ret = gpiod_direction_output(gpio_desc_led, gpio_cfg.gpio_led_value);
        if (ret) {
            break;
        }
// Open A0 GPIO
        gpio_desc_a0 = gpio_to_desc(gpio_cfg.gpio_a0);
        if (!gpio_desc_a0) {
            ret = -EINVAL;
            break;
        }
        ret = gpiod_direction_output(gpio_desc_a0, gpio_cfg.gpio_a0_value);
        if (ret) {
            break;
        }

        break;
    case TFT_SET_WINDOW: {

            if (copy_from_user(&params, (void __user *)arg, sizeof(tft_window))) {
                return -EFAULT;
            }
            
            tft_setWindow(params.x, params.y, params.w, params.h);
            
            break;
        }
    case TFT_SET_ORIENTATION_LANSCAPE: {
            orientation = LANDSCAPE;
            tft_set_orientation(orientation);
            
            break;
        }
    case TFT_SET_ORIENTATION_PORTRAIT: {
            orientation = PORTRAIT;
            tft_set_orientation(orientation);
            break; 
     }
    case TFT_Clear: {
            tft_clear();
            break;
    }
    case TFT_WriteByte: {
            if (copy_from_user(&byte, (uint8_t *)arg, sizeof(byte))) {
                return -EFAULT;
            }
            tft_spi_write_byte(tft_spi_device, byte);
    }
    case TFT_DRAW_PIXEL: {
          if (copy_from_user(&px, (void __user *)arg, sizeof(pixel))) {
                return -EFAULT;
            }
          drawPixel(px.x, px.y, px.color); 
    }
    case TFT_WRITE_STRING: {
          if (copy_from_user(&write_str, (tft_write_string_params __user *)arg, sizeof(tft_write_string_params))) {
                return -EFAULT;
            }
           tft_write_string(write_str.x, write_str.y, write_str.str,Font_7x10, write_str.color, write_str.bgcolor);
           break;
    }
    default:
        return -ENOTTY;
    }

    return ret;
}

static int tft_spi_open(struct inode *inode, struct file *file)
{
    return 0;
}
static int tft_spi_release(struct inode *inode, struct file *file)
{
    return 0;
}

static const struct file_operations tft_spi_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = tft_spi_ioctl,
    .open = tft_spi_open,
    .release = tft_spi_release,
};

static struct miscdevice tft_spi_device_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DRIVER_NAME,
    .fops = &tft_spi_fops,
};

static int tft_spi_probe(struct sp
i_device *spi)
{
    int ret;
    // Request GPIO LED
    gpio_desc_led = gpio_to_desc(GPIO_PIN_LED);
    if (!gpio_desc_led) {
        dev_err(&spi->dev, "Failed to request LED GPIO pin\n");
        return -EINVAL;
    }
    ret = gpiod_direction_output(gpio_desc_led, 1); // Set LED GPIO as output and turn ON LED
    if (ret) {
        dev_err(&spi->dev, "Failed to set LED GPIO direction\n");
        return ret;
    }

    // Request GPIO A0
    gpio_desc_a0 = gpio_to_desc(GPIO_PIN_A0);
    if (!gpio_desc_a0) {
        dev_err(&spi->dev, "Failed to request A0 GPIO pin\n");
        return -EINVAL;
    }
     ret = gpiod_direction_output(gpio_desc_a0, 1); // Set A0 GPIO as output and turn it ON
    if (ret) {
        dev_err(&spi->dev, "Failed to set A0 GPIO direction\n");
        return ret;
    }

    tft_spi_device = spi;
    
    // Initialize MAX7219
    TFT_LED_ON();
    TFT_Init();
    orientation = PORTRAIT;
    tft_set_orientation(orientation); 
    
    
    // Register misc device
    ret = misc_register(&tft_spi_device_misc);
    if (ret) {
        dev_err(&spi->dev, "Failed to register misc device\n");
        return ret;
    }
    
    // Register character device /dev/tft_spi_driver
    major_number = register_chrdev(0, DRIVER_NAME, &tft_spi_fops);
    if (major_number < 0) {
        dev_err(&spi->dev, "Failed to register a major number\n");
        misc_deregister(&tft_spi_device_misc);
        return ret;
    }

    dev_info(&spi->dev, "TFT SPI driver installed\n");

    return 0;
}
static void tft_spi_remove(struct spi_device *spi)
{
    // Deregister misc device
    misc_deregister(&tft_spi_device_misc);

    // Deregister character device
    unregister_chrdev(major_number, DRIVER_NAME);

    // Release GPIOs
    if (gpio_desc_led) {
        gpiod_set_value(gpio_desc_led, 0);
        gpiod_put(gpio_desc_led);
    }

    if (gpio_desc_a0) {
        gpiod_set_value(gpio_desc_a0, 0);
        gpiod_put(gpio_desc_a0);
    }

    dev_info(&spi->dev, "TFT SPI driver removed\n");
}

static const struct spi_device_id tft_spi_id[] = {
    { "tft", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, tft_spi_id);

static struct spi_driver tft_spi_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },
    .probe      = tft_spi_probe,
    .remove     = tft_spi_remove,
    .id_table   = tft_spi_id,
};
static int __init tft_spi_init(void)
{
    pr_info("Initializing TFT SPI driver\n");
    return spi_register_driver(&tft_spi_driver);
}

static void __exit tft_spi_exit(void)
{
    pr_info("Exiting TFT SPI driver\n");
    spi_unregister_driver(&tft_spi_driver);
}

module_init(tft_spi_init);
module_exit(tft_spi_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("TFT SPI Client Driver");
MODULE_LICENSE("GPL");


