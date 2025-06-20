#include <stdio.h>
#include <8052.h>
#include "I2C.h"

__sbit __at (0xB7) SDA; // P3.7
__sbit __at (0xB6) SCL; // P3.6 

void I2C_init(void);
void I2C_delay(void);
void I2C_start(void);
void I2C_stop(void);
void I2C_write(unsigned char data);
unsigned char I2C_read(void);

void I2C_init(void)
{
    SDA = 1;  
    SCL = 1;  
}

void I2C_delay(void)
{
    unsigned int i;
    for (i = 0; i < 50; i++); 
}

void I2C_start(void)
{
    SDA = 1;
    SCL = 1;
    I2C_delay();
    SDA = 0;
    I2C_delay();
    SCL = 0;
}


void I2C_stop(void)
{
    SCL = 0;
    SDA = 0;
    I2C_delay();
    SCL = 1;
    I2C_delay();
    SDA = 1;
    I2C_delay();
}

void I2C_write(unsigned char data)
{
    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        SDA = (data & 0x80) ? 1 : 0;
        data <<= 1;
        SCL = 1;
        I2C_delay();
        SCL = 0;
        I2C_delay();
    }
    
    SDA = 1; 
    SCL = 1;
    I2C_delay();
    while (SDA == 1);
    SCL = 0;
    I2C_delay();
}

unsigned char I2C_read(void)
{
    unsigned char i, data = 0;
    SDA = 1; 
    for (i = 0; i < 8; i++)
    {
        SCL = 1;
        I2C_delay();
        data <<= 1;
        if (SDA) 
            data |= 0x01; 
        SCL = 0;
        I2C_delay();
    }
    SDA = 1;
    SCL = 1;
    I2C_delay();
    SCL = 0;
    return data;
}