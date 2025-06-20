#ifndef I2C_H
#define I2C_H

#include <8052.h>

extern __sbit __at (0xB6) SCL;
extern __sbit __at (0xB7) SDA;

// Function declarations
void I2C_init(void);
void I2C_delay(void);
void I2C_start(void);
void I2C_stop(void);
void I2C_write(unsigned char data);
unsigned char I2C_read(void);

#endif // I2C_H
