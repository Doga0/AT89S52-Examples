#include <8052.h>
#include "I2C.h"


void lcd_slave(unsigned char address);
void delay_ms(unsigned int n);
void lcd_send_cmd(unsigned char cmd);
void lcd_send_data(unsigned char data);
void lcd_send_str(const char *p);
void lcd_init(void);

unsigned char slave_add;


void lcd_slave(unsigned char address) {
    slave_add = address;
}

void delay_ms(unsigned int n) { 
    unsigned int i, j; 
    for (i = 0; i < n; i++) { 
        for (j = 0; j < 123; j++) { 
            __asm nop __endasm; 
        } 
    }
}

void lcd_send_cmd(unsigned char cmd) {
    unsigned char cmd_u = cmd & 0xF0;
    unsigned char cmd_l = (cmd << 4) & 0xF0;

    I2C_start();
    I2C_write(slave_add);
    I2C_write(cmd_u | 0x0C);
    delay_ms(1);
    I2C_write(cmd_u | 0x08);
    delay_ms(1);
    I2C_write(cmd_l | 0x0C);
    delay_ms(1);
    I2C_write(cmd_l | 0x08);
    delay_ms(10);
    I2C_stop();
}

void lcd_send_data(unsigned char dataw) {
    unsigned char data_u = dataw & 0xF0;
    unsigned char data_l = (dataw << 4) & 0xF0;

    I2C_start();
    I2C_write(slave_add);
    I2C_write(data_u | 0x0D);
    delay_ms(1);
    I2C_write(data_u | 0x09);
    delay_ms(1);
    I2C_write(data_l | 0x0D);
    delay_ms(1);
    I2C_write(data_l | 0x09);
    delay_ms(10);
    I2C_stop();
}


void lcd_send_str(const char *p) {
    while (*p) {
        lcd_send_data((unsigned char)*p++);
    }
}

// Initialize LCD in 4-bit mode
void lcd_init(void) {
    delay_ms(50);            
    lcd_send_cmd(0x02);      
    lcd_send_cmd(0x28);      
    lcd_send_cmd(0x0C);      
    lcd_send_cmd(0x06);      
    lcd_send_cmd(0x01);  
    delay_ms(10);
    lcd_send_cmd(0x80);   
}

void main(void) {
    unsigned char slave1 = 0x4E;
    delay_ms(100); 
    I2C_init();            
    lcd_slave(slave1);     
    lcd_init();           

    lcd_send_cmd(0x80);
    lcd_send_str("Hello World!");

    while (1);
}
