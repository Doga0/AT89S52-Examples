#include <8052.h>
#include <stdio.h>
#include "I2C.h"


__sbit __at (0xB2) MAX30102_INT; // P3.2

#define MAX30102_I2C_ADDR 0x57

#define I2C_ADDRESS											0x57
#define MODE_CONFIG 										0x09
#define SPO2_CONFIG  										0x0A
#define FIFO_WR_PTR  										0x04
#define FIFO_RD_PTR  										0x06
#define OVF_COUNTER  										0x05
#define FIFO_CONFIG  										0x08

#define FIFO_CONFIG_SMP_AVE 5
#define FIFO_CONFIG_ROLL_OVER_EN 4
#define FIFO_CONFIG_FIFO_A_FULL 0
#define SAMPLE_LEN_MAX 32
 
#define SPO2_LED_PW  0     
#define SPO2_ADC_RGE 5
#define SPO2_SR 2

#define LED_IR_PA1 0x0c
#define LED_RED_PA2 0x0d

#define MODE_CONFIG 0x09

#define INTERRUPT_ENABLE_1 0x02
#define INTERRUPT_A_FULL 7

#define INTERRUPT_DIE_TEMP_RDY 1
#define INTERRUPT_ENABLE_2 0x03
#define DIE_TEMP_EN 1
#define DIE_TEMP_CONFIG 0x21

#define INTERRUPT_STATUS_1 0x00
#define DIE_TINT 0x1f
#define DIE_TFRAC 0x20

#define FIFO_WR_PTR 0x04
#define FIFO_RD_PTR 0x06
#define FIFO_DATA 0x07
#define INTERRUPT_PPG_RDY 6
#define INTERRUPT_ALC_OVF 5

// led pulse width
typedef enum {
    max30102_pw_15_bit = 0,
    max30102_pw_16_bit,
    max30102_pw_17_bit,
    max30102_pw_18_bit
} max30102_led_pw_t;

// adc resolution
typedef enum max30102_adc_t
{
    max30102_adc_2048,
    max30102_adc_4096,
    max30102_adc_8192,
    max30102_adc_16384
} max30102_adc_t;

// sampling rate
typedef enum max30102_sr_t
{
    max30102_sr_50,
    max30102_sr_100,
    max30102_sr_200,
    max30102_sr_400,
    max30102_sr_800,
    max30102_sr_1000,
    max30102_sr_1600,
    max30102_sr_3200
} max30102_sr_t;

typedef unsigned long uint32_t; 

typedef struct {
    unsigned long _ir_samples[SAMPLE_LEN_MAX];
    uint32_t _red_samples[SAMPLE_LEN_MAX];
    unsigned char _interrupt_flag;
} max30102_t;

max30102_t sensor; 

typedef enum {
    SMP_AVE_1,
    SMP_AVE_2,
    SMP_AVE_4,
    SMP_AVE_8,
    SMP_AVE_16,
    SMP_AVE_32
} max30102_smp_ave_t;

typedef enum max30102_mode_t
{
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;


void UART_Init(void);
void GPIO_Init(void);
void max30102_init(void);
void max30102_reset(void);
void max30102_fifo_config(unsigned char smp_ave, unsigned char roll_over_en, unsigned char fifo_a_full);
void max30102_write(unsigned char reg, unsigned char *buf, unsigned char buflen);
void max30102_read(unsigned char reg, unsigned char *buf, unsigned char buflen);
void max30102_set_led_pulse_width(max30102_led_pw_t pw);
void max30102_set_adc_resolution(max30102_adc_t adc);
void max30102_set_sampling_rate(max30102_sr_t sr);
void max30102_set_led_current_1(float ma);
void max30102_set_led_current_2(float ma);
void max30102_set_mode(max30102_mode_t mode);
void max30102_set_a_full(unsigned char enable);
void max30102_set_die_temp_rdy(unsigned char enable);
unsigned char max30102_has_interrupt(void);
void max30102_read_fifo(void);
void max30102_read_temp(unsigned char *temp_int, unsigned char *temp_frac);
void max30102_interrupt_handler(void);


void UART_Init(void)
{
    TMOD = 0x20;    
    TH1 = 0xFD;     
    SCON = 0x50;   
    TR1 = 1;       
}

void GPIO_Init(void)
{
    P1 = 0xFF;    
    P3 |= 0x04;   

    IT0 = 1;      
    EX0 = 1;      
    EA = 1;       
}

void max30102_init(void)
{
		unsigned char i;
    sensor._interrupt_flag = 0;
		
    for (i = 0; i < SAMPLE_LEN_MAX; i++) {
        sensor._ir_samples[i] = 0;
        sensor._red_samples[i] = 0;
    }
}

void max30102_reset(void)
{
	unsigned char val = 0x40;
    max30102_write(MODE_CONFIG, &val, 1);

}

void max30102_clear_fifo(void)
{
	unsigned char val = 0x00;
	max30102_write(FIFO_WR_PTR, &val, 3);
  max30102_write(FIFO_RD_PTR, &val, 3);
  max30102_write(OVF_COUNTER, &val, 3);
}

void max30102_fifo_config(unsigned char smp_ave, unsigned char roll_over_en, unsigned char fifo_a_full)
{
	unsigned char config = 0x00;
  config |= smp_ave << FIFO_CONFIG_SMP_AVE;
  config |= ((roll_over_en & 0x01) << FIFO_CONFIG_ROLL_OVER_EN);
  config |= ((fifo_a_full & 0x0f) << FIFO_CONFIG_FIFO_A_FULL);
	
	max30102_write(FIFO_CONFIG, &config, 1);
}

void max30102_write(unsigned char reg, unsigned char *buf, unsigned char buflen)
{
    unsigned char i;

    I2C_start();
    I2C_write((unsigned char)(MAX30102_I2C_ADDR << 1));  
    I2C_write(reg);                     

    for (i = 0; i < buflen; i++)
    {
        I2C_write(buf[i]);              
    }

    I2C_stop();
}

void max30102_read(unsigned char reg, unsigned char *buf, unsigned char buflen)
{
    unsigned char i;

    // Set register address to read from
    I2C_start();
    I2C_write((unsigned char)(MAX30102_I2C_ADDR << 1));  // write mode (address + write bit)
    I2C_write(reg);                     
    I2C_stop();

    // Read data bytes
    I2C_start();
    I2C_write((unsigned char)((MAX30102_I2C_ADDR << 1) | 0x01)); // read mode (address + read bit)

    for (i = 0; i < buflen; i++)
    {
        buf[i] = I2C_read();          // read one byte

        // Send ACK after every byte except the last
        if (i < (buflen - 1))
        {
            // Send ACK 0 by pulling SDA low during ACK clock pulse
            SDA = 0;
        }
        else
        {
            // Send NACK 1 on last byte
            SDA = 1;
        }
        
        SCL = 1;
        I2C_delay();
        SCL = 0;
        I2C_delay();
    }

    I2C_stop();
}

void max30102_set_led_pulse_width(max30102_led_pw_t pw)
{
    unsigned char config;

    max30102_read(SPO2_CONFIG, &config, 1);
    config = (config & 0x7C) | (pw << SPO2_LED_PW);
    max30102_write(SPO2_CONFIG, &config, 1);
}

void max30102_set_adc_resolution(max30102_adc_t adc)
{
	unsigned char config;
	
	max30102_read(SPO2_CONFIG, &config, 1);
	config = (config & 0x1f) | (adc << SPO2_ADC_RGE);
	max30102_write(SPO2_CONFIG, &config, 1);
}

void max30102_set_sampling_rate(max30102_sr_t sr)
{
	unsigned char config;
	
	max30102_read(SPO2_CONFIG, &config, 1);
	config = (config & 0x63) | (sr << SPO2_SR);
	max30102_write(SPO2_CONFIG, &config, 1);
}

void max30102_set_led_current_1(float ma)
{
    unsigned char pa = ma / 0.2;
    max30102_write(LED_IR_PA1, &pa, 1);
}

void max30102_set_led_current_2(float ma)
{
    unsigned char pa = ma / 0.2;
    max30102_write(LED_RED_PA2, &pa, 1);
}

void max30102_set_mode(max30102_mode_t mode)
{
    unsigned char config;
    max30102_read(MODE_CONFIG, &config, 1);
    config = (config & 0xf8) | mode;
    max30102_write(MODE_CONFIG, &config, 1);
    max30102_clear_fifo();
}

void max30102_set_a_full(unsigned char enable)
{
    unsigned char reg = 0;
    max30102_read(INTERRUPT_ENABLE_1, &reg, 1);
    reg &= ~(0x01 << INTERRUPT_A_FULL);
    reg |= ((enable & 0x01) << INTERRUPT_A_FULL);
    max30102_write(INTERRUPT_ENABLE_1, &reg, 1);
}

void max30102_set_die_temp_rdy(unsigned char enable)
{
    unsigned char reg = (enable & 0x01) << INTERRUPT_DIE_TEMP_RDY;
    max30102_write(INTERRUPT_ENABLE_2, &reg, 1);
}

void max30102_set_die_temp_en(unsigned char enable)
{
    unsigned char reg = (enable & 0x01) << DIE_TEMP_EN;
    max30102_write(DIE_TEMP_CONFIG, &reg, 1);
}

unsigned char max30102_has_interrupt(void)
{
    return (MAX30102_INT == 0) ? 1 : 0;
}

// Simple peak detection for heart rate 
float compute_heart_rate(uint32_t *ir_buf, int len) {
    int i, peaks = 0;
		float duration;
	
    for (i = 1; i < len-1; i++) {
        if (ir_buf[i] > ir_buf[i-1] && ir_buf[i] > ir_buf[i+1] && ir_buf[i] > 50000) {
            peaks++;
        }
    }
    
    duration = (float)len / 100.0f;
    return ((float)peaks / duration) * 60.0f;
}

float compute_spo2(uint32_t *ir_buf, uint32_t *red_buf, int len) {
    uint32_t ir_ac = 0, ir_dc = 0;
    uint32_t red_ac = 0, red_dc = 0;
    int i;
		float ratio;
    // Compute DC component as average, AC as peak-to-peak
    uint32_t ir_min = ir_buf[0], ir_max = ir_buf[0];
    uint32_t red_min = red_buf[0], red_max = red_buf[0];
    for (i = 0; i < len; i++) {
        ir_dc += ir_buf[i];
        red_dc += red_buf[i];
        if (ir_buf[i] < ir_min) ir_min = ir_buf[i];
        if (ir_buf[i] > ir_max) ir_max = ir_buf[i];
        if (red_buf[i] < red_min) red_min = red_buf[i];
        if (red_buf[i] > red_max) red_max = red_buf[i];
    }
    ir_dc /= len;
    red_dc /= len;
    ir_ac = ir_max - ir_min;
    red_ac = red_max - red_min;
    ratio = (float)(red_ac * ir_dc) / (ir_ac * red_dc);
    
    return 110.0f - 25.0f * ratio;
}

void max30102_read_fifo(void)
{
    unsigned char wr_ptr = 0;
    unsigned char rd_ptr = 0;
    unsigned char num_samples;
    unsigned char sample[6];
    unsigned char i;
		float hr;
		float spo2;

    // Read write and read pointers
    max30102_read(FIFO_WR_PTR, &wr_ptr, 1);
    max30102_read(FIFO_RD_PTR, &rd_ptr, 1);

    //  Calculate number of samples
    num_samples = (unsigned char)wr_ptr - (unsigned char)rd_ptr;
    if (num_samples < 1)
        num_samples += 32;

    if (num_samples > SAMPLE_LEN_MAX)
        num_samples = SAMPLE_LEN_MAX;

    //  Read and store samples
    for (i = 0; i < num_samples; i++)
    {
        max30102_read(FIFO_DATA, sample, 6);

        sensor._ir_samples[i] = (((unsigned long)sample[0] << 16) |
                                   ((unsigned long)sample[1] << 8) |
                                   ((unsigned long)sample[2])) & 0x3FFFF;

        sensor._red_samples[i] = (((uint32_t)sample[3] << 16) |
                                    ((uint32_t)sample[4] << 8) |
                                    ((uint32_t)sample[5])) & 0x3FFFF;
    }
		// Compute and print
    hr = compute_heart_rate(sensor._ir_samples, num_samples);
    spo2 = compute_spo2(sensor._ir_samples, sensor._red_samples, num_samples);
    printf("Heart Rate: %.1f bpm, SpO2: %.1f%%\r\n", hr, spo2);
}

void max30102_read_temp(unsigned char *temp_int, unsigned char *temp_frac)
{
    max30102_read(DIE_TINT, (unsigned char *)temp_int, 1);
    max30102_read(DIE_TFRAC, temp_frac, 1);
}

void max30102_interrupt_handler(void)
{
    unsigned char reg[2] = {0x00};
    // Interrupt flag in registers 0x00 and 0x01 are cleared on read
    max30102_read(INTERRUPT_STATUS_1, reg, 2);

    if ((reg[0] >> INTERRUPT_A_FULL) & 0x01)
    {
        // FIFO almost full
        max30102_read_fifo();
    }

    if ((reg[0] >> INTERRUPT_PPG_RDY) & 0x01)
    {
        // New FIFO data ready
    }

    if ((reg[0] >> INTERRUPT_ALC_OVF) & 0x01)
    {
        // Ambient light overflow
    }

    if ((reg[1] >> INTERRUPT_DIE_TEMP_RDY) & 0x01)
    {
        // Temperature data ready
        unsigned char temp_int;
        unsigned char temp_frac;
        max30102_read_temp(&temp_int, &temp_frac);
        float temp = temp_int + 0.0625f * temp_frac;
        printf("Temperature: %.2f C\r\n", temp);
    }

    // Reset interrupt flag
    sensor._interrupt_flag = 0;
}

void main(void)
{
	unsigned char en_reg[2] = {0};
	//Initializations
	UART_Init();
	I2C_init();
	GPIO_Init();
	max30102_init();
	
	// Initial configurations
	max30102_reset();
	max30102_clear_fifo();
	max30102_fifo_config(SMP_AVE_8, 1, 7);
	
	// Sensor settings
    max30102_set_led_pulse_width(max30102_pw_16_bit);
    max30102_set_adc_resolution(max30102_adc_2048);
    max30102_set_sampling_rate(max30102_sr_800);
    max30102_set_led_current_1(6.2);
    max30102_set_led_current_2(6.2);

    // Enter SpO2 mode
    max30102_set_mode(max30102_multi_led);
    max30102_set_a_full(1);
  
    // Initiate 1 temperature measurement
    max30102_set_die_temp_en(1);
    max30102_set_die_temp_rdy(1);
	
    max30102_read(0x00, en_reg, 1);
	
    while (1)
    {
        if (max30102_has_interrupt())
        {
            max30102_interrupt_handler();
        }
    }
}