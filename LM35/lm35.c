#include <REG51.h>
#include <stdio.h>

sbit read_adc = P2^0;
sbit wrt_adc  = P2^1;
sbit intr_adc = P3^2;

unsigned char adc_value;
float temperature;
char buffer[30]; 

void delay_ms(unsigned int ms);
void ADC_conversation();
void UART_Init(void);
void Transmit_data(char tx_data);
void String(char *str);

void delay_ms(unsigned int ms)
{
	int i, j;
	for(i=0; i<ms; i++)
		for(j=0; j<200; j++);
}

void ADC_conversation()
{
	float voltage = (adc_value / 255.0) * 5.0;  // Convert ADC value to voltage
  temperature = voltage * 100.0;   
}

void UART_Init(void)
{
    TMOD = 0x20;    // Timer1 Mode2 (8-bit auto-reload) for baud rate generation
    TH1 = 0xFD;     // 9600 baud rate for 11.0592MHz clock
    SCON = 0x50;    // 8-bit data, 1 stop bit, REN enabled
    TR1 = 1;        // Start Timer1
}

void Transmit_data(char tx_data)
{
	SBUF = tx_data;		/* Load char in SBUF register */
	while (TI==0);		/* Wait until stop bit transmit */
	TI = 0;			/* Clear TI flag */
}

void String(char *str)
{
	int i;
	for(i=0;str[i]!=0;i++)	/* Send each char of string till the NULL */
	{
		Transmit_data(str[i]);	/* Call transmit data function */
	}
}

void main()
{
	P1=0xFF;
	read_adc = 1;
	wrt_adc = 1;
	intr_adc = 1;
	
	UART_Init();
	
	while(1)
	{
		wrt_adc = 0; 
		delay_ms(1);
		wrt_adc = 1;
		
		while(intr_adc == 1); 
		
		read_adc = 0; 
		adc_value = P1; 
		read_adc = 1;
		
		ADC_conversation();
		
		sprintf(buffer, "Temperature = %.2f C\r\n", temperature);  // format temperature into buffer
		String(buffer); // send formatted string
		
    delay_ms(500);
	}
}

