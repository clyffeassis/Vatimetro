#include"voltimeter.h"              // Cabeçalho .h de configuração fuses
#include <pic18f4550.h>             // Cabeçalho padrão pic18f4550
#include <stdio.h>                  // Standard input/output cabeçalho
#include <stdlib.h>                 // biblioteca Standard IO
#include <math.h>

/*********************Definiçao de Portas********************************/

#define RS LATB0  /*PIN 0 do PORTB é atribuido ao "Register Select" PIN do LCD*/
#define EN LATB1  /*PIN 1 do PORTB é atribuido ao "Enable" PIN do LCD*/
#define ldata LATB  /*PORTB(PB4-PB7) é atribuido ao "DATA" do LCD*/ 
#define LCD_Port TRISB  /*Define macros para o PORTB Direction Register*/
#define PAR_ESCALE_CURRENT 0.2 /* Fator de escala corrente/tensao amperimetro */
#define PAR_FILTER_CURRENT 0.5 /* Fator de atualização do valor da corrente */
/*********************Prototipo de Funçoes*****************************/

void MSdelay(unsigned int );                /*Gera delay em ms*/
void LCD_Init();                            /*Inicializa LCD*/
void LCD_Command(unsigned char );           /*Envia comandos para o LCD*/
void LCD_Char(unsigned char x);             /*Envia dados para o LCD*/
void LCD_String(const char *);              /*Imprime String no LCD*/
void LCD_String_xy(char,char,const char *); /*Imprime String no LCD na posição desejada*/
void LCD_Clear();                           /*Limpa o LCD*/
void LCD_Shift_Right();                     /**/
void TIMER1_Start();                        /*Inicializa configuraçoes do timer1*/
void ADC_Init();                            /*Inicializa configuraçoes do ADC*/
int  ADC_Read(int);                         /*Lê valores do ADC*/

/*******************Declaraçao de Variáveis******************************/

int current_read;
int voltage_read;
int max_current = 0;
int max_current_ant = 0;
float current = 0; 
float voltage = 0;
char current_str[10];
char voltage_str[10];
unsigned int counter = 0x00;


int main(void)
{    
    OSCCON = 0x72;          /*Oscilador interno e freq. de 8 MHz*/ 
    TRISC = 0x00;           /*Porta C configurada como saída*/
	LCD_Init();             /*Inicializa LCD*/    
    TIMER1_Start();         /*Inicializa conf. TIMER1*/
    ADC_Init();             /*Inicializa conf. ADC */
    LCD_Clear();            /*Limpa LCD*/
	while(1)                /*Loop Principal*/
    {
        
        //MSdelay(500);
        //PORTCbits.RC0 = ~PORTCbits.RC0;
        //MSdelay(500);
        current_read = ADC_Read(0);
        
        if (current_read > max_current)
        {
            max_current = (max_current + current_read) / 2;
        }
        if (counter >= 500)
        {
            max_current = (PAR_FILTER_CURRENT * max_current_ant) + (1 - PAR_FILTER_CURRENT)*max_current;
            current = max_current * 5. / 1023. - 2.5;
            current =  current / ( PAR_ESCALE_CURRENT * sqrt(2));
            if (current >= 10 || (current <0 && current >-10))
            {
                sprintf(current_str, "I = %0.3f  A", current);
            }
            if (current <= -10)
            {
                sprintf(current_str, "I = %0.3f A", current);
            }
            if (current >= 0 && current <10)
            {
                sprintf(current_str, "I = %0.3f   A", current);
            }
            LCD_String_xy(2,1, current_str);
            counter = 0;
            max_current_ant = max_current;
            max_current = 0;
        }
        
        
//        //voltage_read = ADC_Read(1);
//        //voltage = voltage_read * 5. / 1023. ;
//        current = current_read * 5. / 1023. - 2.5;
//        //sprintf(voltage_str, "V = %0.3f  V", voltage);
//        //LCD_String_xy(1,1, voltage_str);
//        current = current / PAR_ESCALE_CURRENT;
//        if (current >= 10 || current <0)
//        {
//            sprintf(current_str, "I = %0.3f A", current);
//        }
//        else
//        {
//            sprintf(current_str, "I = %0.3f  A", current);
//        }
//        LCD_String_xy(2,1, current_str); 
   }   // fim loop principal
}   // fim main

/****************************Funções********************************/

void LCD_Init()
{
    LCD_Port = 0;       /*PORT as Output Port*/
    MSdelay(15);        /*15ms,16x2 LCD Power on delay*/
    LCD_Command(0x02);  /*send for initialization of LCD 
                          for nibble (4-bit) mode */
    LCD_Command(0x28);  /*use 2 line and 
                          initialize 5*8 matrix in (4-bit mode)*/
	LCD_Command(0x01);  /*clear display screen*/
    LCD_Command(0x0c);  /*display on cursor off*/
	LCD_Command(0x06);  /*increment cursor (shift cursor to right)*/	   
}

void LCD_Command(unsigned char cmd )
{
	ldata = (ldata & 0x0f) |(0xF0 & cmd);  /*Send higher nibble of command first to PORT*/ 
	RS = 0;  /*Command Register is selected i.e.RS=0*/ 
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/ 
	NOP();
	EN = 0;
	MSdelay(1);
    ldata = (ldata & 0x0f) | (cmd<<4);  /*Send lower nibble of command to PORT */
	EN = 1;
	NOP();
	EN = 0;
	MSdelay(3);
}


void LCD_Char(unsigned char dat)
{
	ldata = (ldata & 0x0f) | (0xF0 & dat);  /*Send higher nibble of data first to PORT*/
	RS = 1;  /*Data Register is selected*/
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/
	NOP();
	EN = 0;
	MSdelay(1);
    ldata = (ldata & 0x0f) | (dat<<4);  /*Send lower nibble of data to PORT*/
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/
	NOP();
	EN = 0;
	MSdelay(3);
}
void LCD_String(const char *msg)
{
	while((*msg)!=0)
	{		
	  LCD_Char(*msg);
	  msg++;	
    }
}

void LCD_String_xy(char row,char pos,const char *msg)
{
    char location=0;
    if(row<=1)
    {
        location=(0x80) | ((pos) & 0x0f);  /*Print message on 1st row and desired location*/
        LCD_Command(location);
    }
    else
    {
        location=(0xC0) | ((pos) & 0x0f);  /*Print message on 2nd row and desired location*/
        LCD_Command(location);    
    }  
    

    LCD_String(msg);

}
void LCD_Clear()
{
   	LCD_Command(0x01);  /*clear display screen*/
    MSdelay(3);
}

void LCD_Shift_Right()
{
   	LCD_Command(0x1c);  /*clear display screen*/
    MSdelay(3);
}

void MSdelay(unsigned int val)
{
 unsigned int i,j;
 for(i=0;i<val;i++)
     for(j=0;j<165;j++);  /*This count Provide delay of 1 ms for 8MHz Frequency */
 }


void __interrupt() timer1_isr(void)
{
    counter++;
    TMR1=0xF856;
    
    PIR1bits.TMR1IF=0;  /* Make Timer1 Overflow Flag to '0' */
}

void TIMER1_Start()
{
    GIE=1;		/* Enable Global Interrupt */
    PEIE=1;  		/* Enable Peripheral Interrupt */
    TMR1IE=1;		/* Enable Timer1 Overflow Interrupt */
    TMR1IF=0;
    
    /* Enable 16-bit TMR1 register,no pre-scale,internal clock, timer OFF */
    T1CON=0x80;		

    TMR1=0xF856;	/* Load Count for generating delay of 1ms */
    TMR1ON=1;		/* Turn ON Timer1 */
}

void ADC_Init()
{    
    TRISA = 0xff;		/*Set as input port*/
    //ADCON1 = 0x0e;  		/*Ref vtg is VDD & Configure pin as analog pin*/  0000 1110      0011 1011  
    ADCON1 = 0x0d;          /* VSS = Vref-, VDD = Vref+ & utilizando AN0 e AN1        0000 1101 */
    ADCON2 = 0x92;  		/*Right Justified, 4Tad and Fosc/32. */
    ADRESH=0;  			/*Flush ADC output Register*/
    ADRESL=0;   
}

int ADC_Read(int channel)
{
    int digital;
    ADCON0 =(ADCON0 & 0b11000011)|((channel<<2) & 0b00111100);

    /*channel 0 is selected i.e.(CHS3CHS2CHS1CHS0=0000)& ADC is disabled*/
    ADCON0 |= ((1<<ADON)|(1<<GO));/*Enable ADC and start conversion*/

    /*wait for End of conversion i.e. Go/done'=0 conversion completed*/
    while(ADCON0bits.GO_nDONE==1);

    digital = (ADRESH*256) | (ADRESL);/*Combine 8-bit LSB and 2-bit MSB*/
    return(digital);
}

