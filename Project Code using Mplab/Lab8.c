#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include <p18f4620.h>
#include <usart.h>

#pragma config OSC      =   INTIO67
#pragma config BOREN    =   OFF
#pragma config WDT      =   OFF
#pragma config LVP      =   OFF
#pragma config CCP2MX   =   PORTBE

#define PULSE       PORTEbits.RE0
#define TFT_RST     PORTBbits.RB4         // Location of TFT Reset
#define TFT_CS      PORTBbits.RB2         // Location of TFT Chip Select
#define TFT_DC      PORTBbits.RB5         // Location of TFT D/C

#define TS_1                 1            // Size of Normal Text
#define TS_2                 2            // Size of Number Text
 
#define title_txt_X          2            // X-location of Title Text
#define title_txt_Y          2            // X-location of Title Text
 
#define voltage_txt_X       25            // X-location of Voltage Text
#define voltage_txt_Y       25            // Y-location of Voltage Text
#define voltage_X           40            // X-location of Voltage Number
#define voltage_Y           37            // Y-location of Voltage Number
#define voltage_Color       ST7735_BLUE   // Color of Voltage data
 
#define dc_txt_X            37
#define dc_txt_Y            60
#define dc_X                52
#define dc_Y                72
#define dc_Color            ST7735_MAGENTA
 
#define RPS_txt_X           20
#define RPS_txt_Y           95
#define RPS_X               20
#define RPS_Y               107
#define RPS_Color           ST7735_CYAN
 
#define HZ_txt_X            90
#define HZ_txt_Y            95
#define HZ_X                75
#define HZ_Y                107
#define HZ_Color            ST7735_CYAN
 
#define RPM_txt_X           37
#define RPM_txt_Y           130
#define RPM_X               20
#define RPM_Y               142
#define RPM_Color           ST7735_WHITE
                   //
#define _XTAL_FREQ          8000000           // Set operation for 8 Mhz        //
#define TMR_CLOCK           _XTAL_FREQ/4      // Timer Clock 2 Mhz              //
#define COUNT_PER_MS        TMR_CLOCK/1000    // Count per ms = 2000            //
#define COUNT_SCALED        COUNT_PER_MS/32   //                                //                    //                    //

void delay_500ms(void);                                                         // Prototyping our subroutines
void init_ADC(void);  
void do_update_pwm(char);  
unsigned int get_full_ADC(void);
float Read_Volt_In(void);
int get_RPS(void);  
void putch (char);  
void init_UART(void);  
void Initialize_Screen(void);  
void delay_ms(int);  
void DO_INIT(void);
void init_IO(void);

#include "ST7735_TFT.inc"                 // Important file that contains important definitions for the Initialize screen subroutines
                                          // and define statements
#define _XTAL_FREQ      8000000           // Set operation for 8 Mhz
#define TMR_CLOCK       _XTAL_FREQ/4      // Timer Clock 2 Mhz
#define COUNT_PER_MS    TMR_CLOCK/1000    // Count per ms = 2000
#define COUNT_SCALED    COUNT_PER_MS/32   //

char *txt;                                // pointer to txt that is used in Initialize Screen subroutine
char buffer[30] = "";
char voltage_text[] = "0.0v";               // different variables relating to text values
char dc_text[] = "00%";                   // that will track the numerical value
char RPS_text[] = "00";                   // depending on the name, ex: dc_text will track value of dc
char HZ_text[] = "000";                   // RPS_text tracks the rps of the fan, etc
char RPM_text[] = "0000 RPM";

void init_ADC()  
{
    ADCON0=0x05; // select channel AN0, and turn on the ADC subsystem
    ADCON1=0x1B ; // select pins AN0 through AN3 as analog signal, VDD-VSS as reference voltage
    ADCON2=0xA9; // right justify the result. Set the bit conversion time (TAD) and acquisition time

}

void delay_500ms(void)
{
 T0CON = 0x04; // Timer 0, 16-bit mode, pre scaler 1:32
 TMR0L = 0xED; // set the lower byte of TMR
 TMR0H = 0x88; // set the upper byte of TMR
 INTCONbits.TMR0IF = 0; // clear the Timer 0 flag
 T0CONbits.TMR0ON = 1; // Turn on the Timer 0
 while (INTCONbits.TMR0IF == 0); // wait for the Timer Flag to be 1 for done
 T0CONbits.TMR0ON = 0; // turn off the Timer 0
}

void main(void)
{
    init_UART();                           // Subroutine used for tera-term purposes
    init_ADC();                            // Initializes our ports and ADCONs
    OSCCON = 0x70;                         // set the CPU speed to be at 8Mhz  
    txt = buffer;
    init_IO();
    Initialize_Screen();                   // subroutine that displays our text on the screen
    DO_INIT(); // initialize the I/O (make sure to setup the direction
     // of the I/O especially the signal ?PULSE?
       
    OSCCON = 0x70; // set the CPU speed to be at 8Mhz
    PULSE = 0; // set the PULSE to be 0 first
    nRBPU = 0;                                // Enable PORTB internal pull up resistor
  TRISA = 0xFF;                             // Set the PORT directions
  TRISB = 0x03;                             //
  TRISC = 0x01;                             //
  TRISD = 0x00;                             //
  TRISE = 0x00;                             //
  PORTD = 0x00;                             //
   
      while (1)    
      {  
         
         
         
         float input_voltage = Read_Volt_In();          // read the input voltage  
         char dc = (input_voltage/4)*100;                                  // calculate the percentage of the ratio        
         do_update_pwm(dc);                              // call routine to generate the PWM pulse  
         char RPS = get_RPS();                           // measure RPS    
         int HZ = RPS * 2;                               // calculate HZ equivalent  
         int RPM = RPS * 60;                             // calculate RPM equivalent
         
         char iv1 = (int) input_voltage;                 // defining how the value and placement order is calculated
         char iv2 = (int) ((input_voltage - iv1) * 10);  // i.e. "tens, hundreds, tenths place
         voltage_text[0] = iv1 + '0';                    // assign the char to text variable
         voltage_text[2] = iv2 + '0';    
         drawtext(voltage_X, voltage_Y, voltage_text, voltage_Color, ST7735_BLACK, TS_2);
         
         //float duty_cycle = ((int));
         //int dcpint = RPS*1;
         
         
         char dcchar1 = ((int)dc)/10;
         char dcchar2 = ((int)dc)%10;
         dc_text[0] = dcchar1 + '0';
         dc_text[1] = dcchar2 + '0';
         drawtext(dc_X , dc_Y , dc_text, dc_Color, ST7735_BLACK, TS_2);
         
   
         char rps_first = ((int)RPS)/10;
         char rps_second = ((int)RPS)%10;
         RPS_text[0] = rps_first + '0';
         RPS_text[1] = rps_second + '0';
         drawtext(RPS_X, RPS_Y , RPS_text, RPS_Color, ST7735_BLACK, TS_2);
         
       
         char Hertz1 = HZ/100;
         char Hertz2 = (HZ/10)%10;
         char Hertz3 = HZ%10;
         HZ_text[0] = Hertz1 + '0';
         HZ_text[1] = Hertz2 + '0';
         HZ_text[2] = Hertz3 + '0';
         drawtext(HZ_X , HZ_Y  , HZ_text, HZ_Color, ST7735_BLACK, TS_2);
         
       
         char RPM1 = RPM/1000;
         char RPM2 = (RPM/100)%10;
         char RPM3 = (RPM/10)%10;
         char RPM4 = RPM%10;
         RPM_text[0] = RPM1 + '0';
         RPM_text[1] = RPM2 + '0';
         RPM_text[2] = RPM3 + '0';
         RPM_text[3] = RPM4 + '0';
         drawtext(RPM_X, RPM_Y , RPM_text, RPM_Color , ST7735_BLACK, TS_2);
                                       
      }  

   
}

void Initialize_Screen(void)
{
    LCD_Reset();                                                                      // Screen reset
    TFT_GreenTab_Initialize();    
   
    fillScreen(ST7735_BLACK);                                                         // Fills background of screen with color passed to it
 
     strcpy(txt, " ECE3301L Fall 2019\0");                                            // Text displayed
     drawtext(title_txt_X, title_txt_Y, txt, ST7735_WHITE, ST7735_BLACK, TS_1);       // X and Y coordinates of where the text is to be displayed
                                                                                   // including text color and the background of it
     strcpy(txt, "Input Voltage:");                                                   // Input Voltage is written on the screen
     drawtext(voltage_txt_X, voltage_txt_Y, txt, voltage_Color, ST7735_BLACK, TS_1);  //Text coordinates and their color is drawn
   
     //->   * duty cycle    
     strcpy(txt, "Duty Cycle");                                                       // Duty cycle text displayed
     drawtext(dc_txt_X, dc_txt_Y, txt, dc_Color, ST7735_BLACK, TS_1);                 // X and Y coordinates and corresponding color drawn
   
      //->   * RPS
     strcpy(txt, "RPS");
     drawtext(RPS_txt_X, RPS_txt_Y , txt, RPS_Color, ST7735_BLACK, TS_1);             // For the rest of the code in Initialize screen function
 
     
     strcpy(txt, "HZ");                                                               // and gives us the option to change the color we want the text
     drawtext(HZ_txt_X, HZ_txt_Y , txt, HZ_Color, ST7735_BLACK, TS_1);                // to be displayed in
   
     //->    * RPM
     strcpy(txt, "FAN Speed");
     drawtext(RPM_txt_X, RPM_txt_Y  , txt, RPM_Color , ST7735_BLACK, TS_1);
     
}

int get_RPS(void)
{
 TMR1L = 0; // clear TMR1L to clear the pulse counter
 T1CON = 0x03; // enable the hardware counter
 PULSE = 1; // turn on the PULSE signal
 delay_500ms (); // delay 500 msec
 PULSE = 0; // turn off the PULSE signal
 char RPS = TMR1L; // read the number of pulse
 T1CON = 0x02; // disable the hardware counter
 return (RPS); // return the counter
}


void do_update_pwm(char duty_cycle)
{
    float dc_f;  
    int dc_I;    
    PR2 = 0b00000100 ;                                    // set the frequency for 25 Khz  
    T2CON = 0b00000111 ;                                  //    
    dc_f = ( 4.0 * duty_cycle / 20.0) ;                   // calculate factor of duty cycle versus a 25 Khz          
                                                          // signal    
    dc_I = (int) dc_f;                                    // get the integer part  
    if (dc_I > duty_cycle) dc_I++;                        // round up function    
    CCP1CON = ((dc_I & 0x03) << 4) | 0b00001100;    
    CCPR1L = (dc_I) >> 2;
}


float Read_Volt_In(void)
{
    ADCON0=0x05;
    int STEP = get_full_ADC();                            //assigns the analog digital converter subroutine to our variabel STE{
    float VOLT = (STEP*4.0)/1000.0;
    return VOLT;
}

unsigned int get_full_ADC(void)
{
int result;
ADCON0bits.GO=1;                        // Start Conversion
while(ADCON0bits.DONE==1);                         // wait for conversion to be completed (DONE=0)
result =((ADRESH*0x100)+ADRESL);                   // combine result of upper byte and lower byte into result
return result;                         // return the result.
}

void putch (char c)                                       // Used for tera term debugging purposes
{  
    while (!TRMT);      
    TXREG = c;
}

void init_UART()                                          // Used for tera term debugging purposes
{
    OpenUSART (USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 25);
    OSCCON = 0x60;
}

void delay_ms(int ms)
{
    int count;
    count = (0xffff - COUNT_SCALED) - 1;
    count = count * ms;
 
T0CON = 0x04;                                                                      // Timer 0, 16-bit mode, pre scaler 1:32
TMR0H = count >> 8;                                                                // set the upper byte of TMR
TMR0L = count & 0x00ff;                                                            // set the lower byte of TMR

INTCONbits.TMR0IF = 0;                                                             // clear the Timer 0 flag
T0CONbits.TMR0ON = 1;                                                              // Turn on the Timer 0

while (INTCONbits.TMR0IF == 0);                                                 // wait for the Timer Flag to be 1 for done
T0CONbits.TMR0ON = 0;                                                              // turn off the Timer 0
}


void DO_INIT()
{
init_UART();    
TRISA = 0xFF;               // setup Port A as input
TRISB = 0x00;               // setup Port B as output
TRISC = 0x00;               // setup Port C as output
TRISD = 0x00;               // setup Port D as output
TRISE = 0x00;               // setup Port E as output
init_ADC();
}

void init_IO()
{
TRISA = 0xFF;               // setup Port A as input
TRISB = 0x00;               // setup Port B as output
TRISC = 0x01;               // setup Port C as output
TRISD = 0x00;               // setup Port D as output
TRISE = 0x00;
}
