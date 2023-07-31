/*
 * File:   DigitalClock.c
 * Author: masah
 *
 * Created on July 27, 2023, 2:49 PM
 */



#include <xc.h>
#include <stdio.h>
#include <string.h>
#include "Lcd.h"
#include "Serial.h"


#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2Lv
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#define STARTVALUE  3036


// Global variables to store time and alarm state
unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;
unsigned char alarm_hours = 0;
unsigned char alarm_minutes = 0;
unsigned char alarm_seconds = 0;

volatile unsigned char alarm_enabled = 0; //disabled initially

// Function prototypes
void update_lcd_display(void);
void handle_serial_commands(void);

void setupPorts(void);
void reloadTimer0( void );

void initTimers01(void) ;
void initialization(void) ;
void updateClockDisplay(void) ;
void updateAlarmStateDisplay(void) ;
void updateAlarmDisplay(void) ;
void __interrupt(high_priority) highIsr(void);
void __interrupt(low_priority) lowIsr(void);
void toggle_alarm(void);
void DisplayNames(void);
void clearBuffer(void);
int isDigit(char);


//for main cases
unsigned char start_recieved = 0; 
unsigned char read_time_flag = 0;
unsigned char read_flag = 0;
unsigned char read_alarm_flag = 0; 
unsigned char capital_w_flag = 0; 
unsigned char small_w_flag = 0; 
unsigned char first_digit = 0;
unsigned char h_flag = 0;
unsigned char m_flag = 0;
unsigned char s_flag = 0;
unsigned char a_flag = 0;
volatile unsigned char toggle_alarm_flag = 0;
void disableInterrupts(void) {
    INTCONbits.GIEH = 0;
    INTCONbits.GIEL = 0;
}

void enableInterrupts(void) {
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
}

void main(void) {
setupSerial();
    initTimers01();
    setupPorts();
    lcd_init();
    initialization();
     DisplayNames();
   send_string_no_lib("Hello\n");
    while (1) {
        CLRWDT();
        updateAlarmDisplay();
       updateAlarmStateDisplay();
       updateClockDisplay();
      
      
       unsigned char c=read_byte_no_lib();

       
            if (c == '[') {
               start_recieved=1;
                
            }
       //[rt]
            else if ((c=='r')&& (start_recieved==1)){
               start_recieved=2;
               read_flag=1;
            }
             else if ((c=='t')&& (start_recieved==2)){
               start_recieved=3;
               read_time_flag=1;
            }
             
        //[ra]
          else if ((c=='a')&& (start_recieved==2)&&read_flag==1){
               start_recieved=3;
               read_flag=0;
               read_alarm_flag=1;
            }
            else  if ((c==']')&& (start_recieved==3)){ // if start_recieved=3 means its read alarm or time
               start_recieved=0;
               if(read_time_flag==1){
               char time_response[9];
               sprintf(time_response, "%02u:%02u:%02u", hours, minutes, seconds);
               send_string_no_lib(time_response);
               read_time_flag=0;
               }
               else if(read_alarm_flag==1){
                char alarm_response[13];
                sprintf(alarm_response, "%02u:%02u:%02u %s", alarm_hours, alarm_minutes, alarm_seconds,alarm_enabled?"ON":"OFF");
                send_string_no_lib(alarm_response);
                read_alarm_flag=0;
               }
               
                 
            }
       /********************************************************************/
         /********************************************************************/
            else if((c=='w') &&(start_recieved==1)){
                start_recieved=2;
                small_w_flag=1;
             }
            else if((c=='W') &&(start_recieved==1)){
                start_recieved=2;
                capital_w_flag=1;
             }
            else if((c=='h') &&(start_recieved==2)){
                start_recieved=3;
                h_flag=1;
             }
             else if((c=='m') &&(start_recieved==2)){
                start_recieved=3;
                m_flag=1;
             }
              else if((c=='s') &&(start_recieved==2)){
                start_recieved=3;
                s_flag=1;
             }
              else if((c=='a') &&(start_recieved==2)){
                start_recieved=3;
                a_flag=1;
             }
       
            else if(isDigit(c)&&start_recieved==3){
                first_digit=c;
                start_recieved=4;
            }
       
       
             else if(isDigit(c)&&start_recieved==4){
                start_recieved=5;
                if(small_w_flag==1 &&h_flag==1){
                    small_w_flag=0;
                    h_flag=0;
                    if(first_digit<='1'&&c<='2')
                    hours = (first_digit - '0') * 10 + (c - '0');
                }
                else if(small_w_flag==1 &&m_flag==1){
                    small_w_flag=0;
                    m_flag=0;
                    if(first_digit<='5'&&c<='9')
                    minutes = (first_digit - '0') * 10 + (c - '0');
                }
                else if(small_w_flag==1 &&s_flag==1){
                    small_w_flag=0;
                    s_flag=0;
                    if(first_digit<='5'&&c<='9')
                    seconds = (first_digit - '0') * 10 + (c - '0');
                }
                else if(capital_w_flag==1 &&h_flag==1){
                    capital_w_flag=0;
                    h_flag=0;
                    if(first_digit<='1'&&c<='2')
                    alarm_hours = (first_digit - '0') * 10 + (c - '0');
                }
                else if(capital_w_flag==1 &&m_flag==1){
                    capital_w_flag=0;
                    m_flag=0;
                    if(first_digit<='5'&&c<='9')
                    alarm_minutes = (first_digit - '0') * 10 + (c - '0');
                }
                else if(capital_w_flag==1 && s_flag==1){
                    capital_w_flag=0;
                    s_flag=0;
                    if(first_digit<='5'&&c<='9')
                    alarm_seconds = (first_digit - '0') * 10 + (c - '0');
                }
                else if(capital_w_flag==1 && a_flag==1){
                    capital_w_flag=0;
                    a_flag=0;
                    if(first_digit=='0'&& c=='0'){
                         
                        if(alarm_enabled==1){
                            // toggle_alarm_flag = 1;
                            disableInterrupts(); // Disable interrupts before modifying alarm_enabled
                            alarm_enabled = !alarm_enabled;
                            toggle_alarm_flag = 0; // Reset the flag
                            enableInterrupts(); 
                        }
                         
                    }
                   
                    else { 
                          if(alarm_enabled==0){
                            // toggle_alarm_flag = 1;
                              disableInterrupts(); // Disable interrupts before modifying alarm_enabled
                            alarm_enabled = !alarm_enabled;
                            toggle_alarm_flag = 0; // Reset the flag
                            enableInterrupts(); 
                        }
                     
                    }
//                      if (toggle_alarm_flag) {
//                            disableInterrupts(); // Disable interrupts before modifying alarm_enabled
//                            alarm_enabled = !alarm_enabled;
//                            toggle_alarm_flag = 0; // Reset the flag
//                            enableInterrupts(); // Enable interrupts after modifying alarm_enabled
//                      }
                }
                
              }
             else  if ((c==']')&& (start_recieved==5)){ // if start_recieved=3 means its read alarm or time
               start_recieved=0;
             }
             
           
       
        
    
    }
 
    
}

void setupPorts(void) {
   
      ADCON0 = 0;
    ADCON1 = 0b00001100; 
    TRISB = 0xFF; 
    TRISC = 0x80; 
    TRISA = 0xFF; 
    TRISD = 0x00;
    TRISE = 0x00;
  
}


void updateAlarmDisplay(void) {
     char alarm_string[17];
         sprintf(alarm_string, "Alarm: %02u:%02u:%02u", alarm_hours, alarm_minutes, alarm_seconds);

     lcd_gotoxy(1, 2);
    lcd_puts(alarm_string);
    
}

void reloadTimer0( void )
{
    
    TMR0H = (unsigned char) ((STARTVALUE >>  8) & 0x00FF);
    TMR0L =  (unsigned char)(STARTVALUE & 0x00FF );
     
}
void initTimers01(void) {
    T0CON = 0;
    
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; 
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
    T1CONbits.TMR1CS = 1; 
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;
 
    INTCONbits.GIE = 1;
    INTCONbits.T0IE = 1;
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

}
void initialization(void) {
    INTCONbits.GIEH   = 1;    // Enable global interrupt bits
    INTCONbits.GIEL   = 1;    // Enable global interrupt bits
    INTCON2 = 0;
    INTCON3 = 0;
    INTCON2bits.INTEDG0 = 1;  // Interrupt 0 on rising edge
    INTCON2bits.INTEDG1 = 1;  // Interrupt 1 on rising edge
    INTCON2bits.INTEDG2 = 1;  // Interrupt 2 on rising edge
    INTCON3bits.INT1IE  = 1;  // Enable external interrupt 1
    INTCON3bits.INT2IE  = 1;  // Enable external interrupt 2
    RCONbits.IPEN = 0;        // Disable Interrupt priority , All are high
    INTCONbits.INT0IE   = 1;  // Enable external interrupt 0
    PORTCbits.RC5 = 0; // Turn Off Heater
    T3CONbits.TMR3ON = 1; 
    T0CON = 0;
    T0CONbits.T0PS0  = 1; // 16 Pre_Scalar
    T0CONbits.T0PS1  = 1;
    T0CONbits.T0PS2  = 0;
    reloadTimer0();
    INTCONbits.TMR0IE = 1; // Enable Timer 0 interrupt
    T0CONbits.TMR0ON  = 1;
    alarm_enabled = 0;
}

void updateClockDisplay(void) {
    char Buffer[32];
    lcd_gotoxy(1, 1);
    sprintf(Buffer, "Time: %02u:%02u:%02u", hours, minutes, seconds);
    lcd_puts(Buffer);
}
void updateAlarmStateDisplay(void){
     lcd_gotoxy(1, 3);
    lcd_puts("Alarm State: ");
    lcd_puts(alarm_enabled?"ON ":"OFF");
    
}
void DisplayNames(void){
      lcd_gotoxy(1, 4);
    lcd_puts("Masa & Salsabeel");
    
}
//void clearBuffer(void){
//    for (int i=0;i<MAX_COMMAND_LENGTH;i++){
//        if (command_buffer[i]){
//            command_buffer[i]='\0';
//        }
//    }
//}
int isDigit(char c){
    if (c>='0'&& c<='9'){
        return 1;
       
    } return 0;
}

void __interrupt(high_priority) highIsr(void) {
    
     if (INTCONbits.INT0IF) {
        INTCONbits.INT0IF = 0; // Clear INT0 interrupt flag
       
        disableInterrupts(); // Disable interrupts before modifying alarm_enabled
            alarm_enabled = !alarm_enabled;
            toggle_alarm_flag = 0; // Reset the flag
            enableInterrupts(); // Enable interrupts after modifying alarm_enabled
    }
     if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0; // Clear Timer 0 interrupt flag

        seconds++;
        if (seconds >= 60) {
            seconds = 0;
            minutes++;
            if (minutes >= 60) {
                minutes = 0;
                hours++;
                if (hours >= 24) {
                    hours = 0;
                }
            }
        }

       
    }
    
}

void __interrupt(low_priority) lowIsr(void) {
  
   
}
