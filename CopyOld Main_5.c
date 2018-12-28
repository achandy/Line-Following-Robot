#include <math.h>
#include <plib.h>
#include <stdio.h>
#include "dsplib_dsp.h"
#include "fftc.h"
#define SYS_FREQ  (80000000L) 
#define MOTOR_UPDATE    50   // Update servo motors 50 times a second 
#define MOTOR_TICK_RATE (SYS_FREQ/256/MOTOR_UPDATE) 
#pragma config ICESEL = ICS_PGx1	//configure on board licensed debugger
#pragma config FNOSC = PRIPLL		//configure system clock 80 MHz
#pragma config POSCMOD = EC, FPLLIDIV = DIV_2,FPLLMUL = MUL_20,FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1		//configure peripheral bus clock to 10 MHz            
#define FOSC 80000000
#define PB_DIV 4
#define PRESCALE 256
#define TICKS_SEC 2                // interrupt at 100th of a second
#define INT_SEC     10           // interrupt at 10th of a second
#define CORE_TICK_RATE	    (SYS_FREQ/2/INT_SEC) 
#define SAMPLE_FREQ 20000
#define T2_TICK_RATE	    (FOSC/PB_DIV/PRESCALE/SAMPLE_FREQ)
#define REFRESH_SSD  (SYS_FREQ/2/100)   //Counter to set refresh rate of displays 100/sec
#define SAMPLE_RATE 5000
#define FPB  36000000
#define MAX_PWM FPB/SAMPLE_RATE // eg 7200 at 5000hz.
#define T1_TICK_RATE (SYS_FREQ/2/INT_SEC)

/* ------------------------------------------------------------ */
/*				Definitions                                     */
/* ------------------------------------------------------------ */    
#define Btn1        PORTGbits.RG6   //Define Btn1 to appropriate port bit
#define Btn2        PORTGbits.RG7//Define Btn2 to appropriate port bit

#define Led1        LATGbits.LATG12 //Define Led1 to appropriate port latch bit
#define Led2        LATGbits.LATG13 //Define Led2 to appropriate port latch bit
#define Led3        LATGbits.LATG14  //Define Led3 to appropriate port latch bit
#define Led4        LATGbits.LATG15  //Define Led4 to appropriate port latch bit


//SSD1
#define SegA1       LATBbits.LATB7
#define SegB1       LATBbits.LATB8
#define SegC1       LATBbits.LATB9
#define SegD1       LATBbits.LATB10
#define SegE1       LATEbits.LATE4
#define SegF1       LATEbits.LATE5
#define SegG1       LATEbits.LATE6
#define DispSel1    LATEbits.LATE7 //Select between the cathodes of the 2 SSDs


// SSD2
#define SegA2       LATBbits.LATB15
#define SegB2       LATDbits.LATD5
#define SegC2       LATDbits.LATD4
#define SegD2       LATBbits.LATB14
#define SegE2       LATDbits.LATD1
#define SegF2       LATDbits.LATD2
#define SegG2       LATDbits.LATD3
#define DispSel2    LATDbits.LATD12 //Select between the cathodes of the 2 SSDs


#define LSS1       PORTDbits.RD14
#define LSS2       PORTFbits.RF8
#define LSS3       PORTFbits.RF2
#define LSS4       PORTDbits.RD15    



int Period = 1/(MOTOR_UPDATE);
void DelayCount(int count);
void DisplayChar(int num,int SSD);




/* ------------------------------------------------------------ */
/*				Variables                                      */
/* ------------------------------------------------------------ */    
//SSDCONFIG
int left_SSD = 0;
int ones = 0; 
int tens = -1; 
int hunds = -1; 
int thous = -1; 





int main(void) {
    
	//Output compares	
    
    OC2CON = 0x0000; // Turn off the OC1 when performing the setup
    OC2R = 0x0064; // Initialize primary Compare register
    OC2RS = 0x0064; // Initialize secondary Compare register
    OC2CON = 0x0006; // Configure for PWM mode without Fault pin
    // enabled
    
    PR2 = 0x00C7; // Set period
    OC2CONSET=0x0000;
    
   
    OC3CON = 0x0000; // Turn off the OC1 when performing the setup
    OC3R = 0x0064; // Initialize primary Compare register
    OC3RS = 0x0064; // Initialize secondary Compare register
    OC3CON = 0x0006; // Configure for PWM mode without Fault pin
    // enabled
    
    PR3 = 0x00C7; // Set period
    OC3CONSET=0x0000;
	
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE); //  Configure device for max performance
    OpenCoreTimer(CORE_TICK_RATE);
    
    
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK_RATE);// Configure Timer 1
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_3); // set the timer interrupt priority level to 2
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0)); // set up the core timer interrupt with a prioirty of 2 and zero sub-priority 
    INTEnableSystemMultiVectoredInt(); //multi-vectored interrupts
    
    
    PORTSetPinsDigitalOut (IOPORT_G, BIT_12|BIT_13|BIT_14|BIT_15); //Set LD1 through LD4 as digital output
    PORTClearBits (IOPORT_G, BIT_12|BIT_13| BIT_14|BIT_15); //Turn all 4 LEDs OFF
    PORTSetPinsDigitalIn (IOPORT_G, BIT_6 |BIT_7); //Set Btn1 and Btn2 as inputs
    PORTSetPinsDigitalOut (IOPORT_B,BIT_7|BIT_8| BIT_9| BIT_10| BIT_14| BIT_15);// Set MX7 Port A as output for SSD1R
    PORTSetPinsDigitalOut (IOPORT_E,BIT_4|BIT_5| BIT_6|BIT_7);// Set MX7 Port B as output for SSD1R
    PORTSetPinsDigitalOut (IOPORT_D,BIT_1|BIT_2| BIT_3|BIT_4 |BIT_5| BIT_12);// Set MX7 Port C and D as output for SSD2

    OpenTimer2(T2_ON|T2_SOURCE_INT|T2_PS_1_256,MOTOR_TICK_RATE);
    mT2SetIntPriority(3);
    mT2SetIntSubPriority(0);
    mT2ClearIntFlag();
    mT2IntEnable(1);
	
    int ButtonCock = 0;
    int mode = 1;
    int pause = 0;
	
    while(1){
        DisplayChar(ones,0);
        DisplayChar(hunds,2);
        while (!mCTGetIntFlag());           // wait for flag to set
        mCTClearIntFlag();                  // clear CoreTimer flag
        UpdateCoreTimer(REFRESH_SSD);       // updating core timer.
        DisplayChar(tens,1);
        DisplayChar(thous,3);  
        while (!mCTGetIntFlag());           // wait for flag to set
        mCTClearIntFlag();                  // clear CoreTimer flag
        UpdateCoreTimer(REFRESH_SSD);       // updating core timer.
        
        //Timer 
 
        
        
        
        //Current State Logic
        
        if(mode == 1){;
            //LEDs
            Led1 = Led2 = Led3 = Led4 = 1;
            
            //Timer
            pause = 1;
            
            
        }
        if(mode == 2){
            //LEDs
            Led1 = 1;
            Led2 = Led3 = Led4 = 0;
            
            //Timer
            pause = 0;
            
        }
        if(mode == 3){
            //LEDs
            Led1 = Led2 = 1;
            Led3 = Led4 = 0;
            
            //Timer
            pause = 0;
            
        }
        if(mode == 4){
            //LEDs
            Led1 = Led2 = 0;
            Led3 = Led4 = 1;
            
            //Timer
            pause = 0;
        }
         if(mode == 5){
            //LEDs
            Led1 = Led2 = Led3 = 0;
            Led4 = 1;
            
            //Timer
            pause = 0;
        }
       
        
        
        
        // Next State Logic
        if(Btn1 && !ButtonCock && mode == 1){
            mode = 2;
            ButtonCock = 1; 
        }
        
         if(Btn1 && !ButtonCock && mode == 2){
            mode = 3;
            ButtonCock = 1; 
        }
        
        if(Btn1 && !ButtonCock && mode == 3){
            mode = 4;
            ButtonCock = 1; 
        }
        
        if(Btn1 && !ButtonCock && mode == 4){
             mode = 5;
             ButtonCock = 1;
         }
         if(Btn1 && !ButtonCock && mode  == 5){
             mode = 1;
             ButtonCock = 1;
         }
        
         if(!Btn1 && ButtonCock){
             ButtonCock = 0;
         }
        
    }
    
}



/* ------------------------------------------------------------  
**  __ISR for the servo motor Update using Timer2 
**  Parameters: 
**    none 
**  Return Value: 
**    none 
**  Description: 
**    Updates the servo motors to move the robot 
** ------------------------------------------------------------ */ 
void __ISR(_TIMER_2_VECTOR, IPL3SOFT) Timer2Handler(void){
    
        SetDCOC2PWM(30);
        SetDCOC3PWM(85);
        mT2ClearIntFlag();
}

    



void DisplayChar(int num,int SSD) //used to diplay the characters of the SSD
{
    if(SSD > 1)
    {      
        DispSel2 = SSD-2;
        switch(num)
        {           
                
            case -1: //off
                SegA2=0; SegB2=0; SegC2=0; SegD2=0; SegE2=0; SegF2=0; SegG2=0;break;
            case 0:
                SegA2=1; SegB2=1; SegC2=1; SegD2=1;SegE2=1; SegF2=1; SegG2=0;break;
            case 1:
                SegA2=0; SegB2=0; SegC2=0; SegD2=0;SegE2=1; SegF2=1; SegG2=0;break;
            case 2:
                SegA2=1; SegB2=1; SegC2=0; SegD2=1;SegE2=1; SegF2=0; SegG2=1;break;
            case 3:
                SegA2=1; SegB2=1; SegC2=0; SegD2=0;SegE2=1; SegF2=1; SegG2=1;break;
            case 4:
                SegA2=0; SegB2=0; SegC2=1; SegD2=0;SegE2=1; SegF2=1; SegG2=1;break;
            case 5:
                SegA2=1; SegB2=0; SegC2=1; SegD2=1;SegE2=0; SegF2=1; SegG2=1;break;
            case 6:
                SegA2=1; SegB2=1; SegC2=1; SegD2=1;SegE2=0; SegF2=1; SegG2=1;break;
            case 7:
                SegA2=0; SegB2=0; SegC2=0; SegD2=1;SegE2=1; SegF2=1; SegG2=0;break;
            case 8:
                SegA2=1; SegB2=1; SegC2=1; SegD2=1;SegE2=1; SegF2=1; SegG2=1;break;
            case 9:
                SegA2=0; SegB2=0; SegC2=1; SegD2=1;SegE2=1; SegF2=1; SegG2=1;break;
            default:
                break;
        }   
    }
    else
    {
        DispSel1 = SSD;
        switch(num)
        {
              case -1: //off
                SegA2=0; SegB2=0; SegC2=0; SegD2=0; SegE2=0; SegF2=0; SegG2=0;break;
            case 0:
                SegA2=1; SegB2=1; SegC2=1; SegD2=1;SegE2=1; SegF2=1; SegG2=0;break;
            case 1:
                SegA2=0; SegB2=0; SegC2=0; SegD2=0;SegE2=1; SegF2=1; SegG2=0;break;
            case 2:
                SegA2=1; SegB2=1; SegC2=0; SegD2=1;SegE2=1; SegF2=0; SegG2=1;break;
            case 3:
                SegA2=1; SegB2=1; SegC2=0; SegD2=0;SegE2=1; SegF2=1; SegG2=1;break;
            case 4:
                SegA2=0; SegB2=0; SegC2=1; SegD2=0;SegE2=1; SegF2=1; SegG2=1;break;
            case 5:
                SegA2=1; SegB2=0; SegC2=1; SegD2=1;SegE2=0; SegF2=1; SegG2=1;break;
            case 6:
                SegA2=1; SegB2=1; SegC2=1; SegD2=1;SegE2=0; SegF2=1; SegG2=1;break;
            case 7:
                SegA2=0; SegB2=0; SegC2=0; SegD2=1;SegE2=1; SegF2=1; SegG2=0;break;
            case 8:
                SegA2=1; SegB2=1; SegC2=1; SegD2=1;SegE2=1; SegF2=1; SegG2=1;break;
            case 9:
                SegA2=0; SegB2=0; SegC2=1; SegD2=1;SegE2=1; SegF2=1; SegG2=1;break;
            default:
                break;
        }
    }
}