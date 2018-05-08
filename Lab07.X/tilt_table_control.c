/*
  This program controls the tilt table
 This version assume the servo goes from 0 degrees (553 mu -sec)
 to 197 degrees (3530 mu -sec)
 
 The program works in radians
 
  It used timer1 and interrupts
 */

#include <p30f4011.h>

// #define	FCY	117920000
#include <libpic30.h>
#include <delay.h>
#include <uart.h>
#include <string.h>
#include <stdio.h>
#include <qei.h>
#include <pwm.h>
#include <adc10.h>
#include <timer.h>
#include <ports.h>

// Configuration Bits

#pragma config FPR = FRC_PLL16
#pragma config FOS = PRI
#pragma config FCKSMEN = CSW_FSCM_OFF
#pragma config WDT = WDT_OFF
#pragma config FPWRT = PWRT_16
#pragma config BODENV = BORV27
#pragma config BOREN = PBOR_OFF
#pragma config MCLRE = MCLR_EN
#pragma config GWRP = GWRP_OFF


// define some variables

#define max(A ,B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))


#define MAX_COUNT 1
#define MAX_ISUM 0.1
#define MIN_DUTY_SERVO 256   // corresponds to 0 degrees
#define MAX_DUTY_SERVO 857 // 1166   // corresponds to 130 degress
#define MAX_DELTA_U_DUTY 0.5*4.6 // 4.6 corresponds to a 1 degree change 
#define PERIOD  4630
//4630 
#define MAX_DUTY 2*PERIOD // using up/down mode for pwm
#define MAX_COUNT 1
#define PI 3.14159265

unsigned int GO;
int encindex, p2;

/***************************************************************/

// Initialize timer 1

void Init_Timer1(unsigned int period) {
    unsigned int config;

    config = T1_INT_PRIOR_4 & // set interrupt priority to 2
            T1_INT_ON; // enable the interrupts

    ConfigIntTimer1(config);

    config = T1_ON & // turn on the timer
            T1_IDLE_CON & // operate during sleep
            T1_GATE_OFF & // timer gate accumulation is disabled
            T1_PS_1_256 & // timer prescale is 256
            T1_SYNC_EXT_OFF & // don't synch with external clock
            T1_SOURCE_INT; // use the internal clock

    OpenTimer1(config, period);

    TRISEbits.TRISE1 = 0; // prepare for the overrun LED indicator
    LATEbits.LATE1 = 0; //  the LED should be off

    return;
}
/***************************************************************/

// timer 1 interrupt handler

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    unsigned int ReadQEI(void);
    extern unsigned int GO;
    extern int p2;

    // update the position variables

    p2 = (int) ReadQEI();

    // reset Timer 1 interrupt flag 

    IFS0bits.T1IF = 0;

    // if GO is 1 we are not done before the next interrupt!

    if (GO == 1) // should be 1
        LATEbits.LATE1 = 1;
    GO = 1;
}
/***********************************************************************/

// Initialize external interrupt 1

void Init_INT1(void) {
    unsigned int config;

    config = FALLING_EDGE_INT & // interrupt on a falling edge
            EXT_INT_ENABLE & // enable the interrupts
            //EXT_INT_PRI_0 ;
            GLOBAL_INT_ENABLE;

    ConfigINT1(config);


    // turn on the LED to show interrupt is set

    TRISFbits.TRISF6 = 0;
    LATFbits.LATF6 = 1; // signal the interrupt is set

    // prepare for an input on RD0

    TRISDbits.TRISD0 = 1;

    // enable the interrupt

    DisableINT1;
    EnableINT1;

    return;
}

/***************************************************************/

// external input interrupt handler

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    unsigned int dutycyclereg, dutycycle;
    char updatedisable;

    // turn off the pwm signal

    dutycycle = (unsigned int) 0;
    dutycyclereg = 2;
    updatedisable = 0;
    SetDCMCPWM(dutycyclereg, dutycycle, updatedisable); // duty cycle set to low


    // turn off the LED to indicate power if off

    TRISFbits.TRISF6 = 0;
    LATFbits.LATF6 = 0; // signal the power is off

    // Disable the interrupt

    DisableINT1;

    // now just wait

    while (1);
}

/*****************************************************************/

// setup the QEI encoder

void encoder_init(void) {

    unsigned int config1, config2;

    config1 = QEI_DIR_SEL_QEB &
            QEI_INT_CLK &
            QEI_INDEX_RESET_DISABLE & // QEI index pulse resets position counter 
            QEI_CLK_PRESCALE_1 &
            QEI_GATED_ACC_DISABLE &
            QEI_NORMAL_IO &
            QEI_INPUTS_NOSWAP &
            QEI_MODE_x4_MATCH & // reset on match, was x2, increase resolution by 4
            QEI_DOWN_COUNT & // count up
            QEI_IDLE_CON; // continue on idle

    config2 = POS_CNT_ERR_INT_DISABLE & // disable error interrupts
            QEI_QE_CLK_DIVIDE_1_1 & //1_256
            QEI_QE_OUT_ENABLE & // enable digital filter
            MATCH_INDEX_INPUT_PHASEA &
            MATCH_INDEX_INPUT_LOW;

    OpenQEI(config1, config2);

    config1 = QEI_INT_ENABLE & // enable the interups
            QEI_INT_PRI_2; // set the priority to two

    WriteQEI((unsigned int) MAXCNT);

    ConfigIntQEI(config1);
}

/************************************************************/

// QEI interrupt handler

void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void) {
    extern int encindex;

    // update the encoder count every time the counter POSCNT gets to MAXCNT

    if (QEICONbits.UPDN) {
        encindex++;
    }
    else {
        encindex--;
    }

    IFS2bits.QEIIF = 0; // 
}

/*************************************************************/

// setup the UART

void uart1_init(void) {

    unsigned int config1, config2, ubrg;

    config1 = UART_EN & // enable the UART
            UART_IDLE_CON & // set idle mode
            UART_ALTRX_ALTTX & // use the alternate pins 
            UART_DIS_WAKE & // disable wake-up on start
            UART_DIS_LOOPBACK & // disable loopback
            UART_DIS_ABAUD & // disable autobaud rate detect
            UART_NO_PAR_8BIT & // no parity, 8 bits
            UART_1STOPBIT; // one stop bit

    config2 = UART_INT_TX_BUF_EMPTY & // interrupt anytime a buffer is empty
            UART_TX_PIN_NORMAL & // set transmit break pin
            UART_TX_ENABLE & // enable UART transmission
            UART_INT_RX_CHAR & // receive interrupt mode selected
            UART_ADR_DETECT_DIS & // disable address detect
            UART_RX_OVERRUN_CLEAR; // overrun bit clear

    ubrg = 15; // 115200 baud

    OpenUART1(config1, config2, ubrg);
}

/***************************************************************/

// setup pwm

void pwm_init(void) {

    unsigned int config1, config2, config3;
    unsigned int sptime;

    config1 = PWM_INT_DIS & // disable the interrupt
            PWM_FLTA_DIS_INT; // disable the interrupt on fault

    ConfigIntMCPWM(config1);

    config1 = PWM_EN & //  enable the PWM module
            PWM_IPCLK_SCALE64 & // input prescaler set to 1
            PWM_OP_SCALE1 & // post scalar set to 1
            PWM_MOD_UPDN; // up/down mode

    config2 = PWM_MOD1_IND & // pwm modules run independently
            PWM_MOD2_IND &
            PWM_MOD3_IND &
            PWM_PDIS1H & // disable 1 high
            PWM_PEN2H & // enable 2 high
            PWM_PDIS3H & // disable 3 high
            PWM_PDIS1L & // disable 1 low
            PWM_PDIS2L & // disable 2 low
            PWM_PDIS3L; // disable 3 low

    config3 = PWM_UEN; // enable updates

    sptime = 0x0;

    OpenMCPWM(PERIOD, sptime, config1, config2, config3);

}

/***************************************************************/

int main(void) {
    extern unsigned int GO;
    extern int p2;
    unsigned int dutycyclereg, dutycycle, period;
    double angle, servo, time, dt;
    int count;
    double u_ref_rad, u_ref_duty, angle_ref;
    double Y, u, error, last_error, Derror, Isum, R;
    double u_duty, u_duty_last;
    char updatedisable = 0;
    double scale_QEI = 2 * PI / 8191.0; // scale to radians
    double scale_servo = 264.875, offset_servo = 256.04;
    double scale_servo_deg = 0.21633, offset_servo_deg = -55.388;
    double kp = 0.4, ki = 1, kd = 0.95;
    // 0 degrees corresponds to 553 mu-seconds
    // 197 degrees corresponds to 2520 mu-seconds
    // the input angle is in radians
    // the output is the corresponding duty cycle
    // y [duty cycle] = scale_servo * x [radians] + offset_servo

    // enable the input for QEI

    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;

    // we also need to set these bits for the QEI 

    ADPCFGbits.PCFG3 = 1;
    ADPCFGbits.PCFG4 = 1;
    ADPCFGbits.PCFG5 = 1;

    // prepare to output for the pwm, and sampling too fast

    TRISEbits.TRISE1 = 0; // sampling too fast
    TRISEbits.TRISE3 = 0; // pwm for servo

    // set up the external interrupt

    Init_INT1();

    // set up the pwm

    pwm_init();

    // set up the uart

    uart1_init();

    // set up the encoder

    encoder_init();

    // enable the QEI interrupt

    EnableIntQEI;

    // initialize timer1

    // dt should be less than or equal to 0.25

    dt = 0.05; // 0.5 the sampling interval in seconds

    // dt = N*256/29,480,000;  assuming a 256 prescaler.
    // so N = dt* 115156

    period = (unsigned int) (dt * 115156.0);

    if (period > 32768) {
        period = 32768;
    }

    printf("....period is %6d (< 32768?)  \n ", period);

    Init_Timer1(period);

    time = -dt;

    count = MAX_COUNT;
    GO = 0;

    /******************** SETUP LOOP **********************/

    while (time < 10.0) {

        while (!GO);
        time = time + dt;

        angle_ref = ((double) p2) * scale_QEI;

        // the set point is 100.0 for this run

        u_ref_rad = 103.3 * PI / 180.0; // convert degrees to radians
        u_ref_duty = u_ref_rad * scale_servo + offset_servo;

        dutycycle = (unsigned int) u_ref_duty;

        dutycyclereg = 2;
        SetDCMCPWM(dutycyclereg, dutycycle, updatedisable);

        servo = dutycycle * scale_servo_deg + offset_servo_deg;

        if (--count == 0) {
            printf("%8.4f %4d %8.4f %5d %8.4f %8.4f \n", time, p2, angle_ref, dutycycle, u_ref_duty, servo);
            count = MAX_COUNT;
        }

        GO = 0; // all done 

    }

    // reset the time

    time = -dt;

    last_error = 0.0;
    Isum = 0.0;
    u_duty_last = u_ref_duty;

    /********************* MAIN LOOP **********************/

    while (1) {

        while (!GO);

        time = time + dt;


        /*********************************************/
        //  implement the PREFILTER (Gpf) functions
        //
        //  the reference signal is 0 since this is a regulator
        //
        /*********************************************/

        R = 0.0;

        /*********************************************/
        //  implement the FEEDBACK (H) functions
        //
        //  Even if H is not explicitly written, we still need to
        //  sample the output and convert it to the correct units.       
        //  For the tilt table, the units are radians
        //  
        // we are measuring everything about the reference angle
        // 
        /*********************************************/

        angle = ((double) p2) * scale_QEI - angle_ref;
        Y = angle;

        /*********************************************/
        //  implement the ERROR computation
        //
        //  The error is the difference between the (possibly)
        //  modified reference signal and the (possibly modified)
        //  output
        //
        /*********************************************/

        error = R - Y;

        // the following is because the system is not quite exact (not symmetric))

        if (error > 0.0) error = error * 1.2;

        /*********************************************/
        //  implement the CONTROLLER (Gc) functions
        // 
        /*********************************************/

        // PID controller

        Derror = error - last_error;
        last_error = error;

        Isum = Isum + error;

        if (Isum > 0.0)
            Isum = min(Isum, MAX_ISUM);
        else
            Isum = max(Isum, -MAX_ISUM);

        u = kp * error + kd * Derror + ki*Isum;

        /*********************************************/
        //  convert the control effort to the correct units for
        //  motor, be sure the control effort is within
        //  the allowable range
        //  
        /*********************************************/

        u_duty = (u_ref_rad + u) * scale_servo + offset_servo;

        // don't let angle change any more than ?? degrees per sample

        u_duty = min(u_duty, u_duty_last + MAX_DELTA_U_DUTY);
        u_duty = max(u_duty, u_duty_last - MAX_DELTA_U_DUTY);
        u_duty_last = u_duty;

        u_duty = max(u_duty, MIN_DUTY_SERVO);
        u_duty = min(u_duty, MAX_DUTY_SERVO);

        dutycycle = (unsigned int) u_duty;

        dutycyclereg = 2;
        SetDCMCPWM(dutycyclereg, dutycycle, updatedisable);

        servo = dutycycle * scale_servo_deg + offset_servo_deg;

        if (--count == 0) {
            printf("%6.2f error = %6.2f kp*e = %6.3f ki*i = %6.3f kd*d = %6.3f Isum = %8.4f  angle = %6.2f\n", time, error, kp*error, ki*Isum, kd*Derror, Isum, servo);
            count = MAX_COUNT;
        }

        GO = 0; // all done 
    }
}
