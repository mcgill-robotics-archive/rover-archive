/*********************************************************************
 *
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  rosserial_tivac chatter tutorial
 *
 *  On this demo your TivaC Launchpad will publish a string over the
 * topic "/chatter".
 *
 * Full guide: http://wiki.ros.org/rosserial_tivac/Tutorials
 *
 *********************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

// TivaC specific includes
extern "C"
{
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/fpu.h>
#include <driverlib/debug.h>
#include <driverlib/pwm.h>
#include <driverlib/pin_map.h>
#include <driverlib/qei.h>
#include <driverlib/timer.h> // Defines and macros for Timer API of DriverLib.
#include <driverlib/uart.h> // Definitions and macros for UART API of DriverLib.
#include <driverlib/adc.h> // Definitions for ADC API of DriverLib.
#include <utils/uartstdio.h> // Prototypes for the UART console functions.
#include <utils/uartstdio.c>                            // Needs to add "utils/uartstdio.c" through a relative link.

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_qei.h>
}
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <arm_tiva/Magnetic_encoder.h>

// ROS nodehandle
ros::NodeHandle nh;

// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);
arm_tiva::Magnetic_encoder encoder_message;

ros::Publisher encoder_magnetic("position", &encoder_message);

#define UART1_BAUDRATE      115200  // UART baudrate in bps

// function prototypes
void init_timer(void);
void init_UART(void);
void init_pwm(void);
void interrupt_h_encoder_0(void);
void interrupt_h_encoder_1(void);
void interrupt_h_encoder_2(void);
void duty_cycle_0(void);
void duty_cycle_1(void);
void duty_cycle_2(void);

// global variables
uint32_t sys_clock;
int32_t  start = 0, end = 0, length = 0;
uint32_t angle0, angle1, angle2;
int encoder0_flag, encoder1_flag, encoder2_flag;


int main(void)
{
  encoder0_flag=0;
  encoder1_flag=0;
  encoder2_flag=0;

  // Configure system clock at 80 MHz.
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  sys_clock = SysCtlClockGet();

  // Enable the processor to respond to interrupts.
  IntMasterEnable();

  init_UART();
  init_pwm();
  init_timer();

  TimerEnable(WTIMER0_BASE, TIMER_BOTH);
  TimerEnable(WTIMER1_BASE, TIMER_BOTH);
  TimerEnable(WTIMER2_BASE, TIMER_BOTH);

  nh.initNode();
  nh.advertise(encoder_magnetic);
  encoder_message.angle0 = 0;
  encoder_message.angle1 = 0;
  encoder_message.angle2 = 0;
  encoder_magnetic.publish(&encoder_message);
  nh.spinOnce();
  nh.getHardware()->delay(30);

  while(1) {
  encoder_message.angle0 = 0;
  encoder_message.angle1 = 0;
  encoder_message.angle2 = 0;
  encoder_magnetic.publish(&encoder_message);
  nh.spinOnce();
  nh.getHardware()->delay(30);
    if (encoder0_flag==1){
      duty_cycle_0();
      nh.spinOnce();
      encoder_message.angle0 = angle0;
      encoder_magnetic.publish(&encoder_message);
    }
    else if(encoder1_flag==1){
      duty_cycle_1();
      nh.spinOnce();
      encoder_message.angle1 = angle1;
      encoder_magnetic.publish(&encoder_message);
    }
    else if(encoder2_flag== 1){
      duty_cycle_2();
      nh.spinOnce();
      encoder_message.angle2 = angle2;
      encoder_magnetic.publish(&encoder_message);
    }

  }
}

void init_UART(void)
{
  // Enable and configure UART0 for debugging printouts.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
  UARTStdioConfig(0, UART1_BAUDRATE, sys_clock);
}

void init_pwm(void)
{
  /*This function configures the rate of the clock provided to the PWM module as a ratio of the
    processor clock. This clock is used by the PWM module to generate PWM signals; its rate
    forms the basis for all PWM signals*/
  //PWM clock rate is 80MHz/8 = 10 MHz
  SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)){}

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}

  //PF3 is the PWM output
  GPIOPinConfigure(GPIO_PF3_M1PWM7);
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

  PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                  PWM_GEN_MODE_NO_SYNC);

  //PWM Period = 10000 PWM clock cycles = 10000*1/10M = 0.001 seconds
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 10000);

  //PWM Pulse width is 500 clock cycles, 500/10000 = 0.05, 0.05*0.001 = 0.00005 seconds
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 500);
  PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
  PWMGenEnable(PWM1_BASE, PWM_GEN_3);
}

void init_timer(void)
{
  // Enable and configure Timer0 peripheral.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);

  // Initialize timer A and B to count up in edge time capture mode
  TimerConfigure(WTIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP));

  TimerConfigure(WTIMER1_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP));

  TimerConfigure(WTIMER2_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP));

  // Timer a records pos edge time and Timer b records neg edge time
  TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerControlEvent(WTIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

  TimerControlEvent(WTIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerControlEvent(WTIMER1_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

  TimerControlEvent(WTIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerControlEvent(WTIMER2_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

  //TimerPrescaleSet(WTIMER0_BASE, TIMER_A , 0 );
  //TimerPrescaleSet(WTIMER0_BASE, TIMER_B , 0 );

  //set the value that the timers count to (0x9C3F = 39999)
  //CO2 sensor outputs 1khz pwm so with mcu at 40Mhz, timers should stay in sync with CO2 output
  TimerLoadSet(WTIMER0_BASE, TIMER_BOTH, 0x9C3F);
  TimerLoadSet(WTIMER1_BASE, TIMER_BOTH, 0x9C3F);
  TimerLoadSet(WTIMER2_BASE, TIMER_BOTH, 0x9C3F);
  //Configure the pin that the timer reads from (PB6)
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

  GPIOPinConfigure(GPIO_PC4_WT0CCP0);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);
  GPIOPinConfigure(GPIO_PC5_WT0CCP1);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5);

  GPIOPinConfigure(GPIO_PC6_WT1CCP0);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);
  GPIOPinConfigure(GPIO_PC7_WT1CCP1);
  GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_7);

  GPIOPinConfigure(GPIO_PD0_WT2CCP0);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
  GPIOPinConfigure(GPIO_PD1_WT2CCP1);
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_1);



  // Registers a interrupt function to be called when timer b hits a neg edge event
  IntRegister(INT_WTIMER0B, interrupt_h_encoder_0);
  // Makes sure the interrupt is cleared
  TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);
  // Enable the indicated timer interrupt source.
  TimerIntEnable(WTIMER0_BASE, TIMER_CAPB_EVENT);

  // Registers a interrupt function to be called when timer b hits a neg edge event
  IntRegister(INT_WTIMER1B, interrupt_h_encoder_1);
  // Makes sure the interrupt is cleared
  TimerIntClear(WTIMER1_BASE, TIMER_CAPB_EVENT);
  // Enable the indicated timer interrupt source.
  TimerIntEnable(WTIMER1_BASE, TIMER_CAPB_EVENT);

  // Registers a interrupt function to be called when timer b hits a neg edge event
  IntRegister(INT_WTIMER1B, interrupt_h_encoder_2);
  // Makes sure the interrupt is cleared
  TimerIntClear(WTIMER2_BASE, TIMER_CAPB_EVENT);
  // Enable the indicated timer interrupt source.
  TimerIntEnable(WTIMER2_BASE, TIMER_CAPB_EVENT);

  // The specified interrupt is enabled in the interrupt controller.
  IntEnable(INT_WTIMER0B);
  IntEnable(INT_WTIMER1B);
  IntEnable(INT_WTIMER2B);
}

void
interrupt_h_encoder_0(void){
  TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);
  start = TimerValueGet(WTIMER0_BASE, TIMER_A);
  end = TimerValueGet(WTIMER0_BASE, TIMER_B);
  encoder0_flag= 1;
}

void
interrupt_h_encoder_1(void){
  TimerIntClear(WTIMER1_BASE, TIMER_CAPB_EVENT);
  start = TimerValueGet(WTIMER1_BASE, TIMER_A);
  end = TimerValueGet(WTIMER1_BASE, TIMER_B);
  encoder1_flag= 1;
}

void
interrupt_h_encoder_2(void){
  TimerIntClear(WTIMER2_BASE, TIMER_CAPB_EVENT);
  start = TimerValueGet(WTIMER2_BASE, TIMER_A);
  end = TimerValueGet(WTIMER2_BASE, TIMER_B);
  encoder2_flag= 1;
}

//When negative edge is hit, record the values and find the difference, output to putty
void duty_cycle_0(void)
{
  encoder0_flag= 0;
  length = end - start;
  if (length<0){
    //UARTprintf("\nLENGTH0 = %d\n", length);
    length= 4000+length;
  }
  //notes: 10MHZ clock, duty cycle of 500 pwm clock cycles => length is always 0.00005 seconds
  //length = 4001 => 4001 sysclock ticks => 4000 * 1/80M = 0.5 E-4 , length read by timer matches PWM duty cycle


  //since PWM period is 10000 PWM clock cycles, and length is given in system clock cycles
  //PWM subdiv = 8
  angle0 = (360*length)/80000;
  UARTprintf("\nSTART0 = %d\n", start);
  UARTprintf("\nEND0 = %d\n", end);
  UARTprintf("\nLENGTH0 = %d\n", length);
  UARTprintf("\nANGLE0 = %d\n", angle0);
}

void duty_cycle_1(void)
{
  encoder1_flag=0;

  length = end - start;
  if (length<0){
    UARTprintf("\nLENGTH1 = %d\n", length);
    length= 4000+length;
  }
  //notes: 10MHZ clock, duty cycle of 500 pwm clock cycles => length is always 0.00005 seconds
  //length = 4001 => 4001 sysclock ticks => 4000 * 1/80M = 0.5 E-4 , length read by timer matches PWM duty cycle


  //since PWM period is 10000 PWM clock cycles, and length is given in system clock cycles
  //PWM subdiv = 8
  angle1 = (360*length)/80000;
  UARTprintf("\nSTART1 = %d\n", start);
  UARTprintf("\nEND1 = %d\n", end);
  UARTprintf("\nLENGTH1 = %d\n", length);
  UARTprintf("\nANGLE1 = %d\n", angle1);
}

void duty_cycle_2(void)
{
  encoder2_flag=0;

  length = end - start;
  if (length<0){
    UARTprintf("\nLENGTH2 = %d\n", length);
    length= 4000+length;
  }
  //notes: 10MHZ clock, duty cycle of 500 pwm clock cycles => length is always 0.00005 seconds
  //length = 4001 => 4001 sysclock ticks => 4000 * 1/80M = 0.5 E-4 , length read by timer matches PWM duty cycle


  //since PWM period is 10000 PWM clock cycles, and length is given in system clock cycles
  //PWM subdiv = 8
  angle2 = (360*length)/80000;
  UARTprintf("\nSTART2 = %d\n", start);
  UARTprintf("\nEND2 = %d\n", end);
  UARTprintf("\nLENGTH2 = %d\n", length);
  UARTprintf("\nANGLE2 = %d\n", angle2);
}
