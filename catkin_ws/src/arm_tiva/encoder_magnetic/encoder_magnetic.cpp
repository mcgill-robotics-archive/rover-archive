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

#include <ros.h>
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
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <rover_common/Magnetic_encoder.h>

#define UART1_BAUDRATE      115200  // UART baudrate in bps
// ROS nodehandle
ros::NodeHandle nh;

// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);
rover_common::Magnetic_encoder encoder_message;

ros::Publisher encoder_magnetic("position", &encoder_message);


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
int32_t  start_0 = 0, end_0 = 0, length_0 = 0;
int32_t  start_1 = 0, end_1 = 0, length_1 = 0;
int32_t  start_2 = 0, end_2 = 0, length_2 = 0;

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

  init_timer();
  IntEnable(INT_WTIMER0B);
  IntPrioritySet(INT_WTIMER0B, 2);
  
  IntEnable(INT_WTIMER1B);
  IntPrioritySet(INT_WTIMER1B, 1);
  
  IntEnable(INT_WTIMER2B);
  IntPrioritySet(INT_WTIMER2B, 0);

  uint32_t counter = 0;
  nh.initNode();
  nh.advertise(encoder_magnetic);
  encoder_message.angle0 = 0;
  encoder_message.angle1 = 0;
  encoder_message.angle2 = 0;
  encoder_magnetic.publish(&encoder_message);
  nh.getHardware()->delay(30);
  TimerDisable(WTIMER0_BASE, TIMER_BOTH);
  TimerDisable(WTIMER1_BASE, TIMER_BOTH);
  TimerDisable(WTIMER2_BASE, TIMER_BOTH);
  
  while(1) {
      counter = (counter+1)%3;
      switch (counter){
          case 0:
              TimerEnable(WTIMER0_BASE, TIMER_BOTH);
              TimerDisable(WTIMER2_BASE, TIMER_BOTH);
              break;

        case 1: 
              TimerDisable(WTIMER0_BASE, TIMER_BOTH);
              TimerEnable(WTIMER1_BASE, TIMER_BOTH);
              break;
        case 2:
              TimerDisable(WTIMER1_BASE, TIMER_BOTH);
              TimerEnable(WTIMER2_BASE, TIMER_BOTH);
              break;
      }
      //encoder_message.angle0 = 0;
      //encoder_message.angle1 = 0;
      //encoder_message.angle2 = 0;
      //encoder_magnetic.publish(&encoder_message);
      //nh.spinOnce();
      //nh.getHardware()->delay(30);

    if (encoder0_flag==1){
      duty_cycle_0();
      encoder_message.angle0 = angle0;
      encoder_magnetic.publish(&encoder_message);
    }
    if(encoder1_flag==1){
      duty_cycle_1();
      encoder_message.angle1 = angle1;
      encoder_magnetic.publish(&encoder_message);
    }
    if(encoder2_flag== 1){
      duty_cycle_2();
      encoder_message.angle2 = angle2;
      encoder_magnetic.publish(&encoder_message);
    }

    nh.spinOnce();
    nh.getHardware()->delay(30);
  }
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
  IntRegister(INT_WTIMER2B, interrupt_h_encoder_2);
  // Makes sure the interrupt is cleared
  TimerIntClear(WTIMER2_BASE, TIMER_CAPB_EVENT);
  // Enable the indicated timer interrupt source.
  TimerIntEnable(WTIMER2_BASE, TIMER_CAPB_EVENT);
}

void
interrupt_h_encoder_0(void){
  TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);
  start_0 = TimerValueGet(WTIMER0_BASE, TIMER_A);
  end_0 = TimerValueGet(WTIMER0_BASE, TIMER_B);
  encoder0_flag= start_0 < end_0 ? 1 : 0;
}

void
interrupt_h_encoder_1(void){
  TimerIntClear(WTIMER1_BASE, TIMER_CAPB_EVENT);
  start_1 = TimerValueGet(WTIMER1_BASE, TIMER_A);
  end_1 = TimerValueGet(WTIMER1_BASE, TIMER_B);
  encoder1_flag= start_1 <end_1 ? 1 : 0;
}

void
interrupt_h_encoder_2(void){
  TimerIntClear(WTIMER2_BASE, TIMER_CAPB_EVENT);
  start_2 = TimerValueGet(WTIMER2_BASE, TIMER_A);
  end_2 = TimerValueGet(WTIMER2_BASE, TIMER_B);
  encoder2_flag= start_2< end_2 ? 1 : 0;
}

//When negative edge is hit, record the values and find the difference, output to putty
void duty_cycle_0(void)
{
  encoder0_flag= 0;
  length_0 = end_0 - start_0;
  if (length_0<0){
    //UARTprintf("\nLENGTH0 = %d\n", length);
    length_0= 4000+length_0;
  }
  //notes: 10MHZ clock, duty cycle of 500 pwm clock cycles => length is always 0.00005 seconds
  //length = 4001 => 4001 sysclock ticks => 4000 * 1/80M = 0.5 E-4 , length read by timer matches PWM duty cycle


  //since PWM period is 10000 PWM clock cycles, and length is given in system clock cycles
  //PWM subdiv = 8
  angle0 = (360*length_0)/40000;
}

void duty_cycle_1(void)
{
  encoder1_flag=0;

  length_1 = end_1 - start_1;
  if (length_1<0){
     length_1= 4000+length_1;
  }
  //notes: 10MHZ clock, duty cycle of 500 pwm clock cycles => length is always 0.00005 seconds
  //length = 4001 => 4001 sysclock ticks => 4000 * 1/80M = 0.5 E-4 , length read by timer matches PWM duty cycle


  //since PWM period is 10000 PWM clock cycles, and length is given in system clock cycles
  //PWM subdiv = 8
  angle1 = (360*length_1)/40000;
}

void duty_cycle_2(void)
{
  encoder2_flag=0;

  length_2 = end_2 - start_2;
  if (length_2<0){
      length_2= 4000+length_2;
  }
  //notes: 10MHZ clock, duty cycle of 500 pwm clock cycles => length is always 0.00005 seconds
  //length = 4001 => 4001 sysclock ticks => 4000 * 1/80M = 0.5 E-4 , length read by timer matches PWM duty cycle


  //since PWM period is 10000 PWM clock cycles, and length is given in system clock cycles
  //PWM subdiv = 8
  angle2 = (360*length_2)/40000;
}
