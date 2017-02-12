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
  // #include <driverlib/interrupt.h>
  // #include <driverlib/fpu.h>
  // #include <driverlib/debug.h>
  #include <driverlib/pwm.h>
  #include <driverlib/pin_map.h>
  // #include <driverlib/qei.h>

  #include <inc/hw_memmap.h>
  // #include <inc/hw_types.h>
  // #include <inc/hw_gpio.h>
  // #include <inc/hw_qei.h>
}
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

// ROS nodehandle

void initialize_pins() {

  // Set system clock to 80MHz using a PLL (200MHz / 2.5 = 80MHz)
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_16MHZ);

  /* Set PWM clock base frequency.
   * PWM base frequency is the processor clock frequency divided by X, where
   * the input to SysCtlPWMClockSet is SYSCTL_PWMDIV_X.
   * If you're using the external oscillator (default), the processor clock
   * will be 80MHz. In this case, the divisor is 8, so the frequency base is
   * 10MHz. Note that this isn't the actual frequency of the PWM signal. That
   * will be set later to be an integer multiple of this value.
   */
  SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

  // Enable the PWM peripheral and wait for it to be ready.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)){}

  // Enable the GPIO ports with base F and wait for it to be ready.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}

  //Enable GPIO ports with base D and wait for it to be ready
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)){}

  // Configure the internal multiplexer to connect the PWM peripheral to PF3
  // PF3 is also attached to the LED
  GPIOPinConfigure(GPIO_PF3_M1PWM7);

  // Set up the PWM module on pin PF3
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
  // INA pin chosen as Tiva Pin D0
  GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
  // INB pin chosen as Tiva Pin D1
  GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
  //Can also write as:
  //GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);

  /* Configure PWM mode to count up/down without synchronization
   * This is just a detail, you probably don't have to worry about it.
   * See pages 1235 and 1237 in the data sheet if you want to learn more.
   */
  PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                  PWM_GEN_MODE_NO_SYNC);
}

void motor_run(int direction, double percentage)
{
  long PWMFrequencyHz= 250;
  double PWMDutyCycleDivisor=(100.00/percentage);

  int periodConstant= SysCtlPWMClockGet()/PWMFrequencyHz;
  /*
   * Warning! The PWM generator often freaks out when you set 100% duty cycle.
   * It's a known issue in the silicon:
   * https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/448664
   * To avoid this, cap the PWM output at (MaxDuty - 1). I personally wouldn't
   * trust it at 0% either. If you strictly need 0% and 100%, you should
   * switch the pin back to a GPIO. Though this may take some time (~5us).
   */

  if ((direction == 1 || direction== -1 ) && percentage>0 && percentage < 100 && PWMDutyCycleDivisor!=1.000) {

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, periodConstant);

    // As muxed from M1PWM7
    int pulseWidth= PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)/PWMDutyCycleDivisor;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pulseWidth);

    // Enable the PWM output signal
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
    // Enable the PWM peripheral
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    //can also write as:
    //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00);
    //GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0xFF);

    // set motor speed according to PWM value
    if (direction==1){
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00);
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0xFF);
    }
    else if(direction==-1){
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0xFF);
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
    }
    while(1){}
  }

  else{
    // Illegal direction or percentage argument
  }


  // ROS_INFO("request: Direction=%d, Percentage=%f", req.direction, req.percentage);
  // ROS_INFO("sending back response: [%s]", (res.success == 1) ? "True" : "False");
}

void motor_callback(const std_msgs::Float64& msg){
  double percentage = (double) abs(msg.data);

if (msg.data == 0){
  motor_run(0,0);
 }
if(msg.data > 0.0){
  motor_run(1, percentage);
 }
 else{
   motor_run(-1,percentage);
 }
}

ros::NodeHandle nh;
std_msgs::Float64 velocity_msg;
ros::Subscriber<std_msgs::Float64> sub("motor_vel", &motor_callback);

int main(int argc, char **argv)
{
  initialize_pins();


  // ros::init(argc, argv, "motor_vel_command");
  // ros::ServiceServer service = nh.advertiseService("motor_set_angular_vel", motor_run);
  // ROS_INFO("Ready to set motor angular velocity.");


  nh.spinOnce();

  return 0;
}
