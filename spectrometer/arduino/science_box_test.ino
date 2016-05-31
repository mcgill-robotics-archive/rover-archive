#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#define address 99  //for the ph sensor


//defines for pins //TODO: fill these in with the actual pinout. the ones here are just placeholders
#define pin_humidity 0
#define pin_thermocouple 1

#define pin_augur_in_a1 2
#define pin_augur_in_b1 3
#define pin_augur_pwm_1 4
#define pin_augur_in_a2 5
#define pin_augur_in_b2 6
#define pin_augur_pwm_2 7
#define pin_augur_in_a3 8
#define pin_augur_in_b3 9
#define pin_augur_pwm_3 10
#define pin_augur_limit_switch_up 11
#define pin_augur_limit_switch_down 12
#define pin_augur_servo_limit_switch 13





//initialisation of objects
Adafruit_MPL3115A2 barometer = Adafruit_MPL3115A2();


//humidity
int get_humidity ()
{
	return (analogRead(pin_humidity)); //it's just an uncallibrated read
}

//barometer
float get_pressure ()
{
	return (barometer.getPressure());		//returns the value in pascals
}
float get_altitude ()
{
	return (barometer.getAltitude());		//in meters
}
float get_ambient_temperature ()			//in degrees c
{
	return(barometer.getTemperature());
}

//thermocouple
float get_ground_temperature ()
{
	return (((analogRead(pin_thermocouple)*5.0/1023.0)-1.25)/0.005);
}


//pH sensor
char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer=0;   //we need to know how many characters have been received.
byte serial_event=0;             //a flag to signal when data has been received from the pc/mac/other.
byte code=0;                     //used to hold the I2C response code.
char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit.
byte in_char=0;                  //used as a 1 byte buffer to store in bound bytes from the pH Circuit.
byte i=0;                        //counter used for ph_data array.
int time_=1800;                   //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
float ph_float;                  //float var used to hold the float value of the pH.
void serialEvent()            //this interrupt will trigger when the data coming from the serial monitor(pc/mac/other) is received.
{
	received_from_computer=Serial.readBytesUntil(13,computerdata,20); //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
	computerdata[received_from_computer]=0;  //stop the buffer from transmitting leftovers or garbage.
	serial_event=1;
}

float get_pH (char command)
{

	if(serial_event)            //if the serial_event=1.
	{
		if(computerdata[0]=='c'||computerdata[0]=='r')time_=1800; //if a command has been sent to calibrate or take a reading we wait 1800ms so that the circuit has time to take the reading.
		else time_=300;         //if any other command has been sent we wait only 300ms.


		Wire.beginTransmission(address); //call the circuit by its ID number.
		Wire.write(computerdata);        //transmit the command that was sent through the serial port.
		Wire.endTransmission();          //end the I2C data transmission.


		delay(time_);                    //wait the correct amount of time for the circuit to complete its instruction.

		Wire.requestFrom(address,20,1); //call the circuit and request 20 bytes (this may be more than we need)
		code=Wire.read();               //the first byte is the response code, we read this separately.

		switch (code)                  //switch case based on what the response code is.
		{
			case 1:                       //decimal 1.
				Serial.println("Success");  //means the command was successful.
				break;                        //exits the switch case.

			case 2:                        //decimal 2.
				Serial.println("Failed");    //means the command has failed.
				break;                         //exits the switch case.

			case 254:                      //decimal 254.
				Serial.println("Pending");   //means the command has not yet been finished calculating.
				break;                         //exits the switch case.

			case 255:                      //decimal 255.
				Serial.println("No Data");   //means there is no further data to send.
				break;                         //exits the switch case.
		}

		while(Wire.available())          //are there bytes to receive.
		{
			in_char = Wire.read();           //receive a byte.
			ph_data[i]= in_char;             //load this byte into our array.
			i+=1;                            //incur the counter for the array element.
			if(in_char==0)                 //if we see that we have been sent a null command.
			{
				i=0;                        //reset the counter i to 0.
				Wire.endTransmission();     //end the I2C data transmission.
				break;                      //exit the while loop.
			}
		}

//		Serial.println(ph_data);          //print the data.
		serial_event=0;                   //reset the serial event flag.
	}

	//Uncomment this section if you want to take the pH value and convert it into floating point number.
	ph_float=atof(ph_data);
	return (ph_float);
}


//augur
boolean activeAugur = false;	//boolean for whether the augur is active or not
int augur_speed=0;
int augur_height_speed =0;
boolean servoSpin = false;       // Flag for sending servo command
boolean notReached = false;       // Flag for sending servo command
boolean upLimitSwitch = false;   //
boolean downLimitSwitch = false; //
boolean servoLimitSwitch = false;//
int servoPosCounter = 0;         //
int PWM3_val = 0;                //(25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM2_val = 0;                //(25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM1_val = 0;                //(25% = 64; 50% = 127; 75% = 191; 100% = 255)
int mapPWM = 0;

void activate_augur (boolean flag)
{
	activeAugur = flag;
}
void set_augur_speed(int speed)//sets the speed. positive for cw, negative for ccw. between -255 and +255, typically one extreme or the other
{
	augur_speed = speed;
}
void set_augur_height_speed (int speed)//sets the height of the augur, between -255 and +255
{
	augur_height_speed = speed;

}
void set_servo_switch(boolean flag) //sets the direction for servo rotation
{
	servoSpin = flag;
}

void setup()
{
	//things which have to be in the setup
	//barometer
	barometer.begin;
	//pH sensor
	Wire.begin();
	//augur
	pinMode(pin_augur_limit_switch_up,INPUT_PULLUP);
	pinMode(pin_augur_limit_switch_down,INPUT_PULLUP);
	pinMode(pin_augur_servo_limit_switch,INPUT_PULLUP);

	pinMode(pin_augur_in_a1, OUTPUT);
	pinMode(pin_augur_in_b1, OUTPUT);
	pinMode(pin_augur_pwm_1, OUTPUT);

	pinMode(pin_augur_in_a2, OUTPUT);
	pinMode(pin_augur_in_b2, OUTPUT);
	pinMode(pin_augur_pwm_2, OUTPUT);

	pinMode(pin_augur_in_a3, OUTPUT);
	pinMode(pin_augur_in_b3, OUTPUT);
	pinMode(pin_augur_pwm_3, OUTPUT);
}

void loop ()
{
	  upLimitSwitch = !digitalRead(pin_augur_limit_switch_up);
	  downLimitSwitch = !digitalRead(pin_augur_limit_switch_down);
	  servoLimitSwitch = !digitalRead(pin_augur_servo_limit_switch);


	  //sets the speed of the augur height motor, if the augur is not already at a limit position
	  if (activeAugur)
	  {
		  if (augur_height_speed > 0 && !upLimitSwitch)
		  {
			  digitalWrite (pin_augur_in_a1, HIGH);
			  digitalWrite (pin_augur_in_b1, LOW);
			  PWM1_val = augur_height_speed;
		  }
		  else if (augur_height_speed < 0 && !downLimitSwitch)
		  {
			  digitalWrite (pin_augur_in_a1, LOW);
			  digitalWrite (pin_augur_in_b1, HIGH);
			  PWM1_val = -1*augur_height_speed;
		  }
		  else
		  {
			  digitalWrite (pin_augur_in_a1, LOW);
			  digitalWrite (pin_augur_in_b1, LOW);
			  PWM1_val = augur_height_speed;
		  }
	  }
	  else
	  {
		  digitalWrite (pin_augur_in_a1, LOW);
		  digitalWrite (pin_augur_in_b1, LOW);
		  PWM1_val = 0;
	  }
	  analogWrite (pin_augur_pwm_1, PWM1_val);

	  //sets the speed of the augur
	  if (activeAugur)
	  {
		  if (augur_speed > 0)
		  {
			  digitalWrite (pin_augur_in_a2, HIGH);
			  digitalWrite (pin_augur_in_b2, LOW);
			  PWM2_val = augur_speed;
		  }
		  else if (augur_speed < 0)
		  {
			  digitalWrite (pin_augur_in_a2, LOW);
			  digitalWrite (pin_augur_in_b2, HIGH);
			  PWM2_val = -1*augur_speed;
		  }
		  else
		  {
			  digitalWrite (pin_augur_in_a2, LOW);
			  digitalWrite (pin_augur_in_b2, LOW);
			  PWM2_val = augur_speed;
		  }
	  }
	  else
	  {
		  digitalWrite (pin_augur_in_a2, LOW);
		  digitalWrite (pin_augur_in_b2, LOW);
		  PWM2_val = 0;
	  }
	  analogWrite (pin_augur_pwm_2, PWM2_val);

	  //servo
	  if (servoLimitSwitch) notReached = false;
	  if (servoSpin) notReached = true;
	  if (notReached)
	  {
		  PWM3_val = 25;
		  digitalWrite(pin_augur_in_a3,HIGH);
	      digitalWrite(pin_augur_in_b3,LOW);
	  }
	  else
	  {
		  PWM3_val = 0;
		  digitalWrite(pin_augur_in_a3,LOW);
		  digitalWrite(pin_augur_in_b3,LOW);
	  }
	  analogWrite(pin_augur_pwm_3, PWM3_val);
}
