#include "mbed.h"

//  ***   BE SURE TO TEST BEFORE AND AFTER FOR 'IDLING'... CLEAR THE CCD EVERY OTHER FRAME TO TRY AND ELIMINATE NOISE BUILDUP
//From http://www.ing.iac.es/~docs/ins/das/ins-das-29/integration.html
//"Idling" vs. integrating
//Charge from light leaks and thermal noise builds up on the detector between observations. If this charge is integrated by the detector, it may not be completely
//cleared away by the clear cycle of the next observation. In that case, the observation will be contaminated by extra counts. (Often, this appears as a ramp in the
//background leading up to a saturated region in the low-numbered rows.)
//To avoid this problem, the detector is made to clear itself continuously between observations. This is called "idling", and is reported as such on the mimic.

Serial raspi(SERIAL_TX, SERIAL_RX);

PwmOut masterClock(PB_4);
PwmOut shiftGate(PB_8);
DigitalOut ICG(PB_3);
AnalogIn imageIn(A0);
DigitalOut LED(LED1);
DigitalOut illuminator(PA_8);
DigitalOut blueLED(PB_5);
DigitalOut redLED(PB_10);
DigitalIn button (PC_13);




float firmwareVersion = 0.2;


int masterFreq_period       = 8;    //microseconds
int masterFreq_width        = 4;    //microseconds

int shiftGate_period        = 16;    //microseconds
int shiftGate_width         = 8;    //microseconds

int none        = 0;
int veryLow     = 10;
int low         = 100;
int mediumLow   = 1000;
int medium      = 100000;
int high        = 5000000;
int veryHigh    = 10000000;

double imageData;
int sensitivity;
int pixelTotal              = 3694;
int leadingDummyElements    = 16;
int leadShieldedElements    = 13;
int headerElements          = 3;
const int signalElements    = 3648;
int trailingDummyElements   = 14;
int pixelCount;
int readOutTrigger;
int state;

#define readOut_Begin               1
#define readOut_ACTIVE              2
#define readOut_LeadingDummy        3
#define readOut_LeadingShielded     4
#define readOut_headerElements      5
#define readOut_signalElements      6
#define readOut_trailingDummy       7
#define readOut_integrationTime     8
#define readOut_IDLE                9
#define readOut_Finish              0

#define MV(x) ((0xFFF*x)/3300)

double pixelValue[signalElements] = { 0 };



void checkState()
{
//    raspi.printf("|");
    switch (state) {
        case readOut_Begin:
//            raspi.printf("readout begin");
            readOutTrigger = 1;
            state = readOut_ACTIVE;
//                state = readOut_signalElements;
//            raspi.printf("+++\r\n");
            LED = 1;
            break;
        case readOut_ACTIVE:
//            raspi.printf("readout active");
            ICG = 1;
            state = readOut_LeadingDummy;
            break;
        case readOut_LeadingDummy:
//            raspi.printf("readout active");
            pixelCount++;
            if (pixelCount >= leadingDummyElements - 1) {
                pixelCount = 0;
                state = readOut_LeadingShielded;
            }
            break;
        case readOut_LeadingShielded:
//            raspi.printf("readout shielded");
            pixelCount++;
            if (pixelCount >= leadShieldedElements - 1) {
                pixelCount = 0;
                state = readOut_headerElements;
            }
            break;
        case readOut_headerElements:
//            raspi.printf("readout header");
            pixelCount++;
            if (pixelCount == headerElements - 1) {
                pixelCount = 0;
                state = readOut_signalElements;
            }
            break;
        case readOut_signalElements:
 //           raspi.printf("readout signal");
            pixelCount++;
 //           wait_us(1);
            raspi.printf("%i  %f\n", pixelCount, imageIn.read());
 //           pixelValue[pixelCount] = imageIn.read();
 //           raspi.printf("%d",pixelValue[pixelCount]);
 //           raspi.printf("%i\t%4.12f\r\n", pixelCount,(imageIn.read_u16() * 5.0) / 4096.0);
            if (pixelCount >= signalElements - 1) {
                pixelCount = 0;
                state = readOut_trailingDummy;
            }
            break;
        case readOut_trailingDummy:
 //           raspi.printf("readout trailing");
            pixelCount++;
            if (pixelCount == trailingDummyElements - 1) {
                pixelCount = 0;
                state = readOut_integrationTime;
                ICG = 0;
            }
            break;
        case readOut_integrationTime:
//            raspi.printf("readout integration");
            wait_us(sensitivity);
            state = readOut_Finish;
//            for (int pixelNumber=0; pixelNumber<signalElements; pixelNumber++) {
//                raspi.printf("%i\t%4.12f\r\n", pixelNumber, 5 - ((pixelValue[pixelNumber - 1] * 5) / 4096.0));
//            }
//            raspi.printf("---\r\n");
            break;
        case readOut_Finish:
//            raspi.printf("readout finished");
            redLED = 0;
            state = readOut_IDLE;
            wait_us(sensitivity);
            LED = 0;
            illuminator = 0;
            ICG = 1;
            break;
        case readOut_IDLE:
//            raspi.printf("idle");
            if (ICG == 1) {
//                ICG = 0;
//                state = readOut_Begin;
                blueLED = 1;
            }
            break;
        default:
            break;
    }
}

void printInfo(){
    raspi.printf("meridianScientific\r\n");
    raspi.printf("ramanPi - The DIY 3D Printable RaspberryPi Raman Spectrometer\r\n");
    raspi.printf("Spectrometer imagingBoard\r\n");
    raspi.printf("-------------------------------------------------------------\r\n");
    raspi.printf("Firmware Version: %f\r\n",firmwareVersion);
    raspi.printf("Current Sensitivity: %d\r\n", sensitivity);
}

int main()
{
    ICG = 1;
    LED = 0;
    blueLED = 0;
    redLED = 0;
    pixelCount = 0;
    readOutTrigger = 0;
    state = readOut_IDLE;

    masterClock.period_us(masterFreq_period);
    masterClock.pulsewidth_us(masterFreq_width);
//    masterClock.write (masterFreq_duty);

    shiftGate.period_us(shiftGate_period);
    shiftGate.pulsewidth_us(shiftGate_width);

    raspi.baud(921600);
    wait(0.5);

    raspi.baud(921600);
    blueLED = 0;
    sensitivity = low;
                

    printInfo();
    while (true)
    {
        if (button==0)
        
        {
            ICG = 0;
            state = readOut_Begin;
            while (state != readOut_IDLE)
            {
//             raspi.printf("%d", jj++);
                checkState();
            }
        }
    }
}