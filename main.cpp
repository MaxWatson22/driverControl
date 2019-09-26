#include "mbed.h"
#include "CANMsg.h"
#include "Adafruit_LEDBackpack.h"

Ticker ticker;
Ticker ticker2;
Timer timer;
AnalogIn currentPot(p15);
float curr_val = 0;
float curr_reading = 0;
AnalogIn speedPot(p16);
float speed_val = 0;
float speed_reading = 0;
float speed_reading_ms = 0;
float speed_reading_kmh = 0;
DigitalIn ignition(p21);
DigitalIn regen(p22);
DigitalIn rev(p23);
DigitalIn brake(p24);
DigitalIn accel(p25);
float maxBusCurrent = 1.0;
float actualSpeedms = 0;
float actualSpeedrpm = 0;
float actualSpeedkmh = 0;

I2C i2c(p28,p27);
Adafruit_LEDBackpack display(&i2c);

DigitalOut debug1(LED1);
DigitalOut debug2(LED2);
DigitalOut debug3(LED3);
DigitalOut debug4(LED4);

//CAN can1(p9, p10);
CAN can1(p30, p29);

const uint16_t SIGNAL_ID = 0x501;
const uint16_t BUS_ID = 0x502;
const uint16_t TRITIUM_ID = 0x603;
const uint8_t DISPLAY_ADDR = 0x70; 
char counter = 0;
uint8_t dig1 = 0;
uint8_t dig2 = 0;

Serial pc(USBTX, USBRX); // tx, rx

CANMsg driverControls;
CANMsg busCurrent;
CANMsg msgReceived;

//====================================================================

void printMsg(CANMessage& msg)
{
    pc.printf("-------------------------------------\r\n");
    pc.printf("  ID      = 0x%.3x\r\n", msg.id);
    pc.printf("  Type    = %d\r\n", msg.type);
    pc.printf("  Format  = %d\r\n", msg.format);
    pc.printf("  Length  = %d\r\n", msg.len);
    pc.printf("  Data    =");
    pc.printf("-------------------------------------\r\n");
    for(int i = 0; i < msg.len; i++)
        pc.printf(" 0x%.2X", msg.data[i]);
    pc.printf("\r\n");
}

//=====================================================================

void onCanReceived()
{
    can1.read(msgReceived);
//    pc.printf("-------------------------------------\r\n");
//    pc.printf("CAN message received\r\n");
//    pc.printf("-------------------------------------\r\n");
    //printMsg(msgReceived);
    //wait(1);
    if (msgReceived.id == TRITIUM_ID) {
        // extract data from the received CAN message 
        // in the same order as it was added on the transmitter side
           
//        pc.printf("  counter = %d\r\n", counter);
//        pc.printf("  voltage = %e V\r\n", voltage);
        debug3 = !debug3;
        //printMsg(msgReceived);
        msgReceived >> actualSpeedrpm;
        //pc.printf("speed in m/s %f\r\n", actualSpeedms);
        msgReceived >> actualSpeedms; 
    }
    //timer.start(); // to transmit next message in main
}

//======================================================================

void displayDigit(uint8_t digit, uint8_t position)
{
    //    if (position > 4)
//    {}
    //printf("digit = %d\t position = %d\r\n", digit, position);
    if (digit > 9) {
        display.displaybuffer[position] = 0;
    } else if (digit == 0) {
        display.displaybuffer[position] = 63;
    } else if (digit == 1) {
        display.displaybuffer[position] = 48;
    } else if (digit == 2) {
        display.displaybuffer[position] = 91;
    } else if (digit == 3) {
        display.displaybuffer[position] = 79;
    } else if (digit == 4) {
        display.displaybuffer[position] = 102;
    } else if (digit == 5) {
        display.displaybuffer[position] = 109;
    } else if (digit == 6) {
        display.displaybuffer[position] = 125;
    } else if (digit == 7) {
        display.displaybuffer[position] = 7;
    } else if (digit == 8) {
        display.displaybuffer[position] = 127;
    } else if (digit == 9) {
        display.displaybuffer[position] = 103;
    }
    //timer.start();
    display.writeDisplay();
    //timer.stop();
    //printf("Time to writeDisplay() is %dus\r\n", timer.read_us());
    //timer.reset();
}

void displayNumber(uint8_t number, bool position)
{
    //timer.start();
    for (int n = 0; n < 100; (n = n + 10))
    {
        if (number >= n && number < n + 10)
        {
            dig1 = n/10;
            dig2 = number - n;
        }
    }
    
    if (number > 99)
    {dig1 = 0;
    dig2 = 0;}

    if (position) 
    {
        displayDigit(dig1, 3);
        displayDigit(dig2, 4);
    } 
    else if (!position) 
    {
        displayDigit(dig1, 0);
        displayDigit(dig2, 1);
    }
    //timer.stop();
    //printf("time taken to print number %d is %dus\r\n", number, timer.read_us());
    //timer.reset();
}

void setDriverControls()
{
    driverControls.clear();
    driverControls.id = SIGNAL_ID;
    driverControls << speed_val;
    driverControls << curr_val;
    
    busCurrent.clear();
    busCurrent.id = BUS_ID;
    busCurrent << 0;
    busCurrent << maxBusCurrent;
}

void sendCAN()
{
    setDriverControls();
    //printMsg(driverControls);
    //printMsg(busCurrent);
    if(can1.write(driverControls))
    {
        debug1 = !debug1;
    }
    else
    {
        //wait(0.5);
        
        
    }
    
    if(can1.write(busCurrent))
    {
        debug2 = !debug2;
    }
    else
    {
        //wait(0.5);
        
    }
}

void dispSpeed()
{
    actualSpeedkmh = abs(actualSpeedms) * 3.6;
    displayNumber( (uint8_t) actualSpeedkmh , 1);
    displayNumber( (uint8_t) speed_reading_kmh , 0);
}

int main()
{
    msgReceived = CANMsg();
    pc.baud(115200);
    can1.frequency(1000000);
    ticker.attach(&sendCAN, 0.1);
    ticker2.attach(&dispSpeed, 0.5);
    can1.attach(&onCanReceived); 
    ignition.mode(PullUp);
    regen.mode(PullUp);
    rev.mode(PullUp);
    brake.mode(PullUp);
    accel.mode(PullUp);
    display.begin(DISPLAY_ADDR);
    display.setBrightness(10);
    
    //pc.printf("-------------------------------------\r\n");
    //printf("Attempting to send a CAN message\n");
    //printf("Current and Speed: %f and %f\n", currentPot.read(), speedPot.read());
    //pc.printf("-------------------------------------\r\n");
    
    while(1) {
        
        //curr_val = 0.2;
        //speed_val = 5.0;
        
        curr_reading = currentPot.read();
        speed_reading = speedPot.read();
        speed_reading_ms = speed_reading * 30;
        speed_reading_kmh = speed_reading_ms*3.6;
        
        //displayNumber(1,1);
        //displayNumber(1,0);

        
        bool ignition_reading = ignition.read();
        bool regen_reading = regen.read();
        bool rev_reading = rev.read();
        bool brake_reading = brake.read();
        bool accel_reading = accel.read();
        
        //pc.printf("Current reading: %f\r\n", curr_reading);
        //pc.printf("Speed reading: %f\r\n", speed_reading);
        //pc.printf("Ignition reading: %d\r\n", ignition_reading);
        //pc.printf("Regen reading: %d\r\n", regen_reading);
        //pc.printf("Forward/reverse reading: %d\r\n", rev_reading);
        //pc.printf("Brake reading: %d\r\n", brake_reading);
        
        //pc.printf("Accelerator reading: %d\r\n\r\n", accel_reading);
        //wait(2);
        
        if (ignition)
        {
            curr_val = 0;
            speed_val = 0;
        }
        else
        {
            if (!brake && !regen)
            {
                speed_val = 0;
                curr_val = curr_reading;
            }
            else if(!brake && regen)
            {
                speed_val = 0;
                curr_val = 0;
            }
            else
            {
                if (rev)
                {
                    speed_val = (-1)*speed_reading_ms;
                }
                else
                {
                    speed_val = speed_reading_ms;
                }
                
                if (!accel)
                {
                    curr_val = curr_reading;
                }
                else
                {
                    curr_val = 0;
                }
             }
            
        }
        //pc.printf("speed_val: %f\r\n", speed_val);
        //pc.printf("curr_val: %f\r\n\r\n", curr_val);
        //sendCAN();
        //pc.printf("\r\n\r\n");
        //wait(3);
    }
}