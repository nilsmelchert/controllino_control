/*
	Sketch title

	This arduino sketch lets you communicate with the controllino module, 
	which is based on the arduino meda 2560 via ros services.
	In order to use more than just the default interrupt pins, the library
	"PinChangeInterrupt" from Github was used. 
	(see https://github.com/NicoHood/PinChangeInterrupt)

	Created 11 11 2019
	By Nils Melchert
*/

#include <Controllino.h> 
#include <SPI.h>
#include <Ethernet.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>
#include <PinChangeInterrupt.h>

// ROS initialization
ros::NodeHandle  nh;
using rosserial_arduino::Test;
int ret_val;

ros::ServiceClient<Test::Request, Test::Response> service_client("controllino_getter");

// Shield settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(130, 75, 27, 117);

// Server settings
IPAddress server(130, 75, 27, 242);
uint16_t serverPort = 11411;

Test::Request test_req;
Test::Response test_resp;

void service_callback(const Test::Request & req, Test::Response & res){
  ///< ALL ON
  if (String(req.input).equals("da1"))
  {                
    digitalWrite(CONTROLLINO_D0, HIGH);
    digitalWrite(CONTROLLINO_D1, HIGH);
    digitalWrite(CONTROLLINO_D2, HIGH);
    digitalWrite(CONTROLLINO_D3, HIGH);
    digitalWrite(CONTROLLINO_D4, HIGH);
    digitalWrite(CONTROLLINO_D5, HIGH);
    digitalWrite(CONTROLLINO_D6, HIGH);
    digitalWrite(CONTROLLINO_D7, HIGH);
    res.output = "1";
  }  
  ///< ALL OFF
  else if (String(req.input).equals("da0"))
  {   
    digitalWrite(CONTROLLINO_D0, LOW);
    digitalWrite(CONTROLLINO_D1, LOW);
    digitalWrite(CONTROLLINO_D2, LOW);
    digitalWrite(CONTROLLINO_D3, LOW);
    digitalWrite(CONTROLLINO_D4, LOW);
    digitalWrite(CONTROLLINO_D5, LOW);
    digitalWrite(CONTROLLINO_D6, LOW);
    digitalWrite(CONTROLLINO_D7, LOW);
    res.output = "0";
  }
  ///< LED-d0
  else if (String(req.input).equals("d01"))
  {    
    digitalWrite(CONTROLLINO_D0, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d00"))
  {    
    digitalWrite(CONTROLLINO_D0, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d0?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D0);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  ///< LED-d1
  else if (String(req.input).equals("d11"))
  {    
    digitalWrite(CONTROLLINO_D1, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d10"))
  {    
    digitalWrite(CONTROLLINO_D1, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d1?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D1);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  ///< LED-d2
  else if (String(req.input).equals("d21"))
  {    
    digitalWrite(CONTROLLINO_D2, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d20"))
  {    
    digitalWrite(CONTROLLINO_D2, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d2?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D2);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  ///< LED-d3
  else if (String(req.input).equals("d31"))
  {    
    digitalWrite(CONTROLLINO_D3, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d30"))
  {    
    digitalWrite(CONTROLLINO_D3, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d3?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D3);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  ///< LED-d4
  else if (String(req.input).equals("d41"))
  {    
    digitalWrite(CONTROLLINO_D4, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d40"))
  {    
    digitalWrite(CONTROLLINO_D4, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d4?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D4);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  ///< LED-d5
  else if (String(req.input).equals("d51"))
  {    
    digitalWrite(CONTROLLINO_D5, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d50"))
  {    
    digitalWrite(CONTROLLINO_D5, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d5?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D5);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  ///< LED-d6
  else if (String(req.input).equals("d61"))
  {    
    digitalWrite(CONTROLLINO_D6, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d60"))
  {    
    digitalWrite(CONTROLLINO_D6, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d6?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D6);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  ///< LED-d7
  else if (String(req.input).equals("d71"))
  {    
    digitalWrite(CONTROLLINO_D7, HIGH);  
    res.output = "1";          
  }
  else if (String(req.input).equals("d70"))
  {    
    digitalWrite(CONTROLLINO_D7, LOW);
    res.output = "0";
  }  
  else if (String(req.input).equals("d7?"))
  {    
    ret_val = digitalRead(CONTROLLINO_D7);
    if (ret_val==1)
    {
      res.output = "1";    
    }
    else if (ret_val==0)
    {
      res.output = "0";    
    }
    else 
    {
      res.output = "-1";    
    }
  }
  else
  {
      res.output = "-1";
  }
}

ros::ServiceServer<Test::Request, Test::Response> service_server("controllino_setter",&service_callback); 

void setup()
{  
  // Define outputs    
  pinMode(CONTROLLINO_D0, OUTPUT);
  pinMode(CONTROLLINO_D1, OUTPUT);
  pinMode(CONTROLLINO_D2, OUTPUT); 
  pinMode(CONTROLLINO_D3, OUTPUT); 
  pinMode(CONTROLLINO_D4, OUTPUT); 
  pinMode(CONTROLLINO_D5, OUTPUT);
  pinMode(CONTROLLINO_D6, OUTPUT);
  pinMode(CONTROLLINO_D7, OUTPUT);  

  //const byte interruptPin = 2;
  //attachInterrupt(digitalPinToInterrupt(interruptPin), test_ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A8), isr8, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A9), isr9, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A10), isr10, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A11), isr11, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A12), isr12, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A13), isr13, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A14), isr14, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CONTROLLINO_A15), isr15, CHANGE);
    
  Ethernet.begin(mac, ip);
  // give the Ethernet shield a second to initialize:
  delay(1000);  
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertiseService(service_server);  
  nh.serviceClient(service_client);
  while(!nh.connected()) nh.spinOnce();
}

static unsigned long last_interrupt_time_8 = 0;
void isr8()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_8 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A8));
        if(trigger == RISING)
          test_req.input = "a81";
        else if(trigger == FALLING)
          test_req.input = "a80";
        else 
          test_req.input = "a8-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_8 = interrupt_time;
  }
static unsigned long last_interrupt_time_9 = 0;
void isr9()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_9 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A9));
        if(trigger == RISING)
          test_req.input = "a91";
        else if(trigger == FALLING)
          test_req.input = "a90";
        else 
          test_req.input = "a9-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_9 = interrupt_time;
  }
static unsigned long last_interrupt_time_10 = 0;
void isr10()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_10 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A10));
        if(trigger == RISING)
          test_req.input = "a101";
        else if(trigger == FALLING)
          test_req.input = "a100";
        else 
          test_req.input = "a10-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_10 = interrupt_time;
  }
static unsigned long last_interrupt_time_11 = 0;
void isr11()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_11 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A11));
        if(trigger == RISING)
          test_req.input = "a111";
        else if(trigger == FALLING)
          test_req.input = "a110";
        else 
          test_req.input = "a11-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_11 = interrupt_time;
  }
static unsigned long last_interrupt_time_12 = 0;
void isr12()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_12 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A12));
        if(trigger == RISING)
          test_req.input = "a121";
        else if(trigger == FALLING)
          test_req.input = "a120";
        else 
          test_req.input = "a12-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_12 = interrupt_time;
  }
static unsigned long last_interrupt_time_13 = 0;
void isr13()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_13 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A13));
        if(trigger == RISING)
          test_req.input = "a131";
        else if(trigger == FALLING)
          test_req.input = "a130";
        else 
          test_req.input = "a13-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_13 = interrupt_time;
  }
static unsigned long last_interrupt_time_14 = 0;
void isr14()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_14 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A14));
        if(trigger == RISING)
          test_req.input = "a141";
        else if(trigger == FALLING)
          test_req.input = "a140";
        else 
          test_req.input = "a14-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_14 = interrupt_time;
  }
static unsigned long last_interrupt_time_15 = 0;
void isr15()
  {
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time_15 > 200)
      {
        char cstr[16];
        uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPinChangeInterrupt(CONTROLLINO_A15));
        if(trigger == RISING)
          test_req.input = "a151";
        else if(trigger == FALLING)
          test_req.input = "a150";
        else 
          test_req.input = "a15-1";
        service_client.call(test_req, test_resp);
      }
      last_interrupt_time_15 = interrupt_time;
  }

void loop()
{
  nh.spinOnce();
  delay(1);
}
