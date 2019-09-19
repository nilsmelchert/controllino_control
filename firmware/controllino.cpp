#include <Controllino.h> /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
 
#include <SPI.h>
#include <Ethernet.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>

// ROS initialization
ros::NodeHandle  nh;
using rosserial_arduino::Test;
int ret_val;

// Shield settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(130, 75, 27, 117);

// Server settings
IPAddress server(130, 75, 27, 242);
uint16_t serverPort = 11411;

const uint8_t ledPin = 7; // 13 already used for SPI connection with the shield

void callback(const Test::Request & req, Test::Response & res){
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
  ///< LED-D0
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
  ///< LED-D1
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
  ///< LED-D2
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
  ///< LED-D3
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
  ///< LED-D4
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
  ///< LED-D5
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
  ///< LED-D6
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
  ///< LED-D7
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

ros::ServiceServer<Test::Request, Test::Response> service_server("test_srv",&callback);

void setup()
{
  // Define outputs
  pinMode(CONTROLLINO_R0, OUTPUT);
  pinMode(CONTROLLINO_D0, OUTPUT); 
  pinMode(CONTROLLINO_D1, OUTPUT);
  pinMode(CONTROLLINO_D2, OUTPUT); 
  pinMode(CONTROLLINO_D3, OUTPUT); 
  pinMode(CONTROLLINO_D4, OUTPUT); 
  pinMode(CONTROLLINO_D5, OUTPUT); 
  pinMode(CONTROLLINO_D6, OUTPUT);
  pinMode(CONTROLLINO_D7, OUTPUT);

  // Define inputs
  pinMode(CONTROLLINO_D8, INPUT); 
  pinMode(CONTROLLINO_D9, INPUT);
  pinMode(CONTROLLINO_D10, INPUT); 
  pinMode(CONTROLLINO_D11, INPUT); 
  pinMode(CONTROLLINO_D12, INPUT); 
  pinMode(CONTROLLINO_D13, INPUT); 
  pinMode(CONTROLLINO_D14, INPUT);
  pinMode(CONTROLLINO_D15, INPUT);
  
  Ethernet.begin(mac, ip);
  // give the Ethernet shield a second to initialize:
  delay(1000);
  pinMode(ledPin, OUTPUT);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertiseService(service_server);  
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

