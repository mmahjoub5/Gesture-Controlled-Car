
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>        
#include <WiFiNINA.h>
#include<WiFiUdp.h>

//https://www.youtube.com/watch?v=UmEPTh80KLc
//port 21567
//github test
unsigned int localPort = 6661;
char packetBuffer[255];
char  ReplyBuffer[15] = "";
int status = WL_IDLE_STATUS;
char ssid[] = "ATT8T56pbr 2.4ghz"; //WiFi Network Name
char pass[] = "7emn9+bq9kab"; //WiFi Network Password
int keyIndex = 0; //do not need
char message[] = "hello world";
IPAddress ip(192,168,1,106); 
WiFiUDP Udp;
WiFiClient client;




// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

int x_acc = 0;
int y_acc = 0;

void setup(void) {
  Serial.begin(9600);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  pinMode (A1, INPUT);
  
  if (!lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  //set range
  lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!

  if (WiFi.status() == WL_NO_MODULE) 
  {
    Serial.println("WiFi shield not present");
    while (true);
  }

  // Connect to WiFi
while ( status != WL_CONNECTED) 
  {
     Serial.print("Attempting to connect to SSID: ");
     Serial.println(ssid);
     status = WiFi.begin(ssid, pass);
  //Wait for connection
     delay(10000);

       //if connection made then report back via serial 

  }

   String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
    //client.connect(ip, 6661);
    //Serial.println(client.connect(ip, 6661));
   
  // if you get a connection, report back via serial:
   if (client.connect(ip, 6661)) {
      Serial.println("connected");
       //Make a HTTP request:
     client.println("GET /search?q=arduino HTTP/1.0");
     client.println();
    }
    else{
         Serial.print("not connected");
      
    }
    
  Serial.println();
  Serial.println("WiFi Status  ");
  printWiFiStatus();

  Serial.println("\n Starting Listener");

  //if connection made then report back via serial 
 Udp.begin(localPort); 

     
  
 
}

void loop() {
 read_data(x_acc,y_acc);
  process();



   
  //send_data(ReplyBuffer);
  Udp.beginPacket(ip, localPort);
  Udp.write(ReplyBuffer);
  Udp.endPacket();
  Serial.println("package sent");
  
  delay(200);
 
}




//this function reads in data from accel and is called when the hand is closed 
void read_data(int &x_acc,int &y_acc){
  //read in data 
  lis.read();      // get X Y and Z data at once

   /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);

  x_acc = event.acceleration.x;
  y_acc = event.acceleration.y;
  
  
 
  delay(200); 

}

//create state machine see which way to go and copy that state onto state array 
void process(){
   
   //hand is leaning forward 
   if (x_acc <= 10 && x_acc > 0){
    if (y_acc < 2 && y_acc > -2){
      strcpy(ReplyBuffer, "Forward");
      Serial.println(ReplyBuffer);
      //implement if we want to include foreward & right and  foreward & left 
      /*if (y > 2 & y < 10){
        strcpy(ReplyBuffer, 
     }*/
    }
   }
   //hand is leaning backword 
   if (x_acc < 0 && x_acc >= -10){
    if (y_acc < 1 && y_acc > -1){
      strcpy(ReplyBuffer, "Backword");
      Serial.println(ReplyBuffer);
    }
   }

   //hand is is leaning to the left 
   if (y_acc <= 10 && y_acc > 0){
    if (x_acc < 2 && x_acc > -2){
      strcpy(ReplyBuffer, "Left");
      Serial.println(ReplyBuffer);
    } 
   }
   
   //hand is is leaning to the right 
   if (y_acc < 0 && y_acc >= -10){
      if (x_acc < 2 && x_acc > -2){
      strcpy(ReplyBuffer, "Right");
      Serial.println(ReplyBuffer);
      }
   }

   else if (x_acc < 2.0 & y_acc < 2.0){
    if(x_acc > -2 && y_acc > -2){
      strcpy(ReplyBuffer, "Home");
      Serial.println(ReplyBuffer);
    
    }
       
   }

   delay(200); 

}

///sends data via UDP
/*void send_data(char x[]){
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(x);
  Udp.endPacket();

}*/


void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
