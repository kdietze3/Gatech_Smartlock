/*
 Gatech SmartLock Controller
 
 Copyright (c) 2013 Kevin Dietze.
 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 
 TODO
 Wifi - Done
 Bluetooth - Done
 NFC
 Camera
 iOS and Android Apps
 Eletronic Door Strike
 
 Current Accepted Commands
  * http://arduino_ip/?OPEN
  * http://arduino_ip/?LOCK
 
 Circuit:
 * WiFi shield attached
 * LED attached to pin 9
 * Bluetooth TX to pin 0
 * Bluetooth RX to pin 1
 
 ******************************************
 SUPER IMPORTANT!!!!!!

  In order to upload the Arduino sketch you must first disconnect pin 0
   on the Arduino circuit. It will fail every time if you dont.
  
  ******************************
  
  */
#include <SPI.h>
#include <WiFi.h>

#define SPKR 5 //Digital pin for the PCS Speaker
#define LED 9 //Digital pin for the LED
#define LOCK 13 //Digital pin for the Strike Lock

int PIN_POT = A0;

// led and potentiometer
int led;
int pot;

char ssid[] = "Smart";      //  your network SSID (name) 
char pass[] = "oxford2012";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
boolean reading = false;
String command = "";

int status = WL_IDLE_STATUS;
WiFiServer server(80);

void beep(unsigned char delayms){
  analogWrite(SPKR, 20);      // Almost any value can be used except 0 and 255
                           // experiment to get the best tone
  delay(delayms);          // wait for a delayms ms
  analogWrite(SPKR, 0);       // 0 turns it off
  delay(delayms);          // wait for a delayms ms   
}

void beepMultiple(unsigned int numTimes, unsigned char delayms){
 for (int i = 0; i < numTimes; i++){
   beep(delayms); 
 }
 
 
  
}

void setup() {
  Serial.begin(115200);      // initialize serial communication for Android
  pinMode(9, OUTPUT);      // set the LED pin mode
  pinMode(SPKR, OUTPUT);
  pinMode(LOCK, OUTPUT);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    while(true);        // don't continue
  } 

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
    status = WiFi.begin(ssid, pass);
    // wait 4 seconds for connection:
    
    delay(4000);
  } 
  server.begin();                           // start the web server on port 80
  //printWifiStatus();                        // you're connected now, so print out the status
  beepMultiple(3,50);
}

//Arduino version of main()
void loop() {
  wifiFullLineCommands();
  bluetoothCommands();
}

void bluetoothCommands(){
  

  if(Serial.available()>0)
  {
    // Serial.read() reads one byte
    led = Serial.read();
    
  }
 
 if(led == 1){
    executeCommand("LOCK");
    led = 0;
 }
  else if (led == 2){
    executeCommand("OPEN");
    led = 0;
  }
}

//Execute recieved command. Currently in if/elseif form because I am unsure if
//Arduino can switch on Strings.
void executeCommand(String command){
 
  if(command == "OPEN"){
    unlock();
  }
  else if(command == "LOCK"){
    lock();
  }
}


//Lock the door and output and play debug codes
void lock(){
   //Serial.println("Locking door");
   digitalWrite(LED,HIGH);
   Serial.write(1);
   digitalWrite(LOCK,LOW);
   beepMultiple(2,300);
   
}

//Unlock the door and output and show debug codes
void unlock(){
  //Serial.println("Unlocking door");
  digitalWrite(LED,LOW);
  digitalWrite(LOCK,HIGH);
  Serial.write(2);
  beepMultiple(2,300);
}


void wifiFullLineCommands(){
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {

    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    boolean sentHeader = false;
    beepMultiple(5,100);
    
    while (client.connected()) {
      if (client.available()) {

        if(!sentHeader){
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println();
          sentHeader = true;
        }

        char c = client.read();

        if(reading && c == ' ') reading = false;
        if(c == '?') reading = true; //found the ?, begin reading the info

        if(reading){
          //Serial.print(c);
          if(c != '?'){
           command += c;
          }
           

        }

        if (c == '\n' && currentLineIsBlank)  break;

        if (c == '\n') {
          currentLineIsBlank = true;
        }else if (c != '\r') {
          currentLineIsBlank = false;
        }

      }
    }
    client.println(command);
    
    //Execute recieved command
    executeCommand(command);
    //Reset Command to an empty string
    command = "";
    
    delay(1); // give the web browser time to receive the data
    client.stop(); // close the connection:

  } 
  
}

void triggerPin(int pin, WiFiClient client){
//blink a pin - Client needed just for HTML output purposes.  
  client.print("Turning on pin ");
  client.println(pin);
  client.print("<br>");

  digitalWrite(pin, HIGH);
  delay(25);
  digitalWrite(pin, LOW);
  delay(25);
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
