#include <Arduino_APDS9960.h>
#include <arm_math.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#define ARM_MATH_CM4

// June 2022
// Code to make a smarter and somewhat autonomous Droid from Galaxy's Edge brought from the Droid Depot
// This code is for Nano BLE 33 Sense
// Some code is from https://github.com/arduino/ArduinoAI
// Some code is from https://forum.arduino.cc/u/gssd/summary
// Using Arduino IDE 2.0.0-rc7 or 2.2.1
// User ArduinoBLE 1.2.2 (crashes sometimes) or 1.3.6

 #define RED 22     
 #define BLUE 24     
 #define GREEN 23
 #define LED_PWR 25
const int buttonPin = 4; // set buttonPin to digital pin 4; 7th from bottom on right side
// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
arm_rfft_instance_q15 FFT;
int bytesAvailable;

// Number of audio samples read
volatile int samplesRead;
volatile bool paired;
volatile bool droidFlag;
volatile bool fifthPastSample; //five audio samples ago
volatile bool fourthPastSample;
volatile bool thirdPastSample;
volatile bool secondPastSample;
volatile bool firstPastSample;
volatile int humanDelay;
volatile int loopCount;
volatile boolean loudFlag;  
volatile boolean accelFlag;

static char firstCommand[] = {0x22,0x20,0x01};  
static char secondCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x00};
static char thirdCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x02}; 
static char fourthCommand[] = {0x29,0x42,0x05,0x46,0x00,0x70,0x01,0x2c,0x00,0x00}; //0first motor forwards half power
static char fifthCommand[] = {0x29,0x42,0x05,0x46,0x00,0x00,0x01,0x2c,0x00,0x00};  //stop motor  
static char sixthCommand[] = {0x25,0x00,0x0c,0x42,0x08,0x02}; //rotate head
static char seventhCommand[] = {0x29,0x42,0x05,0x46,0x01,0x70,0x01,0x2c,0x00,0x00}; //second motor 0x46,0x01 is passenger leg
static char eighthCommand[] = {0x29,0x42,0x05,0x46,0x01,0x00,0x01,0x2c,0x00,0x00}; //stop motor
static char ninthCommand[] = {0x29,0x42,0x05,0x46,0x80,0x90,0x01,0x2c,0x00,0x00}; 
static char tenthCommand[] = {0x29,0x42,0x05,0x46,0x81,0x90,0x01,0x2c,0x00,0x00}; 
static char eleventhCommand[] = {0x29,0x42,0x05,0x46,0x80,0x70,0x01,0x2c,0x00,0x00}; //0first motor backwards half power
static char twelthCommand[] = {0x29,0x42,0x05,0x46,0x81,0x70,0x01,0x2c,0x00,0x00}; //second motor backwards half power

static char firstBank[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x00};  
static char secondBank[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x01}; 
static char thirdBank[] =  {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x02}; 
static char fourthBank[] =  {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x03}; 
static char fifthBank[] =  {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x04};
static char firstSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x00};  
static char secondSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x01};     
static char thirdSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x02};   
static char fourthSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x03};     
static char fifthSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x04};  
static char sixthSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x05}; 

BLEDevice peripheral;  
BLECharacteristic droidCharacteristic;
BLECharacteristic notifyCharacteristic;

int apds[5]; //consider boolean instead of int
int speech[6];

void setup() {
  droidFlag = false;
  humanDelay=0; //count delay or pause in speech
  loopCount=0;
 // intitialize the digital Pin as an output
  paired = false; 
  speech[0]=0; //array of when human speech was heard over 6 samples
  speech[1]=0;
  speech[2]=0;
  speech[3]=0;
  speech[4]=0;
  speech[5]=0;

  apds[0]=0; //array to hold proximity data over 5 samples
  apds[1]=0;
  apds[2]=0;
  apds[3]=0;
  apds[4]=0;
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  lightsOn(1);   
  delay(5000);
  lightsOn(0);
    // begin initialization
  Serial.begin(9600);    // initialize serial communication
  //initialize accelerometer
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor.");
    }
  
   // Set LED output to medium power.
   APDS.setLEDBoost(3);
  //increase microphone sensitivity  
  PDM.setGain(30);  
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);
  //start blue tooth low energy
  startBLE();  
  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    //while (1);
  }  
  delay(5000);
  loudFlag=false;
  accelFlag=false;
  long OnTime = 250;           // milliseconds of on-time
  long OffTime = 750;          // milliseconds of off-time
}

// the loop function runs over and over again
void loop() {
  //initialize variables
  byte notifyValue;  
  boolean heyWord = false;
  boolean heyWordTwice = false;
  boolean forwardWord = false;
  float currentAcceleration;
  String macaddr = String("00:00:00:00:00:00"); //03:04:44:00:00:00
  String droidmacaddrprefix = String("f1:f2");
  String foundmacaddrprefix = String("00:00");
  unsigned long currentMillis = 0; 
  unsigned long previousMillis = 0;  
  float apdsAverage=0;
  String droidString = "DROID";
  loopCount++;
  String myName;
  lightsOn(0);
  digitalWrite(LED_PWR, HIGH); //turn LED off 
  int i = 0;
  int noise = 0;
  int rand = 0;
  bool stringmatch = false;
  //int r, g, b, gesture;
  //sample distance from IR sensor
          if (APDS.proximityAvailable()) {
           // Read the proximity where:
           // - 0   => close, 255=> far, -1 error
            int proximity = APDS.readProximity();
            //Serial.println(proximity);
           }
  if(paired==false && loopCount%60==0){
    Serial.print("+");  
    lightsOn(1);    
    peripheral = BLE.available();
  
    if (peripheral) {
    // ...

    Serial.println("Connecting ...");
     // print the local name, if present
     // print the local name, if present
     if (true) {
       macaddr=peripheral.address();

       Serial.println(macaddr);
       foundmacaddrprefix=macaddr.substring(0,5);
       stringmatch = droidmacaddrprefix.equals(foundmacaddrprefix);
       Serial.println(stringmatch);
       //Serial.println(peripheral.localName().length());
       //myName=peripheral.localName(); doesnt seem to work for nicla vision
       //if (peripheral.localName().length()==5) {i=0;}
       //else {Serial.println("wrong name identified"); return;}
       
       //BLE.stopScan(); //must stop scanning before connecting 
       previousMillis = millis(); 
       currentMillis = millis();     
       while((currentMillis-80)<previousMillis){currentMillis = millis(); } //loop for 80ms
       if(stringmatch) {
         BLE.stopScan();
         peripheral.connect();
       }
       else {    
         Serial.print("unable to connect");   
         return;
       }   
       previousMillis = millis(); 
       currentMillis = millis();     
       while((currentMillis-20)<previousMillis){currentMillis = millis(); } //loop for 20ms
       if(peripheral.discoverAttributes() )
       {
        Serial.println("discovered attributes");
        previousMillis = millis(); 
        currentMillis = millis();     
        while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
        notifyCharacteristic = peripheral.characteristic("09b600b0-3e42-41fc-b474-e9c0c8f0c801");  
        droidCharacteristic = peripheral.characteristic("09b600b1-3e42-41fc-b474-e9c0c8f0c801");
        if (!droidCharacteristic) {
         //Serial.println("Peripheral error at droid characteristic...");
         peripheral.disconnect();
         return;
        }
       previousMillis = millis(); 
       currentMillis = millis();     
       while((currentMillis-40)<previousMillis){currentMillis = millis(); } //loop for 40ms
       Serial.println("@@ DROID @@");
       lightsOn(3);   
       previousMillis = millis(); 
       currentMillis = millis(); 
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis();     
       while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis(); 
       //Serial.println("+");        
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis(); 
       //Serial.println(previousMillis);    
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } //loop for 25ms
       droidCharacteristic.writeValue(firstCommand,3,true);
       previousMillis = millis(); 
       //Serial.println(previousMillis);    
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(secondCommand,8,true);
       previousMillis = millis(); 
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(thirdCommand,8,true);
       previousMillis = millis(); 
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(secondCommand,8,true);
       previousMillis = millis(); 
       while((currentMillis-25)<previousMillis){currentMillis = millis(); } 
       droidCharacteristic.writeValue(thirdCommand,8,true);
       Serial.println("Initialization commands sent, should hear beeps from droid");
       previousMillis = millis(); 
       currentMillis = millis();
       while((currentMillis-1000)<previousMillis){currentMillis = millis(); }  
       paired=true;    
       // assign event handlers for connected, disconnected to peripheral
      // BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
      // BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
    }
    else 
       {
        Serial.print("no characteristics found...");
        loopCount++;
        peripheral.disconnect();             
        BLE.end();
        startBLE();
        }
    }
    else {Serial.println("anonymous BLE device"); return;}
    }
  }
  //what to do when BLE is connected to droid and communicating
  else 
  {
    //BLE.poll();
    //delay(150);
       previousMillis = millis(); 
       currentMillis = millis();
       while((currentMillis-60)<previousMillis){currentMillis = millis();  //wait 60ms
          lightsOn(3);
          rand = random(9);
        //  Serial.println(rand);
        //    Serial.print(speech[0]);
        //    Serial.print(speech[1]);
        //    Serial.print(speech[2]);
        //    Serial.print(speech[3]);
        //    Serial.print(speech[4]);
        //    Serial.println(speech[5]);
          if (rand==0 ) {
            if(paired && speech[0]==1 && speech[1]==1 && speech[2]==0 && speech[3]==1 && speech[4]==1 && speech[5]==1){makeNoise(1,2);Serial.print(".");}                       
          }   
          else if (rand==1 ) {
            if(paired && speech[0]==0 && speech[1]==1 && speech[2]==1 && speech[3]==1 && speech[4]==1 && speech[5]==1){makeNoise(3,1);Serial.print(".");}              
          }    
          else if (rand==2) {
            if(paired && speech[0]==0) {moveHead();}              
          }    
          else if (rand==3) {
            if(paired && speech[0]==0 && speech[1]==1 ) {makeNoise(2,2);}    
            if(paired && speech[0]==0 && speech[1]==0 ) {moveRotate();}             
          }        
          else if (rand==4) {
            if(paired) {moveForward(500);}              
          }     
          else {lightsOn(0);}              
        //print_results = 0;
  } //end else


          if (samplesRead) {
	          arm_rfft_instance_q15 fft_instance;
	          q15_t fftoutput[256*2]; //has to be twice FFT size
            arm_rfft_init_q15(&fft_instance, 256/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);
            arm_rfft_q15(&fft_instance, (q15_t*)sampleBuffer, fftoutput);
		        arm_abs_q15(fftoutput, fftoutput, 256);

	        	int temp = 0;
            int temporary = 0;    
             boolean tooLoud = false; 
             heyWord==false;  
             //shift speech array to the right 
             if(speech[4==1]) {speech[5]=1;} else {speech[5]=0;}
             if(speech[3==1]) {speech[4]=1;} else {speech[4]=0;}
             if(speech[2==1]) {speech[3]=1;} else {speech[3]=0;}
             if(speech[1==1]) {speech[2]=1;} else {speech[2]=0;}
             if(speech[0==1]) {speech[1]=1;} else {speech[1]=0;}
             //ignoring the higher frequencies
             for (int i = 1; i < 40; i++) {
              // Serial.print(fftoutput[i]);
              // Serial.print(",");
               if((int)fftoutput[i]>140) {tooLoud=true;} //probably the droid itself         
               if((int)fftoutput[i]>7 && (int)fftoutput[i]<90) {temp = temp + 1;} //human sound 
               //Serial.print((int)fftoutput[i]);
               //Serial.print(",");         
              }
              for (int i = 60; i < 250; i++) {
               if((int)fftoutput[i]>50) {tooLoud=true;} //probably the droid itself                  
              }
              if(fftoutput[50]>20) {temp=temp-1;} //probably not speech
              if(fftoutput[60]>20) {temp=temp-1;} //probably not speech
              if(fftoutput[70]>20) {temp=temp-1;} //probably not speech
              if(fftoutput[80]>20) {temp=temp-1;} //probably not speech
              if(fftoutput[90]>20) {temp=temp-1;} //probably not speech
              if(fftoutput[100]>20) {temp=temp-1;} //probably not speech
              if(fftoutput[110]>20) {temp=temp-1;} //probably not speech
              if(temp>6 && temp<24 && tooLoud==false){     
              // probably human speech typically looks like 9,1,15,16,13,2,17,3,80,24,10,22,10,15,5,7,1,7,5,14,3,1,1,0..
              //usually repeats for a line or two then values drop to 1,8,12,1,2,4,2,1,3,3,1....
               // Serial.println("h");
                speech[0]=1; //speech heard for this sample
              } 
              else {speech[0]=0;}

     samplesRead=0;
   } //end of samplesread

  }
} //end loop
void startBLE() {
//initialize BLE  
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }
    
Serial.println("BLE started...");
  //BLE central scan
   BLE.scan(); //scan run continuously until stopped
  //BLE.stopScan();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  if(central.address().startsWith("f1")) {Serial.println("match");paired = true;}
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  if(central.address().startsWith("f1")) {Serial.println("match");paired = false; delay(100); BLE.scan();}
}



// function to turn feet and rotate slightly
void moveRotate() {
    Serial.print(7);
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;     
    droidCharacteristic.writeValue(eleventhCommand,10,true);
    currentMillis=millis();
    previousMillis=millis(); 
    while((currentMillis-140)<previousMillis){currentMillis = millis(); } //loop for 140ms
    droidCharacteristic.writeValue(fifthCommand,10,true);
    currentMillis=millis();
    previousMillis=millis(); 
    while((currentMillis-250)<previousMillis){currentMillis = millis(); } //loop for 250ms    
}

// function to turn feet and rotate slightly
void moveForward(int distance) {
    Serial.print(6);
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;     
    droidCharacteristic.writeValue(fourthCommand,10,true);
    droidCharacteristic.writeValue(seventhCommand,10,true);
    currentMillis=millis();
    previousMillis=millis(); 
    while((currentMillis-distance)<previousMillis){currentMillis = millis(); } //loop for parameter value
    droidCharacteristic.writeValue(fifthCommand,10,true);
    droidCharacteristic.writeValue(eighthCommand,10,true);
    currentMillis=millis();
    previousMillis=millis(); 
    while((currentMillis-250)<previousMillis){currentMillis = millis(); } //loop for 250ms    
}

// function to play a sound from the sound bank based on two parameters passed in
void makeNoise(int bank, int slot) {
    Serial.print(8);
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    currentMillis=millis();
    previousMillis=millis();    
    if(bank==0){droidCharacteristic.writeValue(firstBank,8,true);}
    if(bank==1){droidCharacteristic.writeValue(secondBank,8,true);}
    else{droidCharacteristic.writeValue(thirdBank,8,true);}
    while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
    if(slot==0){droidCharacteristic.writeValue(firstSound,8,true);}  
    if(slot==1){droidCharacteristic.writeValue(secondSound,8,true);}  
    if(slot==2){droidCharacteristic.writeValue(thirdSound,8,true);}  
    if(slot==3){droidCharacteristic.writeValue(fourthSound,8,true);}  
    if(slot==4){droidCharacteristic.writeValue(fifthSound,8,true);}     
    else {droidCharacteristic.writeValue(sixthSound,8,true);} 
    while((currentMillis-200)<previousMillis){currentMillis = millis(); } //loop for 200ms    
}

//rotate head
void moveHead() {
  Serial.print(9);
  unsigned long currentMillis = 0; 
  unsigned long previousMillis = 0;  
  droidCharacteristic.writeValue(sixthCommand,6,true);
  currentMillis=millis();
  previousMillis=millis();    
  while((currentMillis-1000)<previousMillis){currentMillis = millis(); } //loop for 1000ms
  droidCharacteristic.writeValue(sixthCommand,6,true);
}

float measureAcceleration() {
  float x,y,z,newx,newy,newz,squared,newsquared;
  squared  = 1.0;
    if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    newx = x * x;
    newy = y * y;
    newz = z * z;
    squared = newx + newy + newz;
    newsquared = sqrt(squared);
    //Serial.print(summed);
    //Serial.print(x);
    //Serial.print('\t');
    //Serial.print(y);
    //Serial.print('\t');
    //Serial.println(z);
  }  
  return newsquared;
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

boolean printByte(byte X) {  
  if (X > 6) {
    //Serial.print(X, HEX);    
    return true;   //heard a loud sound for 250ms    
  }    
  else return false;
   //if (X < 10) {Serial.print("0");}
   //Serial.print(X, HEX);
}

void lightsOn(int color) {
  if(color==0){
    //off
    digitalWrite(RED, HIGH); // turn the LED off by making the voltage HIGH
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  }  
  if(color==1){
    //green
    digitalWrite(RED, HIGH); // turn the LED off by making the voltage HIGH
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, HIGH);
  }  
if(color==2){
    //pink
    digitalWrite(RED, LOW); // turn the LED off by making the voltage LOW
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, LOW);
  }    
  if(color==3){
    //blue
    digitalWrite(RED, HIGH); // turn the LED off by making the voltage LOW
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, LOW);
  }  
    if(color==4){
    //red
    digitalWrite(RED, LOW); // turn the LED off by making the voltage LOW
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
  }  
}
