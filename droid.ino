#include <arm_math.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#define ARM_MATH_CM4
// June 2022
// Code to make a smart, autonomous Droid from Galaxy's Edge bought at Droid Depot
// This code is for Nano BLE 33 Sense
// Some code is from https://github.com/arduino/ArduinoAI
// Some code is from https://forum.arduino.cc/u/gssd/summary
// Using Arduino IDE 2.0.0-rc6

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
volatile int sampleHuman;
volatile int humanDelay;
volatile int loopCount;
BLEDevice peripheral;  
BLECharacteristic droidCharacteristic;
BLECharacteristic notifyCharacteristic;

void setup() {
  droidFlag = false;
  sampleHuman = 0; //count number of sequential human sounds
  humanDelay=0; //count delay or pause in speech
  loopCount=0;
 // intitialize the digital Pin as an output
  paired = false; 
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  lightsOn(1);   
  delay(1500);
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
  //increase microphone sensitivity  
  PDM.setGain(0x10);  
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
    while (1);
  }  
  unsigned long previousMillis = 0;        // will store last time LED was updated
  long OnTime = 250;           // milliseconds of on-time
  long OffTime = 750;          // milliseconds of off-time
}

// the loop function runs over and over again
void loop() {
  loopCount++;
  boolean loudFlag = false;  
  boolean accelFlag = false; 
  float currentAcceleration;
  lightsOn(0);
  digitalWrite(LED_PWR, HIGH); //turn LED off 
  int i = 0;
// assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);  
  BLE.poll(1);
  
  currentAcceleration = measureAcceleration();
  if (currentAcceleration > 1.4) {
       Serial.println("stop shaking me");
       Serial.println(currentAcceleration);       
       accelFlag = true;      
  } 
  // Wait for samples to be read
  if (samplesRead) {
     short micLevel;
     // arm_rms_q15 (sampleBuffer, samplesRead, &micLevel);

	  static arm_rfft_instance_q15 fft_instance;
	  static q15_t fftoutput[256*2]; //has to be twice FFT size
	  static byte spectrum[32];
    arm_rfft_init_q15(&fft_instance, 256/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);
    arm_rfft_q15(&fft_instance, (q15_t*)sampleBuffer, fftoutput);
		arm_abs_q15(fftoutput, fftoutput, 256);

		int temp = 0;
    int temporary = 0;    
    for (int i = 1; i < 256; i++) {
      if(i>3 && i<30) {   
        if((int)fftoutput[i]>9) {temp = temp + 1;} //human frequency heard 
      Serial.print((int)fftoutput[i]);
      Serial.print(",");   
      }       
    }
 
   if(temp>1&&temp<18){     
     // possible speech
     temporary = sampleHuman << 1;
     sampleHuman = temporary + 1;     
     }
   else {
    //not speech  
     temporary = sampleHuman << 1;
     sampleHuman = temporary;             
     } 
   Serial.println(sampleHuman);
   Serial.println("\n");
    // Look for  11001110 or 11110000001111110
     // [1]{2,4},[0]{2,6},[1]{3,6},[0]{1}   
     // 1111010111110000111000     111111111111000000111110    
   int another=0;
   int anotherAgain=0;
   int sampleShifted=0;      
   another=15 & sampleHuman;
   if(another==14){
     for(int i =1; i<16; i++){
        sampleShifted=sampleHuman >> i;
        anotherAgain=15 & sampleShifted;
        if(anotherAgain==14){
        //probably have two spoken words
        Serial.println("speech");         
        loudFlag=true;            
        }               
      }        
    }   
   int loudnessCount = 0;    
   //for (i=0;i<32;i++) {
   //   priorSound = printByte(spectrum[i]);
   //if(priorSound) {loudnessCount=loudnessCount+1;}   
   //}
   // if(loudnessCount>3) {
     // loudFlag=true;         
     // }
        //microphoneLevelCharacteristic.writeValue((byte *) &spectrum, 32);
    // Clear the read count
    samplesRead = 0;  
  if(true) {
      Serial.println(".");
      if(droidFlag){
        droidCharacteristic = peripheral.characteristic("09b600b1-3e42-41fc-b474-e9c0c8f0c801");
        if (!droidCharacteristic) {
          Serial.println("No droid characteristic...");
          peripheral.disconnect();
          droidFlag = false;          
        } 
        else {
             //we are probably connected to droid
            if(loudFlag) {makeBeep(2);}                   
        }
      }
      else 
      {
         if(loopCount%5==0) {
           beginSearching();
         }
      }
  }

  
  //makeNoise(1);
  } 
  if (loudFlag){
    lightsOn(2);
    //makeNoise(1);
  } 
  if (accelFlag){
    lightsOn(2);
    //makeNoise(2);
  } 
}

void startBLE() {
//initialize BLE  
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }
    
Serial.println("BLE started...");
  //BLE central scan
  BLE.scan(); 
  //BLE.stopScan();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void makeNoise(int location) {
  byte data[8] = {0x83, 0x01, 0x0A, 0x04, 0x01, 0x02, 0xA6, 0x01};
  
  if (location==1) {byte data[8] = {0x83, 0x01, 0x0A, 0x04, 0x01, 0x02, 0xA6, 0x01}; digitalWrite(BLUE, LOW); } 
  else {byte data[8] = {0x83, 0x01, 0x0A, 0x04, 0x07, 0x02, 0xA6, 0x01}; digitalWrite(RED, LOW); }
  BLE.setDeviceName("DROIDEPOT");
  BLE.setLocalName("DROIDEPOT");
  BLE.setManufacturerData(data, 8);
  // start advertising
  BLE.setAdvertisingInterval(320);
  BLE.advertise();
  digitalWrite(GREEN, HIGH); // turn the LED off (HIGH is the voltage level)
  digitalWrite(LED_PWR, HIGH);
  delay(250);  
  delay(10000);
  BLE.stopAdvertise();
  digitalWrite(LED_PWR, LOW);
}

void beginSearching() {
  BLE.scan();
  peripheral = BLE.available();
  bool status = false;
  if (peripheral) {
    Serial.println("\n");
    Serial.println(peripheral.localName());
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(peripheral.characteristicCount());  
   if (peripheral.localName() == "DROID") {       
      BLE.stopScan();
      Serial.println("!!!!!!!!      Found an astromech    !!!!!!!");
      droidFlag = true;
      lightsOn(3);
      droidFlag = connectPeripheral();
       }
   else {Serial.println('wrongperiph');}      
  }
  else { Serial.println('noperiph'); }
}

//connect to bluetooth sensor or peripheral
bool connectPeripheral(){
  byte notifyValue;
  char firstCommand[] = {0x22,0x20,0x01};  
  char secondCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x00};
  char thirdCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x02}; 
  char fourthCommand[] = {0x29,0x42,0x05,0x46,0x03,0x50,0x01,0x2c,0x00,0x00}; //0x01 is passenger leg, 0x02 head
  char fifthCommand[] = {0x29,0x42,0x05,0x46,0x03,0x00,0x01,0x2c,0x00,0x00};   
  unsigned long currentMillis = millis();   
  if(!peripheral.connect()) {
    return false;
  }
  Serial.println("Connected to BLE peripheral.\n");  
  if(!peripheral.discoverAttributes() )
  {
    peripheral.disconnect();
    Serial.println("no discovered attr");
    return false;
  }  
  notifyCharacteristic = peripheral.characteristic("09b600b0-3e42-41fc-b474-e9c0c8f0c801");  
  droidCharacteristic = peripheral.characteristic("09b600b1-3e42-41fc-b474-e9c0c8f0c801");
  if (!droidCharacteristic) {
    Serial.println("Peripheral error at droid characteristic...");
    peripheral.disconnect();
    return false;
  } 
  if (!droidCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable droid characteristic!");
    peripheral.disconnect();
    return false;
  }
  Serial.println("Can write to Droid :\)");
  droidCharacteristic.writeValue(firstCommand,3,true);
  currentMillis = millis(); 
  droidCharacteristic.writeValue(firstCommand,3,true);
  currentMillis = millis(); 
  droidCharacteristic.writeValue(firstCommand),3,true;
  currentMillis = millis(); 
  droidCharacteristic.writeValue(firstCommand),3,true;
  currentMillis = millis(); 
  droidCharacteristic.writeValue(secondCommand,8,true);
  currentMillis = millis(); 
  droidCharacteristic.writeValue(thirdCommand,8,true);
  currentMillis = millis(); 
  droidCharacteristic.writeValue(secondCommand,8,true);
  currentMillis = millis(); 
  droidCharacteristic.writeValue(thirdCommand,8,true);
  Serial.println("Initialization commands sent, should hear beeps from droid");
  paired = true;
  delay(250);
  notifyCharacteristic.readValue(notifyValue);  
  Serial.println(notifyValue);
  droidCharacteristic.writeValue(fourthCommand,10,true);
  delay(1000);
  droidCharacteristic.writeValue(fifthCommand,10,true);       
  return true;
}

//have droid make a noise
bool makeBeep(int noise){

  char firstBank[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x00};  
  char secondBank[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x01};  
  char firstSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x00};  
  char secondSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x01};     
  char thirdSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x02};   
  char fourthSound[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x03};     
  if(!peripheral.connect()) {
    return false;
  }
  Serial.println("Connected to BLE peripheral.\n");  
  if(!peripheral.discoverAttributes() )
  {
    peripheral.disconnect();
    Serial.println("no discovered attr");
    return false;
  }  
  droidCharacteristic = peripheral.characteristic("09b600b1-3e42-41fc-b474-e9c0c8f0c801");
  if (!droidCharacteristic) {
    Serial.println("Peripheral does not droid characteristic...");
    peripheral.disconnect();
    return false;
  } 
  if (!droidCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable droid characteristic!");
    peripheral.disconnect();
    return false;
  }
  droidCharacteristic.writeValue(firstBank,8,true);
  delay(30);
  if(noise==0){droidCharacteristic.writeValue(firstSound,8,true);}  
  if(noise==1){droidCharacteristic.writeValue(secondSound,8,true);} 
  if(noise==2){droidCharacteristic.writeValue(thirdSound,8,true);}   
  return true;
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
    Serial.print(X, HEX);    
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
}
