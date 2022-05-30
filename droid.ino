#include <arm_math.h>
#include <PDM.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#define ARM_MATH_CM4

// How to control the RGB Led and Power Led of the Nano 33 BLE boards.  
// This code is for Nano BLE 33 Sense
// Some code is from https://github.com/arduino/ArduinoAI

 #define RED 22     
 #define BLUE 24     
 #define GREEN 23
 #define LED_PWR 25
const int buttonPin = 4;            // set buttonPin to digital pin 4; 7th from bottom on right side
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
void setup() {
 // intitialize the digital Pin as an output
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
}

// the loop function runs over and over again
void loop() {
  boolean loudFlag = false;  
  boolean accelFlag = false; 
  float currentAcceleration;
  lightsOn(0);
  delay(80);
  digitalWrite(LED_PWR, HIGH);  
  currentAcceleration = measureAcceleration();
  if (currentAcceleration > 1.5 || currentAcceleration < 0.5) {
       Serial.println("stop shaking me");
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

		float temp = 0;
    for (int i = 1; i < 256; i++) {
      temp = temp + fftoutput[i];
      if ((i &3) == 2){
        if (temp>1023) {temp=1023;};
        spectrum[i>>3] = (byte)(temp/2);
        temp = 0;
      }
  }
   int i;
   boolean priorSound;   
   int loudnessCount = 0;    
   for (i=0;i<32;i++) {
      priorSound = printByte(spectrum[i]);
   if(priorSound) {loudnessCount=loudnessCount+1;}  
   if(loudnessCount>2) loudFlag=true;     
   }
   
        //microphoneLevelCharacteristic.writeValue((byte *) &spectrum, 32);
    // Clear the read count
    samplesRead = 0;  
  //beginSearching();
  //makeNoise(1);
  }
  Serial.println("\n");  
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
  digitalWrite(GREEN, HIGH); // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_PWR, HIGH);
  delay(250);  
  delay(10000);
  BLE.stopAdvertise();
  digitalWrite(LED_PWR, LOW);
}

void beginSearching() {

  BLEDevice peripheral = BLE.available();
  bool status = false;
  if (peripheral) {
     // ...
  digitalWrite(RED, LOW); // turn the LED on by making the voltage LOW
  delay(100);        // wait for a 100ms
  digitalWrite(GREEN, HIGH);
  delay(100);  
  digitalWrite(BLUE, LOW);
  delay(100);   
  digitalWrite(LED_PWR, HIGH);     
  delay(2000);
  digitalWrite(LED_PWR, LOW); 
  Serial.println(peripheral.localName());
  Serial.println(peripheral.advertisedServiceUuid());
  Serial.println(peripheral.characteristicCount());  
   if (peripheral.localName() == "DROID") {
      Serial.println("Found an astromech!");
      BLE.stopScan();
      status = explorePeripheral(peripheral);            
      }
   else {BLE.stopScan();}      
  }
  else { digitalWrite(LED_PWR, LOW); delay(1000);}
}

//connect to bluetooth sensor or peripheral
bool explorePeripheral(BLEDevice myperipheral){
  if(!myperipheral.connect()) {
    return false;
  }
  if(!myperipheral.discoverAttributes() )
  {
    myperipheral.disconnect();
    return false;
  }  
    
}

float measureAcceleration() {
  float x,y,z,summed;
  summed  = 1.0;
    if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    summed = x + y + z;    
    //Serial.print(summed);
    //Serial.print(x);
    //Serial.print('\t');
    //Serial.print(y);
    //Serial.print('\t');
    //Serial.println(z);
  }  
  return summed;
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
  Serial.print(X, HEX);  
  if (X > 7) return true;   //heard a loud sound for 250ms
  else return false;
   //if (X < 10) {Serial.print("0");}
   //Serial.print(X, HEX);
}

void lightsOn(int color) {
  Serial.println("LightsOn...");
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
}