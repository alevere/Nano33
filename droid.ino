/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0

/**
 * Define the number of slices per model window. E.g. a model window of 1000 ms
 * with slices per model window set to 4. Results in a slice size of 250 ms.
 * For more info: https://docs.edgeimpulse.com/docs/continuous-audio-sampling
 */
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 2

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/* Includes ---------------------------------------------------------------- */
#include <PDM.h>
#include <DroiDepotSpeech_inferencing.h>
//#include <Arduino_APDS9960.h>
//dont use with impulse edge
//#include <arm_math.h>
//#include <Arduino_LSM9DS1.h>
//install arduinoble v.1.3.2
#include <ArduinoBLE.h>
//#define ARM_MATH_CM4

// Feb 2023
// Code to make a smarter and somewhat autonomous Droid from Galaxy's Edge brought from the Droid Depot
// This code is for Nano BLE 33 Sense
// Some code is from https://github.com/arduino/ArduinoAI
// Some code is from https://forum.arduino.cc/u/gssd/summary
// Using Arduino IDE 2.0.3
// Imported zip library from Impulse Edge into IDE

 #define RED 22     
 #define BLUE 24     
 #define GREEN 23
 #define LED_PWR 25
/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static bool record_ready = false;
static signed short *sampleBuffer;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
volatile bool paired;
volatile bool droidFlag;
volatile int loopCount;
static char firstCommand[] = {0x22,0x20,0x01};  
static char secondCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x1f,0x00};
static char thirdCommand[] = {0x27,0x42,0x0f,0x44,0x44,0x00,0x18,0x02}; 
static char fourthCommand[] = {0x29,0x42,0x05,0x46,0x00,0x80,0x01,0x2c,0x00,0x00}; //0first motor forwards half power
static char fifthCommand[] = {0x29,0x42,0x05,0x46,0x00,0x00,0x01,0x2c,0x00,0x00};  //stop motor  
static char sixthCommand[] = {0x25,0x00,0x0c,0x42,0x08,0x02}; //rotate head
static char seventhCommand[] = {0x29,0x42,0x05,0x46,0x01,0x80,0x01,0x2c,0x00,0x00}; //second motor 0x46,0x01 is passenger leg
static char eighthCommand[] = {0x29,0x42,0x05,0x46,0x01,0x00,0x01,0x2c,0x00,0x00}; //stop motor
static char ninthCommand[] = {0x29,0x42,0x05,0x46,0x80,0x90,0x01,0x2c,0x00,0x00}; 
static char tenthCommand[] = {0x29,0x42,0x05,0x46,0x81,0x90,0x01,0x2c,0x00,0x00}; 
static char eleventhCommand[] = {0x29,0x42,0x05,0x46,0x80,0x80,0x01,0x2c,0x00,0x00}; //0first motor backwards half power
static char twelthCommand[] = {0x29,0x42,0x05,0x46,0x81,0x80,0x01,0x2c,0x00,0x00}; //second motor backwards half power

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

BLECharacteristic droidCharacteristic;
BLECharacteristic notifyCharacteristic;
BLEDevice peripheral;
/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    droidFlag = false;
    loopCount=0;
    // intitialize the digital Pin as an output
    paired = false; 
    pinMode(RED, OUTPUT);
    pinMode(BLUE, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(LED_PWR, OUTPUT);
    lightsOn(1);   
    delay(4000);
    lightsOn(0);
    startBLE(); 
    //Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    //Serial.println("Edge Impulse Inferencing Demo");
    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    run_classifier_init();
    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }
}

/**
 * @brief      Arduino main function. Runs the inferencing loop over and over and over.
 */
void loop()
{
    //initialize variables
    loopCount++;
    String droidString = "DROID";
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    String myName;
    lightsOn(0);
    digitalWrite(LED_PWR, HIGH); //turn LED off 
    int i = 0;
    int rand = 0;
  if(loopCount%40==0) {
      if(peripheral.connected()==false){
        paired==false;
        startBLE(); 
      }
    }
  if(!paired){
    //Serial.print("+");    
    lightsOn(2);    
    peripheral = BLE.available();
     if(peripheral) {
    // ...

    //Serial.println("Connecting ...");
     
     // print the local name, if present
     // print the local name, if present
     if (peripheral.hasLocalName()) {
       //Serial.println(peripheral.localName());
     //myName=peripheral.localName();
       if (peripheral.localName().length()==5) {i=0;}
       else {Serial.println("wrong name identified"); return;}
       BLE.stopScan(); //must stop scanning before connecting 
       previousMillis = millis(); 
       currentMillis = millis();     
       while((currentMillis-30)<previousMillis){currentMillis = millis(); } //loop for 30ms
       if(peripheral.connect()) {
         //Serial.println("Connected to BLE peripheral.\n");   
       } 
       else {
         startBLE();         
         return;
       }   
       previousMillis = millis(); 
       currentMillis = millis();     
       while((currentMillis-40)<previousMillis){currentMillis = millis(); } //loop for 30ms
       if(peripheral.discoverAttributes() )
       {
        Serial.println("discovered attributes");
        notifyCharacteristic = peripheral.characteristic("09b600b0-3e42-41fc-b474-e9c0c8f0c801");  
        droidCharacteristic = peripheral.characteristic("09b600b1-3e42-41fc-b474-e9c0c8f0c801");
        if (!droidCharacteristic) {
         //Serial.println("Peripheral error at droid characteristic...");
         peripheral.disconnect();
         return;
        }
       // assign event handlers for connected, disconnected to peripheral
       //BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
       //BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);        
       //Serial.println("@@ DROID @@");
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
       while((currentMillis-1000)<previousMillis){currentMillis = millis(); }  
       paired=true;    
    }
      else {
         peripheral.disconnect();  
         startBLE();            
         return;
         }
    }
    else {Serial.println("anonymous BLE device"); return;}
    }
  }
  else {
    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }
    lightsOn(0); //turn light off
    if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
        // print the predictions
        //ei_printf("Predictions ");
        //ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",result.timing.dsp, result.timing.classification, result.timing.anomaly);
        //ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            //ei_printf("    %s: %.5f\n", result.classification[ix].label,result.classification[ix].value);
        }
        // 0 = backward, 1=dance,2=forward,3=go,4=noise,5=right,6=silence,7=six,8=stop,9=turn,10=two
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        //ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
//if connected to BLE and forward heard make a noise
          //just noise
          rand = random(5);
          if (result.classification[4].value>0.44||result.classification[6].value>0.44) {
            //makeNoise(0,0,droidCharacteristic);     
            //ei_printf("make noise..."); 
            //droidCharacteristic.writeValue(sixthCommand,6,true);  
            //if(paired) {moveForward(); }                       
          }   
          else if (result.classification[10].value>0.6) {
            if(paired) {makeNoise(0,rand); }              
          }    
          else if (result.classification[0].value>0.5) {
            if(paired) {moveBackward(); }              
          }    
          else if (result.classification[2].value>0.6) {
            if(paired) {moveForward(); }              
          }    
          else if (result.classification[9].value>0.4||result.classification[5].value>0.5) {
            if(paired) {moveRotate(0); }              
          }      
          else if (result.classification[7].value>0.6) {
            if(paired) {makeNoise(2,5); }              
          }   
          else if (result.classification[1].value>0.2) {
            if(paired) {
              makeNoise(2,2);
              moveHead(); 
              makeNoise(2,3);
              moveRotate(0);
              makeNoise(2,4);      
              moveHead();
              moveRotate(1);  
              makeNoise(2,5);                                              
              }              
          }     
          else {lightsOn(4);}              
        print_results = 0;
    }
  } //end else
} //end loop

/**
 * @brief      PDM buffer full callback
 *             Get data and call audio thread callback
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (record_ready == true) {
        for (int i = 0; i<bytesRead>> 1; i++) {
            inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

            if (inference.buf_count >= inference.n_samples) {
                inference.buf_select ^= 1;
                inference.buf_count = 0;
                inference.buf_ready = 1;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    lightsOn(3); // turn light blue if mic is listening
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[1] == NULL) {
        free(inference.buffers[0]);
        return false;
    }

    sampleBuffer = (signed short *)malloc((n_samples >> 1) * sizeof(signed short));

    if (sampleBuffer == NULL) {
        free(inference.buffers[0]);
        free(inference.buffers[1]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    PDM.setBufferSize((n_samples >> 1) * sizeof(int16_t));

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
    }

    // set the gain, defaults to 20
    PDM.setGain(127);

    record_ready = true;

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        delay(1);
    }

    inference.buf_ready = 0;

    return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffers[0]);
    free(inference.buffers[1]);
    free(sampleBuffer);
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

void startBLE() {
//initialize BLE  
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    //while (1);
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
  if(central.address().startsWith("f1")) {Serial.println("match+");paired = true;}
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  if(central.address().startsWith("f1")) {Serial.println("match");paired = false; delay(100); BLE.scan();}
}

// function to play a sound from the sound bank based on two parameters passed in
void makeNoise(int bank, int slot) {
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
    while((currentMillis-500)<previousMillis){currentMillis = millis(); } //loop for 500ms       
          }

// function to play a sound from the sound bank based on two parameters passed in
void moveForward() {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    //Serial.println("forward");
    currentMillis=millis();
    previousMillis=millis();    
    droidCharacteristic.writeValue(fourthCommand,10,true);
    droidCharacteristic.writeValue(seventhCommand,10,true);
    while((currentMillis-1500)<previousMillis){currentMillis = millis(); } //loop for 30ms
    droidCharacteristic.writeValue(fifthCommand,10,true);  
    droidCharacteristic.writeValue(eighthCommand,10,true); 
    while((currentMillis-500)<previousMillis){currentMillis = millis(); } //loop for 500ms       
          }
          
   void moveBackward() {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    //Serial.println("backward");
    currentMillis=millis();
    previousMillis=millis();    
    droidCharacteristic.writeValue(eleventhCommand,10,true);
    droidCharacteristic.writeValue(twelthCommand,10,true);
    while((currentMillis-1000)<previousMillis){currentMillis = millis(); } //loop for 30ms
    droidCharacteristic.writeValue(fifthCommand,10,true);  
    droidCharacteristic.writeValue(eighthCommand,10,true); 
    while((currentMillis-500)<previousMillis){currentMillis = millis(); } //loop for 500ms       
          }    


   void moveRotate(int direction) {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    //Serial.println("backward");
    currentMillis=millis();
    previousMillis=millis();    
    if(direction==0) {droidCharacteristic.writeValue(eleventhCommand,10,true);}
    else {droidCharacteristic.writeValue(twelthCommand,10,true);}
    while((currentMillis-300)<previousMillis){currentMillis = millis(); } //loop for 30ms
    droidCharacteristic.writeValue(fifthCommand,10,true);  
    droidCharacteristic.writeValue(eighthCommand,10,true); 
    while((currentMillis-500)<previousMillis){currentMillis = millis(); } //loop for 500ms       
          }     

  void moveHead() {
    unsigned long currentMillis = 0; 
    unsigned long previousMillis = 0;  
    //Serial.println("turn head");
    currentMillis=millis();
    previousMillis=millis();    
    droidCharacteristic.writeValue(sixthCommand,6,true); 
    while((currentMillis-800)<previousMillis){currentMillis = millis(); } //loop for 500ms       
          }  
                   
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
