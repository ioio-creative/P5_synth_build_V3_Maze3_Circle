#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthWaveform       Voice1LFO;      //xy=638.232666015625,481.00472259521484
AudioSynthWaveformDc     Voice1LfoDC;    //xy=640.232666015625,430.00472259521484
AudioSynthWaveformDc     Voice2LFODC;    //xy=639.232666015625,813.0047225952148
AudioSynthWaveform       Voice2LFO;      //xy=639.232666015625,869.0047225952148
AudioSynthNoiseWhite     Voice1NOISE;    //xy=645.232666015625,323.00472259521484
AudioSynthWaveformDc     Voice2Env;      //xy=645.232666015625,740.0047225952148
AudioSynthWaveformDc     Voice1Env;      //xy=647.232666015625,366.00472259521484
AudioSynthWaveform       Voice2waveform; //xy=651.232666015625,691.0047225952148
AudioSynthWaveform       voice5Waveshift;      //xy=662.3437576293945,220.01050758361816
AudioMixer4              Voice1ModMixer; //xy=837.232666015625,464.00472259521484
AudioEffectMultiply      Voice2Multiply; //xy=859.232666015625,745.0047225952148
AudioMixer4              Voice2ModMixer; //xy=863.232666015625,830.0047225952148
AudioEffectMultiply      Voice1Multiply; //xy=867.232666015625,332.00472259521484
AudioSynthWaveformSineModulated sine_fm1;       //xy=875.232666015625,943.0047225952148
AudioFilterStateVariable voice5Filter;        //xy=900.3437805175781,221.23264122009277
AudioFilterStateVariable Voice1filter1;  //xy=1078.232666015625,357.00472259521484
AudioFilterStateVariable Voice2filter;   //xy=1111.232666015625,744.0047225952148
AudioEffectFreeverb      freeverb1;      //xy=1201.232666015625,432.00472259521484
AudioEffectBitcrusher    bitcrusher1;    //xy=1204.232666015625,485.00472259521484
AudioEffectDelay         delay1;         //xy=1205.232666015625,586.0047225952148
AudioMixer4              LEFTMixer;      //xy=1351.232666015625,350.00472259521484
AudioMixer4              RIGHTMixer;     //xy=1460.232666015625,757.0047225952148
AudioOutputI2S           i2s1;           //xy=1672.232666015625,621.0047225952148
AudioConnection          patchCord1(Voice1LFO, 0, Voice1ModMixer, 1);
AudioConnection          patchCord2(Voice1LfoDC, 0, Voice1ModMixer, 0);
AudioConnection          patchCord3(Voice2LFODC, 0, Voice2ModMixer, 0);
AudioConnection          patchCord4(Voice2LFO, 0, Voice2ModMixer, 1);
AudioConnection          patchCord5(Voice2LFO, sine_fm1);
AudioConnection          patchCord6(Voice1NOISE, 0, Voice1Multiply, 0);
AudioConnection          patchCord7(Voice2Env, 0, Voice2Multiply, 1);
AudioConnection          patchCord8(Voice1Env, 0, Voice1Multiply, 1);
AudioConnection          patchCord9(Voice2waveform, 0, Voice2Multiply, 0);
AudioConnection          patchCord10(voice5Waveshift, 0, voice5Filter, 0);
AudioConnection          patchCord11(Voice1ModMixer, 0, Voice1filter1, 1);
AudioConnection          patchCord12(Voice2Multiply, 0, Voice2filter, 0);
AudioConnection          patchCord13(Voice2ModMixer, 0, Voice2filter, 1);
AudioConnection          patchCord14(Voice1Multiply, 0, Voice1filter1, 0);
AudioConnection          patchCord15(sine_fm1, 0, RIGHTMixer, 2);
AudioConnection          patchCord16(sine_fm1, 0, LEFTMixer, 2);
AudioConnection          patchCord17(voice5Filter, 0, LEFTMixer, 3);
AudioConnection          patchCord18(voice5Filter, 0, RIGHTMixer, 3);
AudioConnection          patchCord19(Voice1filter1, 0, freeverb1, 0);
AudioConnection          patchCord20(Voice1filter1, 0, delay1, 0);
AudioConnection          patchCord21(Voice1filter1, 0, bitcrusher1, 0);
AudioConnection          patchCord22(Voice1filter1, 1, RIGHTMixer, 0);
AudioConnection          patchCord23(Voice1filter1, 1, LEFTMixer, 0);
AudioConnection          patchCord24(Voice2filter, 0, RIGHTMixer, 1);
AudioConnection          patchCord25(Voice2filter, 0, LEFTMixer, 1);
AudioConnection          patchCord26(LEFTMixer, 0, i2s1, 0);
AudioConnection          patchCord27(RIGHTMixer, 0, i2s1, 1);
AudioControlSGTL5000     audioShield;    //xy=1542.232666015625,295.00472259521484
// GUItool: end automatically generated code



















//  ====================== end of audio set up  ============================================


//============== PropShield Motion Sensor Library =========================================

#include <NXPMotionSense.h>
#include <EEPROM.h>
//============== End of PropShield Motion Sensor Library =========================================

//============== Gyroscope MPU6050 Library (Not using) ================================================
/*
  #include <MPU6050_tockn.h>
  #include <Wire.h>

  MPU6050 mpu6050(Wire);
  long timer = 0;
*/
//============== End of Gyroscope MPU6050 Library (Not using) ======================================



//================== declare for calculating intensity============================================
//================ Prop Shield Motion ========================
NXPMotionSense imu;
NXPSensorFusion filter;
float gx, gy, gz;
float intensity;
int intensityThreshold = 30;
float intensityMap ;
//Prop shied
float yawPre;

//================== smoothing
boolean smoothEnable = true;
const int numSmooth = 10;
float smooth[numSmooth];
int datIndex = 0;
int total = 0;
float average = 0.0;


//============= audio
long freq;
float vol;

int Voice2lfoWaveShapeIndex;
short waveShapes[4] = {
  WAVEFORM_SINE,
  WAVEFORM_SAWTOOTH,
  WAVEFORM_SQUARE,
  WAVEFORM_SAMPLE_HOLD
};



// ============ knob and switches (hardware)
//-- Switch 1
int Switch1Pin = 1;
int Switch1State = 0;         // current state of the button
int lastSwitch1State = 0;
int Switch1Counter = 0;
//-- Switch 2
int Switch2Pin = 0;
int Switch2State = 0;         // current state of the button
int lastSwitch2State = 0;


//Volume Pot 

int VolumePotPin = 15;
float VolumePotVal; 
//-- Knob 1
int Pot1Pin = A7;
int Pot1Val ;

void setup() {
  // Audio Shield Set up ====
  AudioMemory(100);
  audioShield.enable();
  audioShield.volume(0.5);   //normal volune = 0.5


  //intitialize smooth
  for (int thisDat = 0; thisDat < numSmooth; thisDat++) {
    smooth[thisDat] = 0;

    // Prop Shield ======= Initialize IMU and filter
    imu.begin();
    filter.begin(100);
  }

  // ============ knob and switches (hardware)
  pinMode(Switch1Pin, INPUT);
  pinMode(Switch2Pin, INPUT);

 
 
  //Serial for monitoring
  //Serial.begin(9600);


  // =======SET UP AUDIO SYNTH
  // --- Voice 1 - noise
  Voice1NOISE.amplitude(1.0);
  Voice1LFO.begin(1, 3, WAVEFORM_SAWTOOTH);
  Voice1Env.amplitude(1, 1);
  Voice1filter1.frequency(6000);
  Voice1filter1.resonance(3.1);

  // --- Voice 2 - waveform
  Voice2waveform.begin(1, 1400, WAVEFORM_SINE);
  Voice2Env.amplitude(1);

  Voice2LFO.begin(0.5, 3, WAVEFORM_SINE);
  Voice2filter.frequency(1500);
  Voice2filter.resonance(4.7);
  Voice2filter.octaveControl(1);


  sine_fm1.amplitude(1);
  sine_fm1.frequency(2300);


  // -- voice 5 waveform a bit shift
  voice5Waveshift.begin(1,1410,WAVEFORM_SINE);
  voice5Filter.frequency(1500);
  voice5Filter.resonance(4.7);
  voice5Filter.octaveControl(1);
  

  //=----EFFECT session
  //bitcrusher1.bits(1);
  //waveshape1.shape(WAVESHAPE_EXAMPLE,17);
  //chorus1.begin(l_delayline,CHORUS_DELAY_LENGTH,2);


  //---- EFFECT delay
  delay1.delay(0, 60);

  //---- EFFECT reverb
  freeverb1.roomsize(0.5);
  freeverb1.damping(0.5);
}

void loop() {

  //================= MOTION in proo shield =======================
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, yaw;
  bool yawState;  // 1 is anti clockwise , 0 is clockwise
  float yawDiff;
  
  uint8_t hue, val;
  long freq;
  float vol;
  float intensity;
  // Read motion sensors and filter the results

  imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  intensity = sqrt(sq(sqrt(sq(gx) + sq(gy))) + sq(gz));

  //====smoothing the number
  if (smoothEnable == true) {
    total = total - smooth[datIndex];
    smooth[datIndex] = intensity;
    total = total + smooth[datIndex];
    datIndex = datIndex + 1;

    if (datIndex >= numSmooth) {
      datIndex = 0;
    }

    average = total / numSmooth;
    intensity = average;
  }
  if (intensity < 0) {
    intensity = 0;
  }

  intensityMap = floatMap(intensity, 0, intensityThreshold, 0, 50);

  Serial.print("intensity:");
  Serial.println(intensity);
  Serial.print("intensity Mpp:");
  Serial.println(intensityMap);
  Serial.print("Pitch:");
  Serial.print(pitch); 
  Serial.print(" --- Roll: ") ;
  Serial.print(roll);
  Serial.print(" --- Yaw: ");
  Serial.println(yaw);
  Serial.print(" ---YawPre :");
  Serial.print(yawPre);
  Serial.print("Yaw difference: ");
  yawDiff = yawPre - yaw;
  Serial.println(yawDiff);

  //Traccking rotation direction (clockwise or anti clockwise) 
  if ((yawDiff > 0.3) && (yawDiff <50.0)) { //anti-clockwise
    yawState = true;
  }

  //if (yawDiff< 0.0){
  if ((yawDiff < -0.3) && (yawDiff >-50.0)){ //Clockwise
      yawState = false;
  }

  if (abs(yawDiff) >50) {
    yawState = yawState;
  }

  //Serial.println(yawState);

  if (yawState) {
    Serial.println("ClockWise");
  } else {
    if (!yawState) { 
      Serial.println("AntiClockWise");
      }
    }
  
 delay(50);
  
  yawPre = yaw;
  


  //========= Knob value reading

  /*
  Pot1Val = analogRead(Pot1Pin);
  Serial.print("analog 16 is: ");
  Serial.println(Pot1Val);
  */
  
  /*
  Voice2waveform.frequency(900 + Pot1Val);
  sine_fm1.frequency(2500 - Pot1Val);
  Voice1filter1.frequency(7000 - Pot1Val * 3);
  */
  //Voice2waveform.frequency(900 + 500);
  sine_fm1.frequency(1300);
  //Voice1filter1.frequency(7000 - 500 * 3);
  

    //--- Volumen Knob 
    VolumePotVal = analogRead(VolumePotPin);
    VolumePotVal = VolumePotVal /1024;
    Serial.print("Volume Value:");
    Serial.println(VolumePotVal);
    audioShield.volume(VolumePotVal);
    
    

  //******=======  AUDIO SYNTH ============= ======= =======



  //Voice 1 LFO

  //Voice1LfoDC.amplitude(1,.5);
  if (yawState){
  Voice1ModMixer.gain(1, 1); //LFO mod channel
  Voice1ModMixer.gain(0, 0);  //env mod channel
  } else { 
  Voice1ModMixer.gain(1, 0); //LFO mod channel
  Voice1ModMixer.gain(0, 0);  //env mod channel
  }

  //activate Prop shield ->Voice 1 LFO freq mod filter cutoff

  
  //LFO  ON 
//Voice1LFO.amplitude(1);

  //LFO OFF
Voice1LFO.amplitude(0);  
  
  float LFOFreqFromintensity = floatMap(intensity, 0, 255, 1, 80);
  Voice1LFO.frequency(LFOFreqFromintensity);


/*
  //====Read Switch and knob
  // - - switch 1- -
  Switch1State = digitalRead(Switch1Pin);
  if (Switch1State != lastSwitch1State) {
    // if the state has changed, increment the counter
    if (Switch1State == LOW) {
      // if the current state is HIGH then the button went from off to on:

      Serial.println("Switch 1 on");
      Switch1Counter ++;
      Voice2lfoWaveShapeIndex = Switch1Counter % 4;
      Voice2LFO.begin(waveShapes[Voice2lfoWaveShapeIndex]);
      Voice2waveform.begin(waveShapes[Voice2lfoWaveShapeIndex]);
      Serial.println("waveform change to :" );

    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("Switch 1 off");
    }
    // Delay a little bit to avoid bouncing
    //delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastSwitch1State = Switch1State;


  // - - switch 2- -  for VOUCE 2 modulation on off
  Switch2State = digitalRead(Switch2Pin);
  if (Switch2State != lastSwitch2State) {
    // if the state has changed, increment the counter
    if (Switch2State == LOW) {
      // if the current state is HIGH then the button went from off to on:

      Serial.println("Switch 2 on");
      Voice2LFO.amplitude(1);
      Voice1LFO.amplitude(1);

    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("Switch 2 off");
      Voice2LFO.amplitude(0);
      Voice1LFO.amplitude(0);

    }
    // Delay a little bit to avoid bouncing
    //delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastSwitch2State = Switch2State;
*/
if (yawState){
  Voice2ModMixer.gain(1, 1); //LFO mod channel
  Voice2ModMixer.gain(0, 0);  //env mod channel
} else {
  Voice2ModMixer.gain(0, 1); //LFO mod channel
  Voice2ModMixer.gain(0, 0);  //env mod channel
}
  
  
  //Voice2LFO.frequency(LFOFreqFromintensity);


/*
  // Left Channel Mixer
  LEFTMixer.gain(0, 0.8); //noise and filter output
  LEFTMixer.gain(1, 0.1); //reverb effect
  LEFTMixer.gain(2, 0.0); //bitcrusher
  LEFTMixer.gain(3, 0.0); //delay
  */

  // RIGHT Channel Mixer
 
  if (yawState) { //CLOCKWISE
    LEFTMixer.gain(0, 1.0); //noise and filter outpu8
    LEFTMixer.gain(1, 0.0); //voice2 waveform
    LEFTMixer.gain(2, 0.0); //sineFM
    LEFTMixer.gain(3, 0.0); //voice 5 waveform shift
    
    RIGHTMixer.gain(0, 1.0); //Right Mixer Channel 1 noise and filter outpu8
    RIGHTMixer.gain(1, 0.0); //Right Mixer Channel 2 wave form
    RIGHTMixer.gain(2, 0.0); //Right Mixer Channel 3   FM sine
    RIGHTMixer.gain(3, 0.0); //voice 5 waveform shift
  } else { //AntiClockWise
     LEFTMixer.gain(0, 0.0); //noise and filter outpu8
    LEFTMixer.gain(1, 0.5); //voice2 waveform
    LEFTMixer.gain(2, 0.0); //sineFM
    LEFTMixer.gain(3, 0.5); //voice 5 waveform shift
    
    RIGHTMixer.gain(0, 0.0); //Right Mixer Channel 1 noise and filter outpu8
    RIGHTMixer.gain(1, 0.5); //Right Mixer Channel 2 wave form
    RIGHTMixer.gain(2, 0.0); //Right Mixer Channel 3   FM sine
    RIGHTMixer.gain(3, 0.5);
  }
 
  RIGHTMixer.gain(3, 0.0);

  delay(30);
}


// Function to map a flaot number
float floatMap(float x,
               float inMin,
               float inMax,
               float outMin,
               float outMax) {

  // Set bounds
  if ( x < inMin ) x = inMin;
  if ( x > inMax ) x = inMax;

  return (x - inMin) * (outMax - outMin) /
         (inMax - inMin) + outMin;
}



