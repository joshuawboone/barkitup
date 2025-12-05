/*
* ZettaLights Controller program written by Josh Boone (Zettabit Fox) 
* Converted for SAMD21 from original ARM. 
*
* Uses ArduinoFFT library for better FFT performance on ARM Cortex-M0+
*
*
* VERSION 2.1 SAMD21        COMPILED DEC 4, 2025
*
* Updated to 8 colors with 16 frequency response ranges
* Frequency response calculated to roughly 94-4200 Hz
*/

//USER DEFINITIONS - CHANGE as indicated/needed 
#define posSize 7  // CHANGE THIS VALUE TO MATCH NUMBER OF LINES IN pos ARRAY MINUS 1. Used for streak and static modes. 7 = 0-7 for 8 zones to match number of colors. 
#define NUM_LEDS 69 // CHANGE THIS VALUE TO MATCH LONGEST RUN OF SERIES LEDS 

// LED and FFT definitions
#define LIN_OUT 1   // Do not change, for FFT
#define NUM_STRIPS 1 // Do not change 
#define ADC_CHANNEL 2  // Do not change, for MIC Input
#define SCALE 128  // Do not change, for LEDs


// Mode definitions
#define MUSIC_RANDOM 0   // Music Mode 1 - Random LEDs
#define MUSIC_STATIC 1   // Music Mode 2 - Static visualizer
#define SOLID_COLOR 2    // Solid Color Mode
#define RAINBOW 3        // Rainbow Mode
#define STREAK 4         // Colored Streaks Mode

// Button handling constants
#define DEBOUNCE_TIME 50  // milliseconds
#define HOLD_TIME 1000    // milliseconds for long press

// FFT Configuration - Optimized for SAMD21 performance
#define SAMPLES 128             // Power of 2 - required by ArduinoFFT 256 introduced lag in processing so increased frequency instead. 
#define SAMPLING_FREQUENCY 12000 // Hz, increased for better high-freq response up from 10k 

// Libraries
#include <arduinoFFT.h>
#include "math.h"
#include "Wire.h"
#include "FastLED.h"

// Forward declarations
void setup();
void loop();

/* MODIFY VARIABLES
These are the variables that change based on number of LEDs or dev board used.
*/

byte buttonPin = 2; // Button pin using Gemma M0 mappings, same for custom controller
byte modeVal = MUSIC_STATIC;  // the mode the controller starts in
byte ledPin = 0; // pin for LED data using Gemma M0 mappings, same for custom controller. If changing this is required, change the controller setup in the setup() function. This is just a place-holder. -> controllers[0] = &FastLED.addLeds<NEOPIXEL,0>(leds, NUM_LEDS);
byte gBrightness = 32;  // brightness value - lower depending on number of LEDs and power usage. (0-255)

byte fftFadeRateActive = 120; // default fade rate for LEDs when activity on FFT processing (0-255) 100, 120, 140, 160, 180, 200 (Higher number = sharper visuals meaning the activity looks more responsive vs the slower fade out)
byte fftFadeRateInactive = 160; // default fade rate for LEDs when no activity on FFT processing. Higher than active (0-255) 140, 160, 180, 200, 220, 240

byte randomNum = 3; // Number of LEDs in a group for the Random FFT mode. Random LED is selected, it counts as 1 here, so currently 3 LEDs of the same color will light sequentially when triggered. 
byte staticOffset = 4; // Offset for the specrum analyzer mode (staticize2 function) to be changed depending on number of LEDs per zone defined below

// CHANGE THIS ARRAY TO MATCH DISTRIBUTION OF NUMBER OF LEDS - LED/COLOR ZONES MAPPING
// Say if you had 56, change these and add as needed to make steps clean 0,6 7,13 14,20 ...

// Don't forget to change the posSize definition at the top.
// 8-color LED zone mapping - adjust these arrays based on your LED layout
// I mapped this to 8 zones to match number of colors across the length of the longest series of LEDs. Advise keeping at 8 for zones or multiples of 8 for repeating patterns on long runs of LEDs. 

// Change the staticOffset variable above to the number of LEDs you want the offset frequencies to show as in Static FFT mode. 
// e.g. it's set for 4 currently since the array below is 7 LEDs per zone. 
// This creates the effect in Static mode of 16 frequency range responses using the 8 colors. When the same color is reused, it skips this number of LEDs in the zone to allow both to show up.

byte pos[8][2] = {
  {0,7},   // Red zone
  {8,15},   // Red-Orange zone  
  {16,23},  // Orange zone
  {24,32}, // Yellow zone
  {33,41}, // Green zone
  {42,50}, // Blue zone
  {51,59}, // Indigo zone
  {60,68}  // Violet zone
};

// SHOULD NOT NEED TO CHANGE ANYTHING BELOW HERE
// Feel free to check for accuracy and let me know if I messed something up. 
// <3

// Set up FastLED Library
CRGB leds[NUM_LEDS];
CLEDController *controllers[NUM_STRIPS];

// Complete function prototypes
void checkButton();
void toggleLock();
void redFlash();
void grnFlash();
void cycleMode();
void adjustSensitivity();
void fastFT();
void randomize();
void staticize();
void staticize2();
void solid();
void crossFade();
int calculateStep(int prevValue, int endValue);
int calculateVal(int step, int val, int i);
void rainbow();
void streak();
void runStreakCycle(CRGB color1, CRGB color2);
void setupADC();
void processAudioSamples();

// Button handling variables
unsigned long lastDebounceTime = 0;
byte lastButtonState = HIGH;
byte buttonState = HIGH;
bool buttonPressed = false;
bool lockVal = false;
unsigned long buttonPressStartTime = 0;


// Global timing variables
unsigned long previousPatternMillis = 0;  // For pattern updates
unsigned long previousFadeMillis = 0;     // For crossfade updates
unsigned long previousSensMillis = 0;     // For sensitivity updates
unsigned long patternInterval = 300;      // Default time between pattern updates (ms) - faster for SAMD21
unsigned long fadeInterval = 2000;        // Microseconds between fade steps - faster for SAMD21
unsigned long buttonMillis = 0;           // For button handling

// Pattern state variables
byte currentStep = 0;                // Current step in multi-step patterns
byte currentColor1Index = 0;         // For streak patterns
byte currentPos1 = 0;                // For streak patterns
byte currentPos2 = 0;                // For streak patterns
bool streakCycleComplete = true;     // For streak patterns
int fadeStep = 0;                    // Current step in crossfade

// Mode and sensitivity settings
byte sensCount = 0;
float sensMult = 1.2;  // Initial sensitivity
unsigned long sensTimer = 0;

// Variables for dynamic sensitivity adjustment
unsigned long lastSensAdjustTime = 0;
const unsigned long SENS_ADJUST_INTERVAL = 1000; // Adjust sensitivity every 1 second
int activeCount = 0; // How many times we're detecting activity
int sampleCount = 0; // How many samples we've taken
const float TARGET_PERCENTAGE = 15.0; // Target activity level

// Color values
byte c = 0;
int redVal, grnVal, bluVal, prevR, prevG, prevB;
// Color count array for 8 colors
int colorCount[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// FFT variables using ArduinoFFT
double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Updated frequency band data for 8 colors - adjusted for 128-sample FFT at 12kHz
// Each array contains: [start_bin, end_bin, scale_factor]
// Bin resolution: 93.75 Hz per bin
// Main frequency bands - 8 colors with better resolution
const uint8_t frequencyBands[8][3] = {
  {1, 2, 130},   // Red: ~94-187 Hz (sub-bass/kick drums)
  {3, 4, 110},   // Red-Orange: ~281-375 Hz (bass fundamentals)
  {5, 6, 90},    // Orange: ~469-563 Hz (low-mids/bass harmonics)
  {7, 9, 75},    // Yellow: ~656-844 Hz (vocal fundamentals)
  {10, 13, 60},  // Green: ~937-1219 Hz (presence range)
  {14, 19, 45},  // Blue: ~1313-1781 Hz (clarity/definition)
  {20, 28, 30},  // Indigo: ~1875-2625 Hz (brightness/sibilance)
  {29, 40, 15}   // Violet: ~2719-3750 Hz (air/sparkle/cymbals)
};

// Offset bands for STATIC mode - fills gaps and adds harmonics
const uint8_t frequencyBandsOffset[8][3] = {
  {2, 2, 125},   // Red: ~187 Hz (bass transition)
  {4, 5, 100},   // Red-Orange: ~375-469 Hz (bass-to-mid bridge)
  {6, 7, 85},    // Orange: ~563-656 Hz (low-mid body)
  {8, 10, 70},   // Yellow: ~750-937 Hz (vocal harmonics)
  {11, 15, 55},  // Green: ~1031-1406 Hz (vocal brightness)
  {16, 22, 40},  // Blue: ~1500-2063 Hz (definition range)
  {23, 32, 25},  // Indigo: ~2156-3000 Hz (high harmonics)
  {33, 45, 10}   // Violet: ~3094-4219 Hz (ultra-high sparkle)
};

// Used for EQ evaluation on the mic - adjusted for SAMD21
// Updated noise array for 128 samples (64 useful bins)
// Adjusted for 12kHz sampling and extended frequency range
static const uint8_t
noise[64] = { 
  16,14,12,10,9,10,10,10,9,10,10,9,8,9,9,10,     // 0-15: Very low frequencies
  7,7,8,7,8,8,8,8,7,8,9,7,8,9,10,10,            // 16-31: Low frequencies  
  9,8,8,8,8,8,8,7,9,8,8,8,8,8,8,8,              // 32-47: Mid frequencies
  8,8,8,8,8,8,8,8,8,8,8,8,8,9,9,10              // 48-63: High frequencies
};

// Updated EQ array for 128 samples (64 useful bins)
// Adjusted scaling quotients for 12kHz sampling
static const uint16_t
eq[64] = {
  300, 250, 300, 300, 270, 240, 190, 170, 130, 120, 150, 150, 150, 190, 270, 120,  // 0-15: Enhanced low freq
  170, 200, 210, 220, 220, 220, 200, 190, 170, 170, 150, 140, 130, 120, 110, 110,  // 16-31: Low-mid transition
  100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, // 32-47: Mid frequencies
  105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 110   // 48-63: High frequencies
};


// Per-frequency sensitivity multipliers for 8 colors. Initial values. 
float freqSensMultipliers[8] = {
  1.5,  // Red
  1.5,  // Red-Orange  
  1.6,  // Orange
  1.5,  // Yellow
  1.8,  // Green
  1.5,  // Blue
  1.5,  // Indigo
  1.5   // Violet
};

// 8-color RGB values
byte colors[8][3] = {
  {255,0,0},      // Red - Sub-bass
  {255,69,0},     // Red-Orange - Bass
  {255,140,0},    // Orange - Low-mids
  {255,255,0},    // Yellow - Vocal fundamentals
  {0,255,0},      // Green - Presence
  {0,0,255},      // Blue - Clarity
  {75,0,130},     // Indigo - Brightness
  {148,0,211}     // Violet - Air/sparkle
};

//Setup code to initialize controller, MIC, and LED strip.
void setup() 
{
  //Serial.begin(115200);
  //delay(2000); // Give time for serial monitor to connect
  
  // FastLED setup
  controllers[0] = &FastLED.addLeds<NEOPIXEL,0>(leds, NUM_LEDS);
  FastLED.setBrightness(gBrightness);
  
  //Button Setup
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Initialize ADC for audio sampling
  setupADC();
  
  // Initialize arrays
  for(int i = 0; i < SAMPLES; i++){
    vReal[i] = 0.0;
    vImag[i] = 0.0;
  }
  
  // Test LED strip
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  //Serial.println("ZettaLights SAMD21 Initialized");
  //Serial.print("Starting in mode: ");
  //Serial.println(modeVal);
}

//This is the main loop that runs to switch modes
void loop() 
{
  checkButton();
  
  switch(modeVal) {
    case MUSIC_RANDOM:
    case MUSIC_STATIC:
      fastFT();
      break;
    case SOLID_COLOR:
      solid();
      break;
    case RAINBOW:
      rainbow();
      break;
    case STREAK:
      streak();
      break;
  }
}

void setupADC() {
  // Use Arduino's analogRead setup, but configure for faster sampling
  analogReference(AR_DEFAULT); 
  analogReadResolution(12); // 12-bit resolution (0-4095)
  
  // Pre-read to initialize ADC
  analogRead(A0);
  delay(10);
  
  //Serial.println("ADC initialized for A0");
}

// Audio Sampling function. Updated for 12kHz.
void processAudioSamples() {
  static unsigned long lastSampleTime = 0;
  unsigned long samplePeriod = 83; // 12000Hz = ~83.33us between samples
  static double dcOffset = 2048.0; // Running average for DC offset removal
  
  // Collect audio samples at controlled rate
  for(int i = 0; i < SAMPLES; i++) {
    // Simple timing control - tighter timing for 12kHz
    while(micros() - lastSampleTime < samplePeriod) {
      // Minimal delay
    }
    lastSampleTime = micros();
    
    // ADC read
    uint16_t sample = analogRead(A0);
    
    // Update DC offset (low-pass filter)
    dcOffset = dcOffset * 0.99 + sample * 0.01;
    
    // Convert 12-bit ADC to signed value with DC offset removal
    vReal[i] = (double)(sample - dcOffset);
    vImag[i] = 0.0;
  }
  
  // Apply window function and perform FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
  
  // Apply noise subtraction and EQ scaling - process 64 bins (Nyquist limit)
  for(int i = 0; i < 64; i++) {
    if (i < 1) {
      vReal[i] = 0; // Zero DC bin
    } else {
      // Subtract noise floor
      vReal[i] = max(0.0, vReal[i] - noise[i] * 2.4); // Increased noise subtraction for 12kHz
      
      // Apply EQ scaling (divide by EQ value to normalize)
      if (eq[i] > 0) {
        vReal[i] = vReal[i] / eq[i] * 170; // Increased scaling for better sensitivity at 12kHz
      }
    }
  }
}

void checkButton() {
  byte reading = digitalRead(buttonPin);
  unsigned long currentMillis = millis();
  
  // If the switch changed, due to bounce or pressing...
  if (reading != lastButtonState) {
    // Reset the debouncing timer
    lastDebounceTime = currentMillis;
  }
  
  if ((currentMillis - lastDebounceTime) > DEBOUNCE_TIME) {
    // If the reading has been stable longer than the debounce delay
    
    // If button state changed
    if (reading != buttonState) {
      buttonState = reading;
      
      // Button just pressed
      if (buttonState == LOW) {
        buttonPressed = true;
        buttonPressStartTime = currentMillis;
      } 
      // Button just released
      else if (buttonPressed) {
        buttonPressed = false;
        
        // If it was a short press and system is unlocked
        if ((currentMillis - buttonPressStartTime < HOLD_TIME) && (lockVal == false)) {
          cycleMode();
        }
      }
    }
    
    // Check for long press while button is still down
    if (buttonState == LOW && buttonPressed && 
        (currentMillis - buttonPressStartTime >= HOLD_TIME)) {
      // Toggle lock only once per long press
      buttonPressed = false;
      toggleLock();
    }
  }
  
  lastButtonState = reading;
}

// Toggle between locked and unlocked states
void toggleLock() {
  if (lockVal == false) {
    lockVal = true;
    redFlash();
  } else {
    lockVal = false;
    grnFlash();
  }
}

// Flash LEDs red to indicate locked status
void redFlash() {
  byte flashCount = 0;
  bool ledOn = true;
  unsigned long flashStartTime = millis();
  unsigned long flashInterval = 120; // 75ms on, 75ms off
  
  while(flashCount < 2) {
    unsigned long currentMillis = millis();
    
    // Toggle state every flashInterval ms
    if(currentMillis - flashStartTime >= flashInterval) {
      flashStartTime = currentMillis;
      
      if(ledOn) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
        FastLED.show();
        flashCount++; // Count a completed cycle
      }
      
      ledOn = !ledOn; // Toggle state
    }
  }
}

// Flash LEDs green to indicate unlocked status
void grnFlash() {
  byte flashCount = 0;
  bool ledOn = true;
  unsigned long flashStartTime = millis();
  unsigned long flashInterval = 120; // 75ms on, 75ms off
  
  while(flashCount < 2) {
    unsigned long currentMillis = millis();
    
    // Toggle state every flashInterval ms
    if(currentMillis - flashStartTime >= flashInterval) {
      flashStartTime = currentMillis;
      
      if(ledOn) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Green);
        FastLED.show();
        flashCount++; // Count a completed cycle
      }
      
      ledOn = !ledOn; // Toggle state
    }
  }
}

// Cycle through the modes
void cycleMode()
{
  modeVal = (modeVal + 1) % 5; // Cycle through 5 modes (0-4)
  //Serial.print("Mode changed to: ");
  //Serial.println(modeVal);
}

// Improved auto sensitivity adjustment
void adjustSensitivity() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensAdjustTime >= SENS_ADJUST_INTERVAL) {
    lastSensAdjustTime = currentTime;
    
    float activePercentage = (sampleCount > 0) ? (float)activeCount / sampleCount * 100.0 : 0;
    
    //Serial.print("Activity: ");
    //Serial.print(activePercentage);
    //Serial.print("%, Sensitivity: ");
    //Serial.println(sensMult);
    
    if (activePercentage > (TARGET_PERCENTAGE + 5.0)) {
      sensMult = min(sensMult * 1.1, 10.0);
    } 
    else if (activePercentage < (TARGET_PERCENTAGE - 5.0)) {
      sensMult = max(sensMult * 0.9, 0.1);
    }
    
    // Adjust individual frequency band sensitivity (8 colors)
    for(int x = 0; x < 8; x++) {
      if (colorCount[x] > (activeCount / 8)) {
        freqSensMultipliers[x] = min(freqSensMultipliers[x] * 1.05, 10.0);
      }
      else if (colorCount[x] < (activeCount / 16)) {
        freqSensMultipliers[x] = max(freqSensMultipliers[x] * 0.95, 0.1);
      }
      colorCount[x] = 0;
    }
    
    activeCount = 0;
    sampleCount = 0;
  }
}

// Fast Fourier Transform for audio analysis
void fastFT()
{
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  unsigned long lastFFTUpdateTime = 0;
  unsigned long fftUpdateInterval = (modeVal == MUSIC_STATIC) ? 15 : 25;
  
  //Serial.println("Starting FFT mode (128 samples @ 12kHz, 8 colors)");
  
  do {
    checkButton();
    
    unsigned long currentMillis = millis();
    if (currentMillis - lastFFTUpdateTime < fftUpdateInterval) {
      delay(1);
      continue;
    }
    lastFFTUpdateTime = currentMillis;
    
    processAudioSamples();
    bool anyActivity = false;
    
    // Process main frequency bands (8 colors)
    for(int band = 0; band < 8; band++) {
      double magnitude = 0;
      int startBin = frequencyBands[band][0];
      int endBin = frequencyBands[band][1];
      int scale = frequencyBands[band][2];
      
      // Bounds checking
      if (endBin >= 64) endBin = 63;
      if (startBin >= endBin) continue;
      
      // Sum magnitude in frequency band
      for(int i = startBin; i <= endBin; i++) {
        magnitude += vReal[i];
      }
      
      magnitude = magnitude / (endBin - startBin + 1);
      magnitude = magnitude / scale;
      
      sampleCount++;
      
      // Apply sensitivity thresholds
      double threshold = 50.0 * sensMult * freqSensMultipliers[band];
      
      if(magnitude >= threshold) {
        c = band;
        activeCount++;
        colorCount[band]++;
        anyActivity = true;
        
        if (modeVal == MUSIC_RANDOM) {
          randomize();
        } else {
          staticize();
        }
      }
    }
    
    // Process offset frequency bands only in MUSIC_STATIC mode
    if (modeVal == MUSIC_STATIC) {
      for(int band = 0; band < 8; band++) {
        double magnitude = 0;
        int startBin = frequencyBandsOffset[band][0];
        int endBin = frequencyBandsOffset[band][1];
        int scale = frequencyBandsOffset[band][2];
        
        // Bounds checking
        if (endBin >= 64) endBin = 63;
        if (startBin >= endBin) continue;
        
        // Sum magnitude in offset frequency band
        for(int i = startBin; i <= endBin; i++) {
          magnitude += vReal[i];
        }
        
        magnitude = magnitude / (endBin - startBin + 1);
        magnitude = magnitude / scale;
        
        sampleCount++;
        
        // Higher threshold for offset bands
        double threshold = 65.0 * sensMult * freqSensMultipliers[band];
        
        if(magnitude >= threshold) {
          c = band;
          activeCount++;
          colorCount[band]++;
          anyActivity = true;
          
          staticize2();
        }
      }
    }
    
    FastLED.show();
    
    // Fade LEDs
    if (anyActivity) {
      fadeToBlackBy(leds, NUM_LEDS, fftFadeRateActive);
    } else {
      fadeToBlackBy(leds, NUM_LEDS, fftFadeRateInactive);
    }
    
    adjustSensitivity();

  } while(modeVal == MUSIC_RANDOM || modeVal == MUSIC_STATIC);
  
  //Serial.println("Exiting FFT mode");
}

// Randomize LED location for a specified frequency
void randomize()
{
  byte randomLed = random(0, NUM_LEDS);
  //leds[randomLed].setRGB(colors[c][0], colors[c][1], colors[c][2]);
  byte j = 0;
  while(j < randomNum && randomLed < NUM_LEDS) {
      leds[randomLed].setRGB(colors[c][0], colors[c][1], colors[c][2]);
      j++;
      randomLed++;
    }
}

// Set static location for a specified frequency (spectrum analyzer)
void staticize()
{

  byte j = pos[c][0];

  while(j <= pos[c][1]) {
      leds[j].setRGB(colors[c][0], colors[c][1], colors[c][2]);
      j++;
    }
}

// Frequency offsets
void staticize2()
{
  byte j = pos[c][0];
  j += staticOffset;

  while(j <= pos[c][1]) {
      leds[j].setRGB(colors[c][0], colors[c][1], colors[c][2]);
      j++;
    }
}

// Sets LEDs to fade from one to the next starting at Red. 
void solid() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  c = 0;
  fadeStep = 0;
  
  while(modeVal == SOLID_COLOR) {
    crossFade();
    
    if(fadeStep >= 1020) {
      fadeStep = 0;
      c++;
      if (c > 7) c = 0;
    }
    
    checkButton(); //potentially redundant? check/verify stepping. function is called within child crossFade() function... 
  }
}

void crossFade() {
  unsigned long currentMicros = micros();
  
  int R = colors[c][0];
  int G = colors[c][1];
  int B = colors[c][2];

  int stepR = calculateStep(prevR, R);
  int stepG = calculateStep(prevG, G); 
  int stepB = calculateStep(prevB, B);

  if (currentMicros - previousFadeMillis >= fadeInterval) {
    previousFadeMillis = currentMicros;
    
    redVal = calculateVal(stepR, redVal, fadeStep);
    grnVal = calculateVal(stepG, grnVal, fadeStep);
    bluVal = calculateVal(stepB, bluVal, fadeStep);

    fill_solid(leds, NUM_LEDS, CRGB(redVal, grnVal, bluVal));
    FastLED.show();
    
    fadeStep++;
    
    if (fadeStep >= 1020) {
      prevR = redVal; 
      prevG = grnVal; 
      prevB = bluVal;
    }
  }
  
  checkButton(); //potentially redundant? check/verify stepping. function is called within parent solid() function...
}

// Function for calculating fade steps
int calculateStep(int prevValue, int endValue) 
{
  int step = endValue - prevValue;
  if (step) {
    step = 1020/step;              
  } 
  return step;
}

//Function for calculating RGB values
int calculateVal(int step, int val, int i) {
  if ((step) && i % step == 0) {
    if (step > 0) {
      val += 1;           
    } 
    else if (step < 0) {
      val -= 1;
    } 
  }
  if (val > 255) {
    val = 255;
  } 
  else if (val < 0) {
    val = 0;
  }
  return val;
}

// Rainbow effect with scrolling colors
void rainbow() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  currentStep = 0;
  float steps = 21.25;
  
  while(modeVal == RAINBOW) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousPatternMillis >= patternInterval) {
      previousPatternMillis = currentMillis;
      
      fill_rainbow(leds, NUM_LEDS, steps * (12 - currentStep), 5);
      FastLED.show();
      
      currentStep = (currentStep + 1) % 12;
    }
    
    checkButton();
  }
}

// Streaks of rainbow colors
void streak() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  currentColor1Index = 0;
  streakCycleComplete = true;
  
  while(modeVal == STREAK) {
    switch(currentColor1Index) {
      case 0:
        runStreakCycle(CRGB::Red, CRGB(255,69,0)); // Red to Red-Orange
        break;
      case 1:
        runStreakCycle(CRGB(255,140,0), CRGB::Yellow); // Orange to Yellow
        break;
      case 2:
        runStreakCycle(CRGB::Green, CRGB::Blue); // Green to Blue
        break;
      case 3:
        runStreakCycle(CRGB(75,0,130), CRGB(148,0,211)); // Indigo to Violet
        break;
    }
    
    if(streakCycleComplete) {
      currentColor1Index = (currentColor1Index + 1) % 4; // 4 streak cycles
    }
    
    checkButton();
  }
}

void runStreakCycle(CRGB color1, CRGB color2) {
  if (streakCycleComplete) {
    currentPos1 = 0;
    currentPos2 = posSize;
    streakCycleComplete = false;
  }
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousPatternMillis >= patternInterval && 
      currentPos1 <= posSize && currentPos2 >= 0) {
    previousPatternMillis = currentMillis;
    
    byte y = pos[currentPos2][0];
    byte j = pos[currentPos1][0];
    
    while(j <= pos[currentPos1][1] && j < NUM_LEDS) {
      leds[j++] = color1;
    }
    
    while(y <= pos[currentPos2][1] && y < NUM_LEDS) {
      leds[y++] = color2;
    }
    
    FastLED.show();
    fadeToBlackBy(leds, NUM_LEDS, 128);
    
    currentPos1++;
    currentPos2--;
    
    if(currentPos1 > posSize || currentPos2 < 0) {
      streakCycleComplete = true;
    }
  }
}