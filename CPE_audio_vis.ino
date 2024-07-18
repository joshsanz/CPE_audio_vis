/* This example shows how to use the FFT library with a Circuit Playground 
 * Express. Requires installing the "Adafruit Zero FFT library" (use the 
 * Arduino Library Manager to install it!
 *
 *  The LEDs will map around the circle when frequencies between FREQ_MIN 
 *  and FREQ_MAX are detected
 */

#include <Adafruit_CircuitPlayground.h>
#include "Adafruit_ZeroFFT.h"
#include <FastLED.h>
#include <math.h>

//this must be a power of 2
#define DATA_SIZE 512

#define NUM_PIXELS 10

//the sample rate
#define FS 22000

//the lowest frequency that will register on the meter
#define FREQ_MIN 100

//the highest frequency that will register on the meter
#define FREQ_MAX 3000

//compensate for the ZeroFFT writer not knowing how DFTs work...
#define MIN_INDEX FFT_INDEX(FREQ_MIN, FS, DATA_SIZE / 2)
#define MAX_INDEX FFT_INDEX(FREQ_MAX, FS, DATA_SIZE / 2)

#define SCALE_FACTOR 64

int16_t pixelData[NUM_PIXELS];
int16_t inputData[DATA_SIZE];
int16_t smoothedFFT[DATA_SIZE / 2];
uint16_t fft2pix_map[DATA_SIZE / 2];
uint16_t fft2pix_scaling[NUM_PIXELS];
CRGB fastleds[NUM_PIXELS];

const uint8_t
  // R,G,B values for color wheel covering 10 NeoPixels:
  reds[]   = { 0xAD, 0x9A, 0x84, 0x65, 0x00, 0x00, 0x00, 0x00, 0x65, 0x84 },
  greens[] = { 0x00, 0x66, 0x87, 0x9E, 0xB1, 0x87, 0x66, 0x00, 0x00, 0x00 },
  blues[]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0xE4, 0xFF, 0xE4, 0xC3 },
  gamma8[] = { // Gamma correction improves the appearance of midrange colors
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05, 0x05, 0x05, 0x06,
    0x06, 0x06, 0x06, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09,
    0x0A, 0x0A, 0x0A, 0x0B, 0x0B, 0x0B, 0x0C, 0x0C, 0x0D, 0x0D, 0x0D, 0x0E,
    0x0E, 0x0F, 0x0F, 0x10, 0x10, 0x11, 0x11, 0x12, 0x12, 0x13, 0x13, 0x14,
    0x14, 0x15, 0x15, 0x16, 0x16, 0x17, 0x18, 0x18, 0x19, 0x19, 0x1A, 0x1B,
    0x1B, 0x1C, 0x1D, 0x1D, 0x1E, 0x1F, 0x1F, 0x20, 0x21, 0x22, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2A, 0x2B, 0x2C, 0x2D,
    0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x44, 0x45, 0x46,
    0x47, 0x48, 0x49, 0x4B, 0x4C, 0x4D, 0x4E, 0x50, 0x51, 0x52, 0x54, 0x55,
    0x56, 0x58, 0x59, 0x5A, 0x5C, 0x5D, 0x5E, 0x60, 0x61, 0x63, 0x64, 0x66,
    0x67, 0x69, 0x6A, 0x6C, 0x6D, 0x6F, 0x70, 0x72, 0x73, 0x75, 0x77, 0x78,
    0x7A, 0x7C, 0x7D, 0x7F, 0x81, 0x82, 0x84, 0x86, 0x88, 0x89, 0x8B, 0x8D,
    0x8F, 0x91, 0x92, 0x94, 0x96, 0x98, 0x9A, 0x9C, 0x9E, 0xA0, 0xA2, 0xA4,
    0xA6, 0xA8, 0xAA, 0xAC, 0xAE, 0xB0, 0xB2, 0xB4, 0xB6, 0xB8, 0xBA, 0xBC,
    0xBF, 0xC1, 0xC3, 0xC5, 0xC7, 0xCA, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD7,
    0xDA, 0xDC, 0xDF, 0xE1, 0xE3, 0xE6, 0xE8, 0xEB, 0xED, 0xF0, 0xF2, 0xF5,
    0xF7, 0xFA, 0xFC, 0xFF };

// Accumulate FFT values into pixels with log spacing
// to match human perception better (see Mel spectrogram)
void fill_fft2pix_map(uint16_t fftmin, uint16_t fftmax, uint16_t numpix) {
  uint16_t N = fftmax - fftmin + 1;
  float *logVals = (float*)calloc(DATA_SIZE / 2, sizeof(float));
  // Turn [fftmin, fftmax] into log([*,*]) to respace
  for(uint16_t i=0; i<N; i++) {
    logVals[i + fftmin] = log10f((float) i + fftmin);
  }
  // Everything before min index ==> min index
  for(uint16_t i=0; i<fftmin; i++)
    logVals[i] = logVals[fftmin];
  // Everything after max index ==> max index
  for(uint16_t i=fftmax; i<DATA_SIZE / 2; i++)
    logVals[i] = logVals[fftmax];
  
  // Map arbitrary data range into 0-numpix
  // Gather min/max vals
  float minval = logVals[0];
  float maxval = logVals[0];
  for(uint16_t i=0; i<DATA_SIZE / 2; i++) {
    if(logVals[i] < minval) 
      minval = logVals[i];
    if(logVals[i] > maxval) 
      maxval = logVals[i];
  }

  // Perform mapping
  float scaling = (numpix - 1) / (maxval - minval);
  for(uint16_t i=0; i<DATA_SIZE / 2; i++) {
    logVals[i] = (logVals[i] - minval) * scaling;
  }

  // Truncate and store pixel assignments for fft bins
  memset(fft2pix_scaling, 0, sizeof(fft2pix_scaling));
  for(uint16_t i=0; i<DATA_SIZE / 2; i++) {
    fft2pix_map[i] = (uint16_t) logVals[i];
    // Increment counter for bins assigned to this pixel
    fft2pix_scaling[fft2pix_map[i]] += 1;
  }

  // Calculate scalings to approximately equalize based on number of bins assigned
  uint16_t mincnt = fft2pix_scaling[0];
  uint16_t maxcnt = fft2pix_scaling[0];
  for(uint16_t i=0; i<numpix; i++) {
    if(fft2pix_scaling[i] < mincnt) mincnt = fft2pix_scaling[i];
    if(fft2pix_scaling[i] > maxcnt) maxcnt = fft2pix_scaling[i];
  }
  uint32_t target = mincnt * maxcnt;
  for(uint16_t i=0; i<numpix; i++) {
    fft2pix_scaling[i] = target / fft2pix_scaling[i];
  }

  // Cleanup
  free(logVals);
}

// the setup routine runs once when you press reset:
void setup() {
  CircuitPlayground.begin();
  // Serial.begin(115200);
  // size_t delays = 0;
  // while(!Serial && delays < 10){
  //   delay(10);
  //   delays++;
  // }
  fill_fft2pix_map(MIN_INDEX, MAX_INDEX, NUM_PIXELS);
  FastLED.addLeds<NEOPIXEL, 8>(fastleds, NUM_PIXELS);
}

// Track low and high levels for dynamic adjustment
int minLvl = 0, maxLvl = 1000;

void loop() {
  int i;
  CircuitPlayground.mic.capture(inputData, DATA_SIZE);

  // Center data on average amplitude
  int32_t avg = 0;
  for(i=0; i<DATA_SIZE; i++) avg += inputData[i];
  avg /= DATA_SIZE;
  // Scale for FFT
  for(i=0; i<DATA_SIZE; i++)
    inputData[i] = (inputData[i] - avg) * SCALE_FACTOR;
  
  // Run the FFT
  ZeroFFTMagnitude(inputData, DATA_SIZE, true); // Only writes positive frequency values

  // EMA smoothed FFT bins
  for(i=0; i<DATA_SIZE / 2; i++){
    int32_t val = (int32_t)smoothedFFT[i] * 3 + inputData[i] + 3;
    smoothedFFT[i] = (int16_t)(val >> 2);
  }

  // Sum smoothedFFT[] (FFT smoothed output) into pixelData[] bins.
  memset(pixelData, 0, sizeof pixelData); // Clear pixelData[] buffer
  //downsample into NUM_PIXELS bins
  for(int i=MIN_INDEX; i<MAX_INDEX; i++){
    pixelData[fft2pix_map[i]] += smoothedFFT[i];
  }
  //normalize by assigned bins
  for(int i=0; i<NUM_PIXELS; i++) {
    pixelData[i] *= fft2pix_scaling[i];
  }

  // Figure the max and min levels for the current frame
  int most = pixelData[0], least = pixelData[0];
  for(i=1; i<NUM_PIXELS; i++) {
    if(pixelData[i] > most)  most = pixelData[i];
    if(pixelData[i] < least) least = pixelData[i];
  }
  // Always have some minimum space between, else it's too "jumpy"
  if((most - least) < 15) most = least + 15;

  // Dynamic max/min is sort of a fake rolling average...
  maxLvl = (most > maxLvl) ?
    (maxLvl *  3 + most +  3) /  4 :  // Climb fast
    (maxLvl * 31 + most + 31) / 32;   // Fall slow
  minLvl = (least < minLvl) ?
    (minLvl *  3 + least +  3) /  4 : // Fall fast
    (minLvl * 31 + least + 31) / 32;  // Climb slow
  // DEBUG
  // Serial.print(minLvl);
  // Serial.print(" ");
  // Serial.println(maxLvl);

  //display the data
  int n;
  for(i=0; i<NUM_PIXELS; i++) {
    uint16_t hue = (i * 192) / NUM_PIXELS;

    // Scale pixel data to 0-511ish range based on dynamic levels
    n = map(pixelData[i], minLvl, maxLvl, 0, 511);
    if(n < 0)        n = 0;
    else if(n > 511) n = 511;

    // Scale value from 0 to 255 if n in lower half
    // Otherwise scale down saturation to whiten color
    uint8_t value = 255;
    uint8_t saturation = 255;
    if(n <= 255) {
      value = n;
    } else {
      saturation = (255 + n) - 512;
    }
    fastleds[i].setHSV(hue, saturation, value);
  }
  FastLED.show();
}
