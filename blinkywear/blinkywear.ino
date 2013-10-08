#include <TimerOne.h>
#include "LPD6803.h"

//////////////////////////////
// blinkywear
// author: rfong
//
// LPD6803 RGB LED example by Bliptronics.com Ben Moyes 2009
// Code cleaned up and Object-ified by ladyada, should be a bit easier to use

//user config
int TOTAL_LENGTH = 13;                // number of addressable units in strip
int segments[2][2] = {{6,0}, {12,7}}; // for multi-segment patterns
int numModes = 6;                     // how many modes I'm set to cycle through

// strip
int dataPin = 3;
int clockPin = 4;
LPD6803 strip = LPD6803(TOTAL_LENGTH+1, dataPin, clockPin);

// globals for color/position tracking
int global_wheel_offset = 0;
int pixel_offset = 0;

// test LEDs
int testPin = 13;
int redTestPin = 12;

// audio (NOT CURRENTLY USED)
int msgeqPin = A0;   //the analog output from MSGEQ chip
int strobePin = 10; //MSGEQ strobe
int resetPin = 11;  //MSGEQ reset
int* spectrumValue;

// switching modes
int interruptButton = 0;
volatile boolean restartLoop = 0;
int mode = 0;
void buttonPressed() {
  mode = (mode+1) % numModes;
  restartLoop = 1; //we add a check for this inside rainbow() since it's such a long cycle
}


/* main */

void setup() {  
  pinMode(testPin, OUTPUT);
  digitalWrite(testPin, LOW);
  pinMode(redTestPin, OUTPUT);
  digitalWrite(redTestPin, LOW);
  
  // set up MSGEQ pins
  pinMode(strobePin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  digitalWrite(strobePin, HIGH);
  analogReference(DEFAULT);
  
  // set up button interrupt
  attachInterrupt(interruptButton, buttonPressed, RISING);
  
  strip.setCPUmax(60);  // up this if the strand flickers or is slow
  strip.begin();
  strip.show();
}

void loop() {  
  restartLoop = 0;
  
  switch(mode) {
    case 1:
      // helix up
      helix(40, 5, true);
      break;
    case 2:
      // helix down
      helix(40, 5, false);
      break;
    case 3:
      // long segment bounce
      continuousRainbowBounce(40, 0, TOTAL_LENGTH-1);
      break;
    case 4:
      // twinkle alternating pixels
      helix(100, 1, true);
      break;
    case 5:
      // retina-burning strobe
      strobe(100, 500);
      break;
    default: //mode 0
      // continuous color cycle
      rainbow(50);
  }
  
  //unused
  //multiSegmentRainbowBounce(40, segments);
  //continuousRainbowFill(50, 0, TOTAL_LENGTH-1); set(0);
}


/* fancy patterns */

void wompPattern(int first, int last) {
  wompwomp(first, last);
  
  continuousRainbowFill(50, last, first); // fill back toward the beginning
  for (int i=0; i<30; i++)
    rainbowSetStep(20, first, last);
    
  wompwomp(first, last);
  
  set(0, first, last);
  strip.show();
  delay(900);
}

void strobe(uint8_t light_wait, uint8_t dark_wait) {
  global_wheel_offset = (++global_wheel_offset) % 96;
  delay(dark_wait);
  set(Wheel(global_wheel_offset));
  strip.show();
  
  delay(light_wait);
  set(0);
  strip.show();
}

/* spacing = # of pixels between each lit up pixel as the helix moves
 * up = direction
 */
void helix(uint8_t wait, int spacing, boolean up) {
  spacing++;
  global_wheel_offset = (++global_wheel_offset) % 96;
  
  for (int i=0; i<strip.numPixels(); i++)
    if (i%spacing == pixel_offset%spacing)
      strip.setPixelColor(i, Wheel(global_wheel_offset));
        
  strip.show();
  delay(wait);
  set(0);

  if (up) pixel_offset--;
  else    pixel_offset++;
  pixel_offset += strip.numPixels(); //arduino modulus allows negative output
  pixel_offset %= strip.numPixels();
}

void wompwomp(int first, int last) {
  for (int i=0; i<2; i++)
    pulseSegment(60, first, last);
  for (int i=0; i<2; i++)
    pulseSegment(30, first, last);
}

void multiSegmentRainbowBounce(uint8_t wait, int segments[][2]) {
  int iterations = abs(segments[0][0]-segments[0][1])+1;
  for (int j=0; j<=iterations; j++)
    multiSegmentRainbowStep(wait, segments, j);
  for (int j=iterations; j>=0; j--)
    multiSegmentRainbowStep(wait, segments, j);
}

void multiSegmentRainbowStep(uint8_t wait, int segments[][2], int offset) {
  global_wheel_offset = (++global_wheel_offset) % 96;
  multiSegmentPixel(segments, offset, Wheel(global_wheel_offset));
  strip.show();
  delay(wait);
  multiSegmentPixel(segments, offset, 0);
}

void multiSegmentPixel(int segments[][2], int offset, uint16_t color) {
  int num_seg = sizeof(*segments);
  int first, last, index;
  for (int i=0; i<num_seg; i++) {
    first = segments[i][0];
    last = segments[i][1];
    index = (first<last) ? first+offset : first-offset;
    strip.setPixelColor(index, color);
  }
}

/* single segment patterns */

void pulseSegment(uint8_t wait, int first, int last) {
  continuousRainbowChase(wait, first, last);
  continuousRainbowChase(wait, last, first);
  delay(20);
}

// set all [first,last], using global wheel offset
void rainbowSetStep(uint8_t wait, int first, int last) {
  int i, j;
  for (i=first; i<=last; i++)
    strip.setPixelColor(i, Wheel(global_wheel_offset+i));
  global_wheel_offset = (global_wheel_offset+1) % 96;
  strip.show();
  delay(wait);
}

void rainbow(uint8_t wait) {
  int i, j;
   
  for (j=0; j < 96 * 3; j++) {     // 3 cycles of all 96 colors in the wheel
    if (restartLoop) {
      restartLoop = 0;
      break;
    }
    for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(i + j));
    }  
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// equally distribute rainbow wheel along chain
void rainbowCycle(uint8_t wait) {
  int i, j;
  
  for (j=0; j < 96 * 5; j++) {     // 5 cycles of all 96 colors in the wheel
    for (i=0; i < strip.numPixels(); i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel((i * 96 / strip.numPixels()) + j) );
    }  
    strip.show();   // write all the pixels out
    delay(wait);
  }
}

// uses and modifies global_wheel_offset
void continuousRainbowBounce(uint8_t wait, int first, int last) {
  continuousRainbowChase(wait, first, last);
  continuousRainbowChase(wait, last-1, first+1);
} 
void continuousRainbowChase(uint8_t wait, int first, int last) {
  global_wheel_offset = rainbowChase(wait, first, last, global_wheel_offset);
}
void continuousRainbowFill(uint8_t wait, int first, int last) {
  global_wheel_offset = rainbowFill(wait, first, last, global_wheel_offset);
}
void continuousRainbow(uint8_t wait, int first, int last, int steps) {
  for (int i=0; i<steps; i++)
    global_wheel_offset = rainbowStepRev(wait, first, last, global_wheel_offset);
}

// bounce range starting from set wheel offset
int rainbowBounce(uint8_t wait, int first, int last, int wheel_pos) {
  wheel_pos = rainbowChase(wait, first, last, wheel_pos);
  return rainbowChase(wait, last-1, first+1, wheel_pos);
}
// chase range starting from set wheel offset
int rainbowChase(uint8_t wait, int first, int last, int wheel_pos) {
  if (first<last)
    for (int i=first; i<=last; i++)
      chaseStep(i, Wheel(wheel_pos++), wait);
  else //reverse
    for (int i=first; i>=last; i--)
      chaseStep(i, Wheel(wheel_pos++), wait);
  return wheel_pos % 96;
}
// fill range starting from set wheel offset
int rainbowFill(uint8_t wait, int first, int last, int wheel_pos) {
  if (first<last)
    for (int i=first; i<=last; i++)
      fillStep(i, Wheel(wheel_pos++), wait);
  else //reverse
    for (int i=first; i>=last; i--)
      fillStep(i, Wheel(wheel_pos++), wait);
  return wheel_pos % 96;
}
// increment rainbow starting from set wheel offset
int rainbowStep(uint8_t wait, int first, int last, int wheel_pos) {
  if (first<last)
    for (int i=first; i<=last; i++)
      strip.setPixelColor(i, Wheel(wheel_pos + i));
  else //reverse
    for (int i=first; i>=last; i--)
      strip.setPixelColor(i, Wheel(wheel_pos + i));
  strip.show();
  delay(wait);
  return ++wheel_pos % 96;
}

// rainbow colors count backwards
int rainbowStepRev(uint8_t wait, int first, int last, int wheel_pos) {
  if (first<last)
    for (int i=first; i<=last; i++)
      strip.setPixelColor(i, Wheel(wheel_pos - i));
  else //reverse
    for (int i=first; i>=last; i--)
      strip.setPixelColor(i, Wheel(wheel_pos - i));
  strip.show();
  delay(wait);
  return ++wheel_pos % 96;
}

// bounce entire strip
void bounce(uint16_t c, uint8_t wait) {
  bounce(c, wait, 0, strip.numPixels()-1);
}
// bounce range
void bounce(uint16_t c, uint8_t wait, int first, int last) {
  chase(c, wait, first, last);
  chase(c, wait, last-1, first+1);
}

/* standalone events */

// chase a single pixel along a range [first,last]
void chase(uint16_t c, uint8_t wait, int first, int last) {
  if (first<last)
    for (int i=first; i<=last; i++)
      chaseStep(i, c, wait);
  else //reverse
    for (int i=first; i>=last; i--)
      chaseStep(i, c, wait);
  strip.show();
}

// fill the dots one after the other with said color
void fill(uint16_t c, uint8_t wait, int first, int last) {
  if (first<last)
    for (int i=first; i<=last; i++)
      fillStep(i, c, wait);
  else //reverse
    for (int i=first; i>=last; i--)
      fillStep(i, c, wait);
}


/* steps for multiple simultaneous events */

void multiChaseStep(int indices[], uint16_t colors[], uint8_t wait) {
  multiFillStep(indices, colors, wait);
  set(0, indices);
}
void multiFillStep(int indices[], uint16_t colors[], uint8_t wait) {
  setMulti(colors, indices);
  strip.show();
  delay(wait);
}


/* one-pixel steps */

void chaseStep(int index, uint16_t c, uint8_t wait) {
    fillStep(index, c, wait);
    strip.setPixelColor(index, 0); // Erase pixel, but don't refresh!
}

void fillStep(int index, uint16_t c, uint8_t wait) {
    strip.setPixelColor(index, c); // Set new pixel on
    strip.show();                  // Refresh
    delay(wait);
}


/* set pixels */

// set whole strip to one color
void set(uint16_t c) { set(c, 0, strip.numPixels()-1); }

// set [first,last]
void set(uint16_t c, int first, int last) {
  for (int i=first; i<=last; i++)
    strip.setPixelColor(i, c);
  strip.show();
}

// set many pixels to one color
void set(uint16_t c, int indices[]) {
  int length = sizeof(indices) / sizeof(int);
  for (int i=0; i<length; i++)
    strip.setPixelColor(indices[i], c);
}

// set many pixels individually -- lengths of colors and indices must match
void setMulti(uint16_t colors[], int indices[]) {
  int length = sizeof(indices) / sizeof(int);
  for (int i=0; i<length; i++)
    strip.setPixelColor(indices[i], colors[i]);
}


/* color */

// Create a 15 bit color value from R,G,B
unsigned int Color(byte r, byte g, byte b)
{
  //Take the lowest 5 bits of each value and append them end to end
  return( ((unsigned int)g & 0x1F )<<10 | ((unsigned int)b & 0x1F)<<5 | (unsigned int)r & 0x1F);
}

//Input a value 0 to 127 to get a color value.
//The colours are a transition r - g -b - back to r
unsigned int Wheel(byte WheelPos)
{
  WheelPos = WheelPos % 96;
  byte r,g,b;
  switch(WheelPos >> 5)
  {
    case 0:
      r=31- WheelPos % 32;   //Red down
      g=WheelPos % 32;      // Green up
      b=0;                  //blue off
      break; 
    case 1:
      g=31- WheelPos % 32;  //green down
      b=WheelPos % 32;      //blue up
      r=0;                  //red off
      break; 
    case 2:
      b=31- WheelPos % 32;  //blue down 
      r=WheelPos % 32;      //red up
      g=0;                  //green off
      break; 
  }
  return(Color(r,g,b));
}

    

// NOT CURRENTLY USED
void musicVisualizer() {
  set(Color(63,0,0), 0, 2);
  strip.show();
  delay(50);
  digitalWrite(testPin, HIGH);
  
  int i, numSamples = 5;
  for (i=0; i<7; i++) spectrumValue[i] = 0;
  for (int sample=0; sample<numSamples; sample++) {
    int* spectrum = sampleSpectrum();
    for (i=0; i<7; i++)
      spectrumValue[i] += spectrum[i];
  }
  for (i=0; i<7; i++) {
    spectrumValue[i] /= numSamples;
  }

  digitalWrite(testPin, LOW);
}

int* sampleSpectrum() {
  int spectrum[7];
  
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);
  for (int i=0; i<7; i++) {
    digitalWrite(strobePin, LOW);
    delayMicroseconds(30); // to allow the output to settle
    spectrum[i] = analogRead(msgeqPin);
    digitalWrite(strobePin, HIGH);
  }
  return spectrum;
}


