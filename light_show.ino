#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN_ENCODER_A      5
#define PIN_ENCODER_B      3
#define TRINKET_PINx       PIND
#define PIN_ENCODER_SWITCH 4

#define DIAL_POSITION_COUNT 24
#define DEFAULT_PERIOD 50

#define PATTERN_COUNT 6

void (*patternFuncs[PATTERN_COUNT])(bool);
int currentPattern = 0;
byte frameCounter = 0;
byte dialPosition = 0;
unsigned long updatePeriod = DEFAULT_PERIOD;
unsigned long lastUpdate = 0;

static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;

int8_t enc_action = 0;

#define PIN_PIXEL  6
#define NUMPIXELS 16

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN_PIXEL, NEO_GRB + NEO_KHZ800);

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting the light show...");

  patternFuncs[0] = rainbowPattern;
  patternFuncs[1] = scannerPattern;
  patternFuncs[2] = scannerTwoPattern;
  patternFuncs[3] = scannerTwoReversePattern;
  patternFuncs[4] = randomPattern;
  patternFuncs[5] = pointerPattern;
  
  strip.begin();
  strip.setBrightness(50);  
  strip.show(); 

  // set pins as input with internal pull-up resistors enabled
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);

  // get an initial reading on the encoder pins
  if (digitalRead(PIN_ENCODER_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(PIN_ENCODER_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }

  randomSeed(analogRead(0));
}

void loop() {

  /////////////////////////////////////////////////////////////////
  // ENCODER
  
  enc_action = 0; // 1 or -1 if moved, sign is direction
  // note: for better performance, the code will use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_B)) {
    enc_cur_pos |= (1 << 1);
  }

  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos) {
    if (enc_prev_pos == 0x00) {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      } else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }

    if (enc_cur_pos == 0x03) {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    } else if (enc_cur_pos == 0x00) {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      } else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }

      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      } else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      } else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      } else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }

      enc_flags = 0; // reset for next time
    }
  }

  enc_prev_pos = enc_cur_pos;

  if (enc_action > 0) {
    // Clockwise
    Serial.println("up");
    dialPosition = (dialPosition+1)%DIAL_POSITION_COUNT;
  } else if (enc_action < 0) {
    // Counterclockwise
    Serial.println("down");
    dialPosition = dialPosition == 0 ? DIAL_POSITION_COUNT-1 : dialPosition-1;
  }

  /////////////////////////////////////////////////////////////////
  // BUTTON
  
  // remember that the switch is active low 
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_SWITCH)) {
    if (sw_was_pressed == 0) {
      // only on initial press, so the keystroke is not repeated while the button is held down
      // Encoder pushed down
      nextPattern();
      delay(5); // debounce delay
    }
    sw_was_pressed = 1;
  } else {
    if (sw_was_pressed != 0) {
      delay(5); // debounce delay
    }
    sw_was_pressed = 0;
  }

  /////////////////////////////////////////////////////////////////
  // PATTERN UPDATE
  if (millis()-lastUpdate >= updatePeriod) {
    patternFuncs[currentPattern](true);
    frameCounter++;
    lastUpdate = millis();
  } else {
    patternFuncs[currentPattern](false);
  }
  
  
} // end loop()

void nextPattern() {
  currentPattern = (currentPattern+1)%PATTERN_COUNT;
  // reset pattern period
  updatePeriod = DEFAULT_PERIOD;
  
  Serial.print("Pattern set to: ");Serial.println(currentPattern,DEC);
}

byte pixelsToUpdate = 1;
void randomPattern(bool draw) {
  if (draw) {
    for (int i = 0; i < pixelsToUpdate; i++) {
      strip.setPixelColor(random(NUMPIXELS), Wheel(random(255)));
    }
    strip.show();
  }
  if (enc_action > 0 && pixelsToUpdate < NUMPIXELS) {
    pixelsToUpdate++;
  } else if (enc_action < 0 && pixelsToUpdate > 0) {
    pixelsToUpdate--;
  }
}

void rainbowPattern(bool draw) {
  if (draw) {
    strip.setPixelColor(frameCounter%NUMPIXELS, Wheel(frameCounter));
    strip.show();
  }
  if (enc_action > 0) {
    speedup();
  } else if (enc_action < 0) {
    slowdown();
  }
}

void scannerPattern(bool draw) {
  if (draw) {
    strip.setPixelColor((frameCounter-1)%NUMPIXELS, strip.Color(0, 0, 0));
    strip.setPixelColor(frameCounter%NUMPIXELS, Wheel(frameCounter));
    strip.show();
  }
  if (enc_action > 0) {
    speedup();
  } else if (enc_action < 0) {
    slowdown();
  }
}

void scannerTwoPattern(bool draw) {
  if (draw) {
    strip.setPixelColor((frameCounter-1)%NUMPIXELS, strip.Color(0, 0, 0));
    strip.setPixelColor(frameCounter%NUMPIXELS, Wheel(frameCounter));
    strip.setPixelColor((frameCounter+7)%NUMPIXELS, strip.Color(0, 0, 0));
    strip.setPixelColor((frameCounter+8)%NUMPIXELS, Wheel(frameCounter));
    strip.show();
  }
  if (enc_action > 0) {
    speedup();
  } else if (enc_action < 0) {
    slowdown();
  }
}

void scannerFourPattern(bool draw) {
  if (draw) {
    strip.setPixelColor((frameCounter-1)%NUMPIXELS, strip.Color(0, 0, 0));
    strip.setPixelColor(frameCounter%NUMPIXELS, Wheel(frameCounter));
    strip.setPixelColor((frameCounter+7)%NUMPIXELS, strip.Color(0, 0, 0));
    strip.setPixelColor((frameCounter+8)%NUMPIXELS, Wheel(frameCounter));
    strip.show();
  }
  if (enc_action > 0) {
    speedup();
  } else if (enc_action < 0) {
    slowdown();
  }
}

void scannerTwoReversePattern(bool draw) {
  if (draw) {
    strip.setPixelColor((frameCounter-1)%NUMPIXELS, strip.Color(0, 0, 0));
    strip.setPixelColor(frameCounter%NUMPIXELS, Wheel(frameCounter));
    strip.setPixelColor((frameCounter+7)%NUMPIXELS, strip.Color(0, 0, 0));
    strip.setPixelColor((frameCounter+8)%NUMPIXELS, Wheel((frameCounter+128)%255));
    strip.show();
  }
  if (enc_action > 0) {
    speedup();
  } else if (enc_action < 0) {
    slowdown();
  }
}

byte pointerPosition = 0;
void pointerPattern(bool draw) {
  if (draw) {
    strip.clear();
    strip.setPixelColor(pointerPosition, Wheel(frameCounter));
    strip.show();
  }
  pointerPosition = (pointerPosition - enc_action) % NUMPIXELS;
}

void speedup() {
  if (updatePeriod > 1) { 
    updatePeriod-=2;
  }
}

void slowdown() {
  updatePeriod+=2;
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}



