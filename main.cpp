#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN            13    // Digital pin connected to the Neopixel
#define NUMPIXELS      4    // Number of Neopixels in your strip

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Set color for a specific Neopixel
void setPixelColor(int pixel, uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(pixel, strip.Color(red, green, blue));
  strip.show();
}

void testLight() {
  // Set color for each Neopixel individually
  setPixelColor(0, 0, 255, 0);  // Red for the first Neopixel
  setPixelColor(1, 0, 255, 0);  // Red for the second Neopixel
  setPixelColor(2, 0, 0, 0);    // Turn off other Neopixels
  setPixelColor(3, 0, 0, 0);    // Turn off other Neopixels
}

void setup() {
  strip.begin();  // Initialize the Neopixel strip
  strip.show();   // Initialize all pixels to 'off'
  Serial.begin(9600);
   // Calibrate the servo to a known initial position
  testLight();
}

void loop() {

}