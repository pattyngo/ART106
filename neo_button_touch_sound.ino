#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN    6   // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 3   // number of neopixel (change this accordingly)

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

int showType = 0;

//const int ledpin=6; // ledpin outout
const int RELAY_PIN = A5; //pump pin

const int soundpin=A2; //sound input
const int threshold=300; // sets threshold value for sound sensor

const int touchpin=8; //touch input

const int buttonPin = 4; 
int buttonState = 0; //read btn

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  Serial.begin(9600);
  //pinMode(ledpin,OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(soundpin,INPUT);
  pinMode(touchpin,INPUT);
  pinMode(buttonPin,INPUT);
  
  digitalWrite(6, LOW); //LED Off

}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) { //begin sound sensor test
      int soundsens=analogRead(soundpin); // reads analog data from sound sensor
        if (soundsens>=threshold) {
//        turn on neopixels with sound
        showType++;
        if (showType > 9)
          showType=0;
        startShow(showType);
        
//        turn on pump with sound
//        digitalWrite(ledpin,HIGH); //turns led on
        digitalWrite(RELAY_PIN, HIGH); //turn pump on
        delay(100);
      }
        else{
//          digitalWrite(ledpin,LOW);
          digitalWrite(RELAY_PIN, LOW);
          delay(100);
    }
  } //end sound test
  
  
  else { //begin touch test
  if(digitalRead(8) == HIGH) {
    //neopixel touch
      showType++;
      if (showType > 9)
        showType=0;
      startShow(showType);
    //pump touch
//    digitalWrite(6, HIGH);
    digitalWrite(RELAY_PIN, HIGH); // turn on pump 5 seconds
  delay(100);
    } else {
//    digitalWrite(6, LOW);
    digitalWrite(RELAY_PIN, LOW);  // turn off pump 5 seconds
  delay(100);
    }
  } //end touch test
}

void startShow(int i) {
  switch(i){
    case 0: colorWipe(strip.Color(181, 13, 133), 50);    // Hot Pink
            break;
    case 1: colorWipe(strip.Color(255, 0, 0), 50);  // Red
            break;
    case 2: colorWipe(strip.Color(0, 255, 0), 50);  // Green
            break;
    case 3: colorWipe(strip.Color(0, 0, 255), 50);  // Blue
            break;
    case 4: colorWipe(strip.Color(255, 169, 71), 50);// Orange
            break;
    case 5: colorWipe(strip.Color(255,239,18), 50);  // Yellow
            break;
    case 6: colorWipe(strip.Color(178,92,214), 50);  // Purple
            break;
    case 7: colorWipe(strip.Color(237, 168, 213), 50);  // Pink
            break;
    case 8: colorWipe(strip.Color(67, 209, 195), 50);  // Teal
            break;
    case 9: colorWipe(strip.Color(0, 0, 0), 50);  // White
            break;
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
