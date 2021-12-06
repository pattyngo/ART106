
#include <FastLED_NeoPixel.h>

// Which pin on the Arduino is connected to the LEDs?
#define DATA_PIN 8

// How many LEDs are attached to the Arduino?
#define NUM_LEDS 3

// LED brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 50

// Amount of time for each half-blink, in milliseconds
#define BLINK_TIME 1000




//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);//ALITOVE 100pcs WS2812B 
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGB + NEO_KHZ800);// Neopixels 
FastLED_NeoPixel<NUM_LEDS, DATA_PIN, NEO_GRB> pixels;   


byte button1 = 6;
byte button2 = 7;
byte analogPin1 = A4;  // you can also start with pots and use pins A2 and A3
byte analogPin2 = A5;

int firstSensor = 0;    // first analog sensor (pot or photocell)
int secondSensor = 0;   // second analog sensor (pot or photocell)
int thirdSensor = 0;    // digital sensor (btn)
int fourthSensor = 0;    // digital sensor (btn)
int inByte = 0;         // incoming serial byte

void setup() {
   // start serial port at 9600 or 115200 bps:
   Serial.begin(115200);
  
   pixels.begin();  // initialize strip (required!)
   pixels.setBrightness(BRIGHTNESS);
   pixels.clear(); // reset pixels
  
   for (int i = 0; i < NUM_LEDS; i++ ) {
    pixels.setPixelColor(i, pixels.Color(70,70,70)); // pixel 0
   } 
   pixels.show();

   delay(50);

   for (int i = 0; i < NUM_LEDS; i++ ) {
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // pixel 0
   } 
   pixels.show();

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(button1, INPUT);   // digital sensor is on digital pin 6 for xiao button pins
  pinMode(button2, INPUT);   // digital sensor is on digital pin 7 for xiao button pins
  establishContact();  // send a byte to establish contact until receiver responds
}

void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    firstSensor = analogRead(analogPin1);
    firstSensor = map(firstSensor,0,80,0,255); // the low and high needs to be tuned for your photocell or pot
    firstSensor = constrain(firstSensor,0,255); // to insure there are no negative values and its between 0-255
    // delay 10ms to let the ADC recover:
    delay(5);
    secondSensor = analogRead(analogPin2);
    secondSensor = map(secondSensor,0,80,0,255); // the low and high needs to be tuned for your photocell or pot
    secondSensor = constrain(secondSensor,0,255); // to insure there are no negative values and its between 0-255
    
    // read btn/switch, map it to 0 or 255
    thirdSensor = map(digitalRead(button1), 0, 1, 0, 255);
     // read btn/switch, map it to 0 or 255
    fourthSensor = map(digitalRead(button2), 0, 1, 0, 255);
    // send sensor values:
   
    Serial.write(firstSensor); 
    Serial.write(secondSensor);
    Serial.write(thirdSensor);
    Serial.write(fourthSensor);

    
    pixels.clear();
    if ( thirdSensor ) {
    pixels.setPixelColor(1, pixels.Color(0,0,255)); // pixel 0
    } else {
    pixels.setPixelColor(0, pixels.Color(255,0,0)); // pixel 1
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(10); // Delay for a period of time (in milliseconds).

    
   }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}


/////////////////////////MATCHING  2 RGB Neopixels and Analogs and switch /////////////// 
//// COPY AND PASTE into Processing and then uncomment
/*

 
  import processing.serial.*;

  int fgcolor;           // Fill color
  int bgcolor;           // Background color
  Serial myPort;                       // The serial port
  int[] serialInArray = new int[4];    // Where we'll put what we receive
  int serialCount = 0;                 // A count of how many bytes we receive
  int xpos, ypos;                // Starting position of the ball
  boolean firstContact = false;        // Whether we've heard from the microcontroller
  
  //particle lines
  ArrayList<Particle> particles;
  int numParticles;
  int minDistance;

  void setup() {
    size(900, 900);  // Stage size
   // noStroke();      // No border on the next thing drawn
    background(0);
    frameRate(20);
    // Set the starting position of the ball (middle of the stage)
    xpos = width/2;
    ypos = height/2;

    int min = 100;
    
    particles = new ArrayList();
    numParticles = 30;
    minDistance = 150;

    for (int i = 0; i < numParticles; i++) {
        Particle p = new Particle(random(min, xpos - min), random(min, ypos - min), 5); 
        particles.add(p);
        p.update();
    }
    
   //println(Serial.list());

  //MAC  find your portnumber
  myPort = new Serial(this, Serial.list()[1], 115200);
  //PC find your port name
  //myPort = new Serial(this, "COM1", 115200);
  textSize(12);
  smooth(); 

  }
  
  void draw() {
    background(bgcolor);
    fill(bgcolor,2);
    //rect(0,0,width,height); 
   
    // Draw the shape
    if ( bgcolor > 100 ) {
      
          for (Particle p1 : particles) {
        for (Particle p2 : particles) {
            if (p1 != p2) {
                float distance = dist(p1.x, p1.y, p2.x, p2.y);
              
                if (distance < minDistance) {
                    stroke(color(255-fgcolor,0,0), 255-((distance/minDistance)*255));
                    strokeWeight(1);
                    line(p1.x+xpos, p1.y+ypos, p2.x+xpos, p2.y+ypos);
                }
            }
        }
    }
    
    for (Particle p : particles) {
        p.draw();
        p.update();
    }

     
    } else {
     fill(255-fgcolor,0,fgcolor);
     noStroke();
     star(xpos, ypos, 40, 80, 5);
    }
    
  }

  void serialEvent(Serial myPort) {
    // read a byte from the serial port:
    int inByte = myPort.read();
    // if this is the first byte received, and it's an A, clear the serial
    // buffer and note that you've had first contact from the microcontroller.
    // Otherwise, add the incoming byte to the array:
    if (firstContact == false) {
      if (inByte == 'A') {
        myPort.clear();          // clear the serial port buffer
        firstContact = true;     // you've had first contact from the microcontroller
        myPort.write('A');       // ask for more
         //println("AAAAAAAA");
      }
    }
    else {
      // Add the latest byte from the serial port to array:
      serialInArray[serialCount] = inByte;
      serialCount++;

      // If we have 3 bytes:
      if (serialCount > 3 ) {
        xpos = serialInArray[0];
        xpos = int(map(xpos,0,255,150,width-150));
        ypos = serialInArray[1];
        ypos = int(map(ypos,0,255,150,height-150));
        fgcolor = serialInArray[2];
        bgcolor = serialInArray[3];

        // print the values (for debugging purposes only):
        println(xpos + "\t" + ypos + "\t" + fgcolor + "\t" + bgcolor);

        // Send a capital A to request new sensor readings:
        myPort.write('A');
        // Reset serialCount:
        serialCount = 0;
      }
    }
  }
  
 class Particle {
    float px, py;
    float x, y;
    float rx, ry;
    float rT;
    float size;
    
    Particle(float x, float y, float size) {
        this.px = x;
        this.py = y;
        this.size = size;
        this.rx = random(20, 100);
        this.ry = this.rx;
        if (random(100) < 50) {
            rx *= -1;
        }
        this.rT = random(500, 5000);
    }
    
    void update() {
        float t = millis()/rT;
        x = (int)(px + rx * cos(t));
        y = (int)(py + ry * sin(t));
    }
    
    void draw() {
        noStroke();
        fill(255-fgcolor,0,0);
        ellipse(x+xpos, y+ypos, size, size);
    }
}
  void star(float x, float y, float radius1, float radius2, int npoints) {
  float angle = TWO_PI / npoints;
  float halfAngle = angle/2.0;
  fill(255-fgcolor,0,0,120);
  beginShape();
  for (float a = 0; a < TWO_PI; a += angle) {
    float sx = x + cos(a) * radius2;
    float sy = y + sin(a) * radius2;
    vertex(sx, sy);
    sx = x + cos(a+halfAngle) * radius1;
    sy = y + sin(a+halfAngle) * radius1;
    vertex(sx, sy);
  }
  endShape(CLOSE);
}


*/
