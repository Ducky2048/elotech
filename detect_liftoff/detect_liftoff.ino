/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255
  and uses the result to set the pulsewidth modulation (PWM) of an output pin.
  Also prints the results to the serial monitor.

  The circuit:
   potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
   LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

const int sensorValuesLen = 100;
const int jitterdelta = 10;
int sensorValues[sensorValuesLen];
int ledPin = 3;// value read from the pot
const int buttonPin = 4; // pin where button data is
int buttonState = 0;
bool lastState = false;
bool timeIsRunning = false;

uint8_t    TimeDisp0[] = { 0x10, 0x00, 0x00, 0x00 };
uint8_t    TimeDisp1[] = { 0x10, 0x00, 0x00, 0x00 };

Me7SegmentDisplay disp0(PORT_3); //12 & 13
Me7SegmentDisplay disp1(PORT_2); //3 & 9

int startTime = 0;
int currTime = 0;

void setup() {
  disp0.set();
  disp0.init();
  disp1.set();
  disp1.init();
  pinMode(buttonPin, INPUT_PULLUP);
  startTime =  millis();
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);   // sets the LED on
  lastState = isGlassPresent();
  Serial.write("Entering main loop\n");
}

void loop()
{
   bool presence = isGlassPresent();
   if (presence)
  {
    Serial.write("Glass is present\n");
    if (timeIsRunning) {
      displayTime(disp0, currTime);
      timeIsRunning = false;
    }

  }
  else
  {
    if (!timeIsRunning) {
      startTime = millis();
    }
    timeIsRunning = true;

    currTime = (millis() - startTime);
    displayTime(disp0, currTime);

    Serial.write("Glass is no longer present\n");
  }
  
   buttonState = digitalRead(buttonPin);
  gif (buttonState == LOW) { 
   Serial.write("button Pressed");
   displayTime(disp1, currTime);   
  }
  delay(50);

}

void readValuesIntoArray()
{
  for (int i = 0; i < sensorValuesLen; i++)
  {
    sensorValues[i] = analogRead(analogInPin);
  }
}

bool isGlassPresent()
{
  int jitter = 0;
  readValuesIntoArray();
  for (int i = 0; i < sensorValuesLen; i++)
  {
    if (sensorValues[i] < 1000)
    {
      jitter++;
      if (jitter > jitterdelta)
      {
        return false;
      }
    }

  }
  return true;
}

void printArray()
{
  for (int i = 0; i < sensorValuesLen; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.print('\n');
}


unsigned int pdiv(unsigned int a, unsigned int b)
{
  return (a - ( a % b)) / b;
}


void displayTime(Me7SegmentDisplay disp, int milliseconds) {
  unsigned int centisecs = pdiv(milliseconds, 10);
  unsigned int sec = pdiv(centisecs, 100);
  unsigned int sec1 = pdiv((sec % 100), 10);
  unsigned int sec2 = (sec % 10);

  unsigned int milli1 = pdiv((centisecs % 100), 10);
  unsigned int milli2 = centisecs % 10;

  //1 & 2 sec
  uint8_t timeDisp[] = { 00, 00, 00, 00 };
  if (sec1 == 6) {
    timeDisp[0] = 0x0D;
    timeDisp[1] = 0x0E;
    timeDisp[2] = 0x0A;
    timeDisp[3] = 0x0D;
  } else {
    timeDisp[0] = sec1;
    timeDisp[1] = 0x10 + sec2;
    timeDisp[2] = milli1;
    timeDisp[3] = milli2;
  }
  disp.display(timeDisp);
}



