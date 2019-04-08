/* This code is used for the steering wheel.
   A rotary encoder with push button and a potentiometer are connected to an Arduino Nano.
   The rotary encoder is used to select 1 out of the 4 possible modes. --> [0, 3]
   The push button determines the direction in which the boat sails. (Reverse vs. Forward) --> [bool true = reverse / false = forward]
   The potentiometer functions as throttle. --> [0, 1024]

   Solar Boat Twente 2018/2019
*/


/* www.circuits4you.com
   read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,    encoder0PinA to pin 2, encoder0PinB to pin 4
   it doesn't matter which encoder pin you use for A or B
*/

/* Read Quadrature Encoder
   Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.

   Sketch by max wolf / www.meso.net
   v. 0.1 - very basic functions - mw 20061220
*/

// Define pins for encoder
#define encoder0PinA  2  //CLK Output A Do not use other pin for clock as we are using interrupt
#define encoder0PinB  4  //DT Output B
#define Switch 5 // Switch connection if available

volatile unsigned int encoder0Pos = 0; //current position of encoder
bool reverse = false; //boolean om de boot in achter- / vooruit te zetten
int check = 0;        //check om van achter- / vooruit te kunnen wisselen

int encoder0PinALast = LOW;
int mode = 0;  //initial mode 1


void setup() {
  // Initialize pins
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  pinMode(Switch, INPUT);                 // define the button of the encoder as input
  digitalWrite(Switch, HIGH);
  //attachInterrupt(0, doEncoder, RISING); // encoder pin on interrupt 0 - pin2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
}

void loop() {
  // Call Throttle function
  Throttle();

  // Call modes function
  Modes();
  //Serial.println (encoder0Pos, DEC);  //Angle = (360 / Encoder_Resolution) * encoder0Pos

  // Achteruit of Vooruit knop
  //Serial.println(reverse); //print de output van de pin in de Serial Monitor
  Direction();
}



void Direction() {
  // Zolang de knop ingedrukt is check in welke stand de boot staat en verander van richting
  while (digitalRead(Switch) == LOW) {
    if (reverse == false && check == 0) {
      reverse = true;
      check = 1;
    } else if (reverse == true && check == 2 ) {
      reverse = false;
      check = 4;
    }
  }

  // Zodra de knop weer losgelaten wordt, zet de check klaar voor volgende keer dat de knop ingedrukt wordt.
  if (digitalRead(Switch) == HIGH && reverse == true) {
    check = 2;
  } else if (digitalRead(Switch) == HIGH && reverse == false) {
    check = 0;
  }

  //return reverse
}


void Modes() {
  // lees de pins van de encoder uit
  int pinA = digitalRead(encoder0PinA);
  int pinB = digitalRead(encoder0PinB);

  // bepaal welke rotatie richting van encoder
  if (encoder0PinALast == LOW && pinA == HIGH) {
    if ( pinB == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
    Serial.println(encoder0Pos);
    Serial.print("mode: ");
    Serial.println(mode);

    // scroll door de 4 verschillende modes
    //2 ticks = + or - 1
    if ( encoder0Pos <= 2 ) {
      mode = 0;
    } else if (2 < encoder0Pos && encoder0Pos < 5) {
      mode = 1;
    } else if (5 <= encoder0Pos && encoder0Pos < 7) {
      mode = 2;
    } else if (7 <= encoder0Pos) {
      mode = 3;
    }
  }
  encoder0PinALast = pinA;
  
  //return mode
}


void Throttle() {
  // read the input on analog pin A1:
  float sensorValue = analogRead(A1);
  // print out the value you read:
  //Serial.println(sensorValue);

  delay(1);        // delay in between reads for stability

  //return sensorValue
}
