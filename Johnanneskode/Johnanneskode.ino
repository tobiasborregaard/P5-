//===========Angle relateret inputs og variabler=================

#define ServoPin 27         // GPIO 4 on the ESP  //A1 pin på Arduino
float input_voltage = 0.0;


//===========Velocity  relateret inputs og variabler=================

#define OpticPin 17  //pin 3 på arduino 

double lasttime = 0;
double delaytimer = 5;

double speedTimeCal = 1000;
double speedLastTime = 0;

double calSpeed = 0;
int counter = 0;

int analog_value;
double Angle;

void setup() {
  Serial.begin(115200);
  Serial.print("Engine starting..");
  pinMode(ServoPin, INPUT);
  pinMode(OpticPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OpticPin), Countup, RISING);
}


void loop() {

  //======= Sampling og konverting til spænding======
  analog_value = analogRead(ServoPin);
  //bool latest_value = digitalRead(35);
  input_voltage = (analog_value * 4) / 4095.0;

  if (input_voltage < 0.1)   //Filtrering af shit measurement
  {
    input_voltage = 0.0;
  }
  AngleCalculator(input_voltage);
  int oldTime = millis();
  delay(1000);  //Afgørende delay for at nå at beregne en hastighed! //Skal nok tweakes lidt
  CarVelocity();
  //Serial.println("Counter: " + counter);
  //Serial.println("Latest value: " + latest_value);
}

//===========Angle funktion, tager servospænding som input og retunere vinkel på baggrund af funktion=================
void AngleCalculator(float V) {

  Angle = 39.8613 * V - 57.6003; // Relation mellem vinkel og spænding

  Serial.print("ServoSpænding: ");
  Serial.print(input_voltage);
  Serial.println(" V");

  Serial.print("Angle: ");
  Serial.print(Angle);
  Serial.println(" °");

}

//===========Pulse counter funktion - Kaldes ud fra interrupt=================

void Countup() {
  if (millis() - lasttime >= delaytimer) {
    counter++;
    //Serial.println(counter);
    lasttime = millis();
    //Serial.println(counter);
  }
}


//===========Velocity Funktion=================

void CarVelocity() {
  if (millis() - speedLastTime >= speedTimeCal ) {
    //Beregning i m/s
    calSpeed = ((counter * 3.656) / 100) / (speedTimeCal / 1000); //SpeedTimeCal er divideret med 1000 for omregning til sekunder.
    speedLastTime = millis();
    Serial.print("Speed is currently: ");
    Serial.print(calSpeed);
    Serial.println(" m/s");

    calSpeed *= 3.6;    // Konvertering til km/t
    Serial.print("Speed in km/h: ");
    Serial.print(calSpeed);
    Serial.println(" km/h");

    counter = 0;
  }
}
