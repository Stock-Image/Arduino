int red = 2;
int green = 3;
int blue = 4;
int out_pump = 5;
int in_pump = 6;

void setup ()
{
  pinMode (red, OUTPUT);
  pinMode (green, OUTPUT);
  pinMode (blue, OUTPUT);
  pinMode (out_pump, OUTPUT);
  pinMode (in_pump, OUTPUT);
  Serial.begin (9600);
}

void loop()
{
  // read the input on analog pin 3:
  int value = analogRead(A3);

  if (value > 650)
  {
    Serial.println("Very heavy Rain");
    digitalWrite (red, HIGH);
    digitalWrite(green, LOW);
    digitalWrite(blue, LOW);
    digitalWrite(out_pump, HIGH);
    digitalWrite(in_pump, LOW);
  }
  else if ((value > 300) && (value <= 650))
  {
    Serial.println("AVERAGE Rain");
    digitalWrite (green, HIGH);
    digitalWrite (red, LOW);
    digitalWrite(out_pump, LOW);
    digitalWrite(in_pump, LOW);
  }
  else
  {
    Serial.println("Dry Weather");
    digitalWrite (blue, HIGH);
    digitalWrite (red, LOW);
    digitalWrite (green, LOW);
    digitalWrite(out_pump, LOW);
    digitalWrite(in_pump, HIGH);;
    delay(100);
  }
}
