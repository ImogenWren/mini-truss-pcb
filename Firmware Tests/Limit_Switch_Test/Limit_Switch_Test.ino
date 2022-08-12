
int LIMIT = 11;


void setup() {
  // put your setup code here, to run once:
  pinMode (LIMIT , INPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);
}
 int i;
void loop() {

    if (digitalRead(LIMIT) == HIGH)
    {
       Serial.println("Limit Switch off");
    }
    else if (digitalRead(LIMIT) == LOW)
    {
     Serial.println ("Limit Reached");
    }
    else
    {
     Serial.println("Limit Switch Error");
    }
    Serial.println(i);
    i++;
  delay(500);
}
