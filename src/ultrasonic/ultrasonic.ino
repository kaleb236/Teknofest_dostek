#define trig 2
#define echo 3
int mesafe, zaman;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

}

int distance(){
  digitalWrite(trig, HIGH);
  delayMicroseconds(100);
  digitalWrite(trig, LOW);
  zaman = pulseIn(echo, HIGH);
  mesafe = (zaman / 2) / 29.1;
  return mesafe;
}
void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(distance());
  delay(100);

}
