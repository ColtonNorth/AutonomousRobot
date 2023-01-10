int LLS = 25;
int LMS = 24;
int LRS = 27;
int RLS = 26;
int RMS = 29;
int RRS = 28;


void setup() {
  pinMode(LLS, INPUT);
  pinMode(LMS, INPUT);
  pinMode(LRS, INPUT);
  pinMode(RLS, INPUT);
  pinMode(RMS, INPUT);
  pinMode(RRS, INPUT);

  Serial.begin(115200);
}

void loop() {

  //Left Line Follower
  if(digitalRead(LLS) == LOW)
  {
    Serial.print("W ");
  }
  else
  {
    Serial.print("B ");
  }
  if(digitalRead(LMS) == LOW)
  {
    Serial.print("W ");
  }
  else
  {
    Serial.print("B ");
  }
  if(digitalRead(LRS) == LOW)
  {
    Serial.print("W ");
  }
  else
  {
    Serial.print("B ");
  }
  Serial.print("    |    ");
  //RightLineFollower
  if(digitalRead(RLS) == LOW)
  {
    Serial.print("W ");
  }
  else
  {
    Serial.print("B ");
  }
  if(digitalRead(RMS) == LOW)
  {
    Serial.print("W ");
  }
  else
  {
    Serial.print("B ");
  }
  if(digitalRead(RRS) == LOW)
  {
    Serial.print("W ");
  }
  else
  {
    Serial.print("B ");
  }
  Serial.println("");
}
