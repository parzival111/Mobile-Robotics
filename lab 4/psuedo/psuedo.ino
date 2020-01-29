double Angle

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void GoToLight () {
  if (distance < thresh) {
    reachLignt();
  }
  else {
    pointToLight();
    moveForward();
    GoToLight();
  }
}


void pointTolight() {
  double diff = photoLeft - photoRight;

  int neg;      //the direction the right wheel should move
  if (diff >= 0)
    neg = 1;    //move left
  else
    neg = -1;   //move right

  double dist = theta * PI / 180.0 * w / 2.0;
  int spdL = convertStpL(spd);    //convert speeds from input to steps/sec
  int spdR = convertStpR(spd);    //convert speeds from input to steps/sec
  int distL = convertStpL(dist);  //convert distances from inches to steps
  int distR = convertStpR(dist);  //convert distances from inches to steps
  stepperLeft.move(-neg * distL); //set stepper distance
  stepperRight.move(neg * distR); //set stepper distance
  stepperLeft.setMaxSpeed(spdL);  //set stepper speed
  stepperRight.setMaxSpeed(spdR); //set stepper speed
  runToStop();                    //move to the desired position
}

v
