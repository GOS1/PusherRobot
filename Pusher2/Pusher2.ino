#include <AccelStepper.h>
#include <MultiStepper.h>

const float positions[] = {4.5, 9, 13.5, 18, 22.5, 27, 31.5, 36, 40.5, 45, 49.5, 54, 59.5, 64, 69.5}; //positions in inches measured from "home" position
    // [NOTE] Only using 11 positions to start with right now. 
    //positions are 4.5" (11.43 cm) apart, on center************THESE POSITIONS WILL NEED TO BE ADJUSTED BY OFFSET FROM HOME POSITION->POSITION 1
    //position index values stored as: 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14
const int arraySize = 15; //?????? *NOTE: this seems to be working, but may need to change to " int arraySize=15; " ???????
const float distancePerRevolution=2.6006; //distance (inches) of timing belt travel per one 360deg (200step) rotation of 22-tooth, 3mm pitch drive gear **actual value = 6 cm
const long stepsPerRevolution=3200; // motor's rated # steps per one revolution (1/16th Microstep)

// Keep track of where I am, so I can calculate where I'd be going next. 
float currentXPosition = 0;

// Feedback pin that gets activated when the y-stepper hits the object.
const int yFeedbackPin = 4;
const int xHomePin = 2; 
const int yHomePin = 3; 
const int resetPin = 8; 

// States of the system. 
enum State {
  SEARCH,
  FIND
};

const long maxYSteps = 33508;
const long yStepsPerRevolution = 400;

// For Home, we always do that in setup.
State currentState = SEARCH;

// Initialize X & Y stepper motors. 
AccelStepper xStepper( AccelStepper::DRIVER, 10, 11);
AccelStepper yStepper( AccelStepper::DRIVER, 5, 6);

void setup() {
  Serial.begin(9600);

  // Set output pins. 
  pinMode(yFeedbackPin, INPUT_PULLUP);
  pinMode(xHomePin, INPUT_PULLUP);
  pinMode(yHomePin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);

  // Home the system.
  homeSystem();
}

void loop() {
    boolean xHomeState = digitalRead(xHomePin);
    boolean yHomeState = digitalRead(yHomePin);
    boolean yFeedbackState = digitalRead(yFeedbackPin); 
    boolean resetState = digitalRead(resetPin);
//
//    Serial.print("xHome, yHome, yFeedback, yFowardPin: "); 
//    Serial.print(xHomeState);
//    Serial.print(", ");
//    Serial.print(yHomeState);
//    Serial.print(", ");
//    Serial.print(yFeedbackState);
//    Serial.print(", ");
//    Serial.println(resetState);
//    
    if (currentState == SEARCH) {
        // Configure x-stepper initial speeds. 
        xStepper.setMaxSpeed(20000.0);
        xStepper.setAcceleration(2000.0);
        
        // Calculate a random position, distance to go, and numSteps to get to that position.
        int randomIdx = random(arraySize);
        float distance = positions[randomIdx] - currentXPosition; // It could be +ve/-ve and that'll determine direction to go to.
//        Serial.println(positions[randomIdx]);
//        Serial.println(distance);
        long numSteps = (distance * stepsPerRevolution) / distancePerRevolution;
//        Serial.println(numSteps);
      
        // Print the current position. 
//        printPos(randomIdx, positions[randomIdx], numSteps);
        
        // Move to that random position. 
        xStepper.move(numSteps);
        xStepper.runToPosition();
      
        // Update my current position.
        currentXPosition = positions[randomIdx]; 

        currentState = FIND; 
    }
  
    if (currentState == FIND) {
       // Configure y-stepper for the rest of the system. 
       yStepper.setMaxSpeed(1000.0);
       yStepper.setAcceleration(2000.0);
       
       int ySteps = 0; 
       // Move the motor into the slot.  
       while (digitalRead(yFeedbackPin) == HIGH && yStepper.currentPosition() < maxYSteps) {
          yStepper.moveTo(ySteps);
          yStepper.run();
          ySteps++;
       }

       // If we haven't reached the maxYSteps, then we have detected something. 
       // So, push forward a little bit. Else, home back. 
       if (yStepper.currentPosition() < maxYSteps) {
            yStepper.stop();
            
            delay(1000);

            // Move ySteps. 
            yStepper.setMaxSpeed(1000.0);
            yStepper.setAcceleration(2000.0);
            yStepper.move(yStepsPerRevolution/2);
            yStepper.runToPosition();
       } else {
          yStepper.stop();
          delay(1000);
       }

       homeYStepper();
       currentState = SEARCH;
    }

    delay(500);
}

void homeXStepper() {
  xStepper.setCurrentPosition(0);
  xStepper.setMaxSpeed(1000.0);
  xStepper.setAcceleration(1000.0);
  
  // Move in the positive direction.
  int xSteps = 0; 

  // This is PULLUP so it's HIGH if it's not pressed. 
  while (digitalRead(xHomePin) == HIGH) {  // Make the stepper move counterclockwise until the homeSwitch is activated.
    xStepper.move(xSteps);  // Set the position to move to
    xStepper.run();
    xSteps--;  // Move all the way to the left for calibration. 
  }

  delay(2000);

  xStepper.setCurrentPosition(0);
  xSteps = 0; 

  // This is PULLUP so it's LOW when it's pressed.
  while (digitalRead(xHomePin) == LOW) { // Make the Stepper move clockwise until the homeSwitch is deactivated
    xStepper.move(xSteps);
    xStepper.run();
    xSteps++; // increase by one for next move if needed
  }

  // Now set the current position. 
  xStepper.setCurrentPosition(0);
}

void homeYStepper() {
  // Set slower speeds for the yStepper. 
  yStepper.setCurrentPosition(0);
  yStepper.setMaxSpeed(1000.0);
  yStepper.setAcceleration(2000.0);
  
  // Steps to take in the y direction.
  int ySteps = 0; 
  
  while (digitalRead(yHomePin) == HIGH) {  // Make the stepper move backward until the homeSwitch is activated.
    yStepper.move(ySteps);  // Set the position to move to
    yStepper.run();
    ySteps--;  // Keep going backwards
  }

  delay(2000);

  yStepper.setCurrentPosition(0);
  ySteps = 0;
  
  while (digitalRead(yHomePin) == LOW) { // Make the Stepper move forward until the homeSwitch is deactivated
    yStepper.move(ySteps);
    yStepper.run();
    ySteps++; // increase by one for next move if needed
  }

  // We have found our 0 position. Boom. 
  yStepper.setCurrentPosition(0); 
}

// Helper method to print where I am. 
void printPos(int idx, float distance, float numSteps) {
  Serial.print("Destination: Position, Distance, NumSteps ");
  Serial.print(idx);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.println(numSteps);
}

void homeSystem() {
  homeYStepper();
  homeXStepper();
  currentState = SEARCH;
  currentXPosition = 0;
}

// Testing Homing. 
void moveYForward() {
   float ySteps = 0; 
   
   // Move the motor into the slot.  
   while (digitalRead(yFeedbackPin) == HIGH) {
      yStepper.moveTo(ySteps);
      yStepper.run();
      ySteps++;
   }

   Serial.print("Distance:");
   Serial.println(yStepper.currentPosition());
}

// Testing Homing. 
void moveXForward() {
  Serial.println("Move x Forward");
  int xSteps = 20000; 
  xStepper.move(xSteps);
  xStepper.runToPosition();
  Serial.println("There.");
}

// Some yStepper tuning code that we could use. 

//    int ySteps = 16000;
//    yStepper.move(-ySteps);
//    yStepper.runToPosition();
//    
//   // Move the motor into the slot.  
//   while (!digitalRead(yFeedbackPin) == LOW) {
//      yStepper.move(ySteps);
//      yStepper.run();
//      ySteps++;
//   }
//  
//   int distanceToGo = yStepper.distanceToGo();
//   int stepsToGoBack = ySteps - distanceToGo;
//  
//   Serial.println(stepsToGoBack);
//   Serial.println("Hit something.");
//   yStepper.stop();
//  
//   delay(2000);
//  
//   yStepper.setMaxSpeed(500);
//  
//   int newPushSteps = 2000;
//   yStepper.move(newPushSteps);
//   yStepper.runToPosition();
//  
//   delay(2000);
//  
//   yStepper.setMaxSpeed(3000);
//   yStepper.moveTo(-(stepsToGoBack + newPushSteps));
//   Serial.println(yStepper.targetPosition());
//   yStepper.runToPosition();
//  
//   delay(2000);

