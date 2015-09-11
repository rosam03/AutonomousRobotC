/*
* Assignment: Project 1
* Authors: Catherine Lee and Rosa Mohammadi
*/


/*The LCD circuit:
 * LCD RS pin to digital pin 2
 * LCD Enable pin to digital pin 3
 * LCD D4 pin to digital pin A1
 * LCD D5 pin to digital pin A0
 * LCD D6 pin to digital pin 8
 * LCD D7 pin to digital pin 9
 * LCD R/W pin to ground
 */
 
// include the library code:
#include <LiquidCrystal.h>

//initialize the library with the numbers of the interface pins
LiquidCrystal lcd(2, 3, 15, 14, 8, 9);

/*
-----------------------------------------------------------------
Connected Pins
-----------------------------------------------------------------
*/
//Motor Control Pins
int M1_SPEED_PIN = 5;     //M1 Speed Control - left wheel
int M2_SPEED_PIN = 6;     //M2 Speed Control - right wheel
int M1_DIRECTION_PIN = 4;     //M1 Direction Control - left wheel
int M2_DIRECTION_PIN = 7;     //M2 Direction Control - right wheel
//Pins for Range Sensor 1 (Frontal Object Detection)
#define TRIG1 10 //Trig of range sensor to digital I/O pin 10
#define ECHO1 11 //Echo of range sensor to digital I/O pin 11
//Pins for Range Sensor 2 (Ground Level Detection)
#define TRIG2 12 //Trig of range sensor to digital I/O pin 12
#define ECHO2 13 //Echo of range sensor to digital I/O pin 13
//Calibration Sensors
#define TEMP_SENSE A5 //Temperature Sensor for more accurate distance detection connected Analog Pin A5.
//Pins for Line Detection Photocells
#define PHOTOCELL_BUTTON A2 //Left photocell
/*
-----------------------------------------------------------------
Distances and Speeds
-----------------------------------------------------------------
*/
#define fast_pace 200  // Full speed
#define turn_pace 255  // Turning pace 
//Regular PWM Motion Speeds 
#define FAST_PACE 217  // Full speed for line sensing
#define EDGE_DETECT_PACE 90 //A slower pace for detecting edges
#define TURN_PACE 140  // Turning pace for line sensing

//Notable Distances in cm
#define NORMAL_DISTANCE 35 //The threshold distance, in cm, between a detected object and the range sensor that is considered safe to approach without slowing down.
#define VERY_CLOSE_DISTANCE 30 //The threshold distace, in cm, between a detected object and the range sensor before the robot must stop and perform a turn.
#define OBJECT_IN_FRONT 20
// int distance;

#define PIEZZO_PIN A2
#define FLASHLIGHT_LUX 700

#define lightDiffThreshold 15 //The minimum difference between the light values sensed to indicate that a detected area is black.
#define lightColorValue 800 //Value for a white base.

#define wheel_speed_offset 1.07
#define turning_time 117

int counterForCleaning;
int mode = 0;
/*
-----------------------------------------------------------------
Basic Arduino Functions - Setup functions and superloop
-----------------------------------------------------------------
*/
/*
* Name: setup
*/
void setup(){
	Serial.begin(9600); // Initialize the Serial Monitor
        lcd.begin(16,2);
        lcd.clear();

	initializeRangeSensors(); //Initialize the range sensors and temperature sensor
        pinMode(PHOTOCELL_BUTTON, INPUT); //Initialize the left photocell.

}

/*
* Name: initializeRangeSensors
* Purpose: Decleares the range sensor trig and echo pins and output and input, respectively.
*          Also declares the temperature sensor as an input.
*/
void initializeRangeSensors(){
	//Initialize Range Sensor 1
	pinMode(TRIG1, OUTPUT);  // Activate write for trig of range sensor
	pinMode(ECHO1, INPUT);  // Activate read for echo of range sensor

	//Initialize Range Sensor 2
	pinMode(TRIG2, OUTPUT); // Activate write for trig of range sensor
	pinMode(ECHO2, INPUT);  // Activate read for echo of range sensor
	
  //Initialize Temperature Sensor for added accuracy
	pinMode(TEMP_SENSE, INPUT);  // Initialize the temperature sensor as an input    
}

/*
* Name: loop
* Purpose: The superloop that continually runs all the described functionality. TODO fix this.
*/
void loop(){
 
  lcd.clear();
   mode = checkForSwitch(mode);
   
   if(mode == 0){
          Serial.print("mode Basic");
     processMotion();
   }
   else{
     lcd.noDisplay();
          Serial.print("mode Clean");
   findStartingPosition(0);
   }
   
   //edgeDetect();


delay(500);
}

int checkForSwitch(int currentMode){
  int photocellVal = analogRead(PHOTOCELL_BUTTON);
    Serial.print("PHOTOCELL VAL ");
  Serial.print(photocellVal);
  if(photocellVal > FLASHLIGHT_LUX){
    return 1;
  }
  else{
    return 0;
  }
   

     

}
/*
-----------------------------------------------------------------
Basic Functionality
-----------------------------------------------------------------
*/
/*
* Name: processMotion
* Purpose: For the basic functionality, determines
*          the speed and direction the car should move in.
*          If the object is 25+ cm away, the car proceeds
*          at its fastest speed. Otherwise, it gradually
*          slows down till it is 10 cm away from the object,
*          when it stops briefly, and turns left.
*/
void processMotion(){
	 int distance = getDistance(ECHO1, TRIG1); 
        lcd.clear();
         
          lcd.setCursor(0, 0);
          // print the number of seconds since reset:
          lcd.print("Distance: ");
          lcd.print(distance);
          
        if(distance <0){
        }
        
        
  
	if (distance >= NORMAL_DISTANCE){  // If object is 30+ cm away
		carForward(FAST_PACE, FAST_PACE); // Move forward quickly
                lcd.setCursor(0,1);
                lcd.print("Approaching");
	}
	
      else if (distance > VERY_CLOSE_DISTANCE){  // Gradually decreasing speed between 20-30 cm.
                
		int pwmSpeed = calculatePWMSpeed(distance);
		carForward(pwmSpeed, pwmSpeed); // calcPWMSpeed is calculated based on a formula
	        lcd.setCursor(0,1);
                lcd.print("Slowing Down");
}
	else { // Car is 20 or less cm away, stop briefly, then turn left
		carStop();  // Stop
                delay(250);
		carTurnLeft(TURN_PACE, TURN_PACE);  // Turn left
	        lcd.setCursor(0,1);
                lcd.print("Turn Left");
	}
}

/*
-----------------------------------------------------------------
Edge Sensing Functions
-----------------------------------------------------------------
*/
/*
* Name: determineArea
*/
void edgeDetect(){
	int distanceBelow = getDistance(ECHO2, TRIG2); //Obtain the distance from the range sensor parallel to the edge sensor.
        
	if (distanceBelow > 9){ //If the distance from the ground is significantly lower than level ground.
		carStop(); //Stop the car.
		delay(250);
		carReverse(EDGE_DETECT_PACE, EDGE_DETECT_PACE); //Turn back to make room for turning.
		delay(500);
		carTurnLeft(TURN_PACE,TURN_PACE); //Then turn to remain on table.
	}
  else{
    carForward(EDGE_DETECT_PACE, EDGE_DETECT_PACE);
  }
}
/*
-----------------------------------------------------------------
Update and Retrieve Data
-----------------------------------------------------------------
*/
/*
* Name: startRangeSensor
* Purpose: Initializes the range sensor to begin
*          detecting pulse widths by starting the trigger.
*/
void startRangeSensor(int trig){
	digitalWrite(trig, LOW);  // Set up the sensor to start detecting range
	digitalWrite(trig, HIGH); //Send a pulse to the sensor.
	delay(.01);  // 10 usec delay
	digitalWrite(trig, LOW); //Turn off power to make this a 10 usec pulse.
	delay(.2);  // Pause for the 8 bursts to end
}
/*
* Name: updateDistanceAndSpeed
* Purpose: Prepares the range sensor for gathering data
*          and obtains ECHO1 pulse width duration to compute
*          the distance from the range sensor to
*          an object (in cm). Meanwhile, with the distance,
*          calculates the PWM speed that the wheels should
*          move at (slower for closer distances).
*/
float getDistance(int echo, int trig){
	startRangeSensor(trig);  // Start the trigger and wait for 8 bursts from ECHO1
	// Gather range sensor echo data to assist in calculating distance
	int duration2 = pulseIn(echo, HIGH);  // Returns length of pulse in microseconds or 0 if no pulse  
	float speedOfSound = getSoundSpeed();  //Obtain the current speed of sound
        
        
	int distance = (duration2 / speedOfSound) / 2;  // Formula to calculate a more accurate distance 
        if(distance < 0){
        getDistance(echo,trig);
        }
        Serial.print("Distance: ");
        Serial.println(distance);
	return distance;
}
/*
* Name: updateSpeedSound
* Purpose: Using the voltage recieved from the temperature
*          sensor, calculates the temperature(in C) and then the
*          speed of sound(us/cm) using provided formulas.
*/
float getSoundSpeed(){
	float voltage = analogRead(TEMP_SENSE);  // Read the voltage from temperature sensor
        
	// Formula relating the voltage to temperature(C) for this temperature sensor
	float temperature = (voltage/1024.0)*500;// Convert the voltage to degrees Celsius
         Serial.println("temp ");
         temperature = 25.6;
        Serial.println(temperature);
	float speedOfSound = 331.5 + 0.6*temperature;  // Given formula to calculate speed of sound(m/s) using temperature
	speedOfSound = speedOfSound/ 10000; // Convert speed of sound to cm/us
	speedOfSound = 1 / speedOfSound; // Convert speed of sound to us/cm to use in given distance formula.
      
  return speedOfSound;
}
/*
-----------------------------------------------------------------
Calculate PWM Speed
-----------------------------------------------------------------
*/
int calculatePWMSpeed(int distance){
	//int pwmSpeed = 12*distance - 90;  // Formula to decrease speed as object gets closer. Decrease as a function of distance.
        int pwmSpeed = 26.57*pow(2.72, distance*0.075377);
	return pwmSpeed;
}

// void updateDistanceAndSpeed(){
//   distance=getDistance(ECHO1,TRIG1);
// }

void findStartingPosition(int directions) {
  int distance, shortDistance, height;
  int counter = 3;
  // Serial.println("Distance 1");
  
  shortDistance = getDistance(ECHO1, TRIG1);
  
  carTurnLeft(turn_pace, turn_pace);
  delay(170);
  carStop();
  delay(500);
  distance = getDistance(ECHO1, TRIG1);


    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }

  if (distance < shortDistance) {
    shortDistance = distance;
    counter = 2;
  }

  carTurnLeft(turn_pace, turn_pace);
    delay(170);
  carStop();
  delay(500);
  distance = getDistance(ECHO1, TRIG1);
  
  
    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }
  
  if (distance < shortDistance) {
    shortDistance = distance;
    counter = 1;
  }
  
  
    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }
  
  carTurnLeft(turn_pace, turn_pace);
    delay(170);
  carStop();
  delay(500);
  distance = getDistance(ECHO1, TRIG1);
  
  if (distance < shortDistance) {
    shortDistance = distance;
    counter = 0;
  }


    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }

  for(int i = 0; i < counter; i++) {
    carTurnRight(turn_pace, turn_pace);
      delay(170);
    carStop();
    delay(500);
  }
  
  
    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }
  
  distance = getDistance(ECHO1, TRIG1);
  
  while (distance > OBJECT_IN_FRONT) {
    carForward(120*1.15,120);
    delay(100);
    distance = getDistance(ECHO1, TRIG1);
  }
  
  carStop();
  delay(500);

  if (directions == 0) {
    carTurnRight(turn_pace, turn_pace);
      delay(170);
    carStop();
    delay(500);
    }
  else {
    carTurnLeft(turn_pace, turn_pace);
      delay(170);
    carStop();
    delay(500);
  }


    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }

  height = getDistance(ECHO2, TRIG2);
  int heightOfTopSensor = getDistance(ECHO2, TRIG2);

  // Serial.println("Height: " + height);
  
  while (height <= 15) {
    carForward(120*1.15,120);
    delay(100);
    height = getDistance(ECHO2, TRIG2);
  }

  carReverse(120*1.15,120);
  delay(250);

  carTurnRight(turn_pace,turn_pace);
    delay(170);
 // delay(240);
  carStop();
  delay(2000);

    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }
    
  cleanRoom();
}

void cleanRoom() {
  boolean warning = false;
  int height;
  int frontDistance;
  
  if (counterForCleaning > 9 ) {
    findStartingPosition(1);
  } else if(counterForCleaning >= 18){
    carStop();
    delay(3000);
  }

  int heightOfTopSensor = getDistance(ECHO2, TRIG2);
  
  while (!warning) {
    // Serial.println("Enter while");
    carForward(120*1.15,120);
    delay(100);
    
    height = getDistance(ECHO2,TRIG2);
    frontDistance = getDistance(ECHO1, TRIG1);
    

    mode = checkForSwitch(mode);
    
     if(mode == 0){
      return;
    }
    
    else if (height > 15) {
      // Serial.println("First if statement");
      carStop();
      delay(200);
      carReverse(120*1.15,120);
      delay(500);
      warning = true;
      reachedEdge();

    } else if (frontDistance < OBJECT_IN_FRONT) {
      // Serial.println("Second if statement");
      carStop();
      delay(200);
      warning = true;
      reachedEdge();
    }

  }
  carStop();
  delay(1000);
}

void reachedEdge() {
  counterForCleaning++;
  int rand = random(10, 270);
  carTurnLeft(turn_pace,turn_pace);
  delay(rand);
  cleanRoom();
}
  
  
  
  
  /*
-----------------------------------------------------------------
Car Motion Functions
-----------------------------------------------------------------
*/
// Note: Due to mechanical problems, the car moves slightly to
//       the right rather than in a straight line. To correct this,
//       we adjusted the right wheel speed to move 1.01x faster
//       than the left.
/*
* Name: carStop
* Purpose: Stops the motor by reducing left and
*          right motor speeds to 0
*/
void carStop(){ // Stop the motor
	digitalWrite(M1_SPEED_PIN, 0);  // 0 speed to stop left wheel
	digitalWrite(M2_SPEED_PIN, 0);   // 0 speed to stop right wheel
	digitalWrite(M1_DIRECTION_PIN, LOW); // Left wheel off  
	digitalWrite(M2_DIRECTION_PIN, LOW);  // Right wheel off   
	delay(100);  // Delay briefly to show the car has stopped
}
/*
* Name: carForward
* Purpose: Moves the car forward by directing
*          the left/right wheels forward and
*          adjusting speed to speed desired.
*/
void carForward(int leftSpeed, int rightSpeed){  //Move forward 
	digitalWrite(M1_DIRECTION_PIN, HIGH); // Left wheel direction forward
	digitalWrite(M2_DIRECTION_PIN, HIGH); // Right wheel direction forward 
        analogWrite(M1_SPEED_PIN, leftSpeed);   // Adjust left wheel speed
	analogWrite(M2_SPEED_PIN, rightSpeed*wheel_speed_offset);   // Adjust right wheel speed
}
/*
* Name: carReverse
* Purpose: Moves the car in reverse by directing
*          the left/right wheels backwards and
*          adjusting speed to speed desired.
*/
void carReverse(int leftSpeed, int rightSpeed){  //Move backward
	digitalWrite(M1_DIRECTION_PIN, LOW);    // Left wheel direction backward
	digitalWrite(M2_DIRECTION_PIN, LOW);  // Right wheel direction backward  
	analogWrite(M1_SPEED_PIN, leftSpeed); // Adjust left wheel speed
	analogWrite(M2_SPEED_PIN, rightSpeed*wheel_speed_offset);  // Adjust right wheel speed
}
/*
* Name: carTurnLeft
* Purpose: Turns the car left by directing the
*          left wheel backwards and right wheel
*          forwards and adjusting speed to speed desired.
*/
void carTurnLeft(int leftSpeed, int rightSpeed){  //Turn left 
	digitalWrite(M1_DIRECTION_PIN, LOW);  // Left wheel direction backward
	digitalWrite(M2_DIRECTION_PIN, HIGH);  // Right wheel direction forward
	analogWrite(M1_SPEED_PIN, leftSpeed);  // Adjust left wheel speed  
	analogWrite(M2_SPEED_PIN, rightSpeed);  // Adjust right wheel speed
	delay(turning_time);  // For our turning pace, 110 ms delay will produce a right angle turn
}
/*
* Name: carTurnRight
* Purpose: Turns the car right by directing the
*          right wheel backwards and left wheel
*          forwards and adjusting speed to speed desired.
*/
void carTurnRight(int leftSpeed, int rightSpeed){  //Turn right
	digitalWrite(M1_DIRECTION_PIN, HIGH);  // Left wheel forward   
	digitalWrite(M2_DIRECTION_PIN, LOW);  // Right wheel backward 
	analogWrite(M1_SPEED_PIN, leftSpeed);  // Adjust left wheel speed 
	analogWrite(M2_SPEED_PIN, rightSpeed);  // Adjust right wheel speed
	delay(turning_time);  // For our turning pace, 110 ms delay will produce a right angle turn
}

void carSpiral() {
  int rightWheel = 75;
  int leftWheel = 150;

  carForward(leftWheel, rightWheel);
  delay(250);  
}


