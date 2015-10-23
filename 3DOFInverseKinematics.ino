#include <Servo.h>
#include <math.h>

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;

float upperArm = 95;//mm
float foreArm = 95;

void setup(){
  servoBase.attach(A4, 350, 2450);
  servoShoulder.attach(A3, 350, 2450);
  servoElbow.attach(A2, 350, 2450);
  
  Serial.begin( 19200 );
  Serial.println("Ready");
}
 
void loop(){
  
  xAxis();
  yAxis();
  zAxis();
  

  //homePosition();
  displayXYPerimeter();
  
  circle();
  circle();
  circle();
  circle();
  //rectangle();
  //verticalRTriangle();
}

//x must be positive
void inverseKinematics( float x, float y, float z){
  float base = atan2( x, y );//angle of rotation for base
  float tempZ = z;
  z = sqrt(sq(x) + sq(z)) + sqrt(sq(y) + sq(z)) - sqrt(sq(y));//new z height with arc compensation in x and z dimensions
  
  float wristShoulderSep = sqrt(sq(z) + sq(y));//distance between the wrist and the shoulder
  float zHeightAngle = (isnan(tan(tempZ/y)) == 1) ? 0 : tan(tempZ/y);//angle between the zero height position and the current height, the adjacent angle is y
  float shoulder = acos((sq(wristShoulderSep) + sq(upperArm) - sq(foreArm))/(2*wristShoulderSep*upperArm)) + zHeightAngle;//the angle of the shoulder in the x and z dimensions
  float elbow = acos(( sq(upperArm) + sq(foreArm) - sq(wristShoulderSep) ) / ( 2 * upperArm * foreArm ));//angle of the elbow

  float baseDegrees = 180 - (base * 57);
  float shoulderDegrees = (shoulder * 57) + 35;
  float elbowDegrees = 180 - (elbow * 57);
 
  servoBase.write(baseDegrees);
  servoShoulder.write(shoulderDegrees);
  servoElbow.write(elbowDegrees);

  Serial.println(zHeightAngle);
}

void homePosition(){
  delay(250);
  inverseKinematics(0,35,0);
  delay(1000);
}

void displayXYPerimeter(){
  float outerRadius = 120;
  float innerRadius = 30;
  float angle;
  float x,y,z;
  
  for( angle = 180; angle > 0; angle-- ) {
      x = sqrt(sq(innerRadius * sin( radians( angle )) ));
      y = innerRadius * cos( radians( angle ));
      inverseKinematics( x, y, 0);
      delay( 15 );
  }
  for( float radius = innerRadius; radius < outerRadius; radius++ ) {
      x = sqrt(sq(radius * sin( radians( angle )) ));
      y = radius * cos( radians( angle ));
      inverseKinematics( x, y, 0);
      delay( 15 );
  }
  for( angle = 0; angle < 180; angle++ ) {
      x = sqrt(sq(outerRadius * sin( radians( angle )) ));
      y = outerRadius * cos( radians( angle ));
      inverseKinematics( x, y, 0);
      delay( 15 );
  }
  for( float radius = outerRadius; radius > innerRadius; radius-- ) {
      x = sqrt(sq(radius * sin( radians( angle )) ));
      y = radius * cos( radians( angle ));
      inverseKinematics( x, y, 0);
      delay( 15 );
  }
}

void verticalRTriangle(){
  float adjacent = 50;
  float opposite = 50;
  float hypotenuse = sqrt(sq(adjacent) + sq(opposite));

  for (float y = 25; y < 25+adjacent; y+= 0.25){
    inverseKinematics(0, y, 0);
    delay( 15 );
  }
  for (float z = 0; z < adjacent; z+= 0.25){
    inverseKinematics(0, 25+adjacent, z);
    delay( 15 );
  }
  
  float z = opposite;
  for (float y = 25+adjacent; y > adjacent - 1; y-= 0.25){
    z--;
    inverseKinematics(0, y, z);
    delay( 15 );
  }
}

void rectangle(){
  float width = 20;
  float depth = 20;
  float baseDistanceX = 20;
  float baseDistanceY = -10;
  float baseWidth = width + baseDistanceY;
  float baseDepth = depth + baseDistanceX;
  
  for (float x = baseDistanceX; x < baseDepth; x++){
    inverseKinematics(x, baseDistanceY, 0);
    delay( 15 );
  }
  for (float y = baseDistanceY; y < baseWidth; y++){
    inverseKinematics(baseDepth, y, 0);
    delay( 15 );
  }
  for (float x = baseDepth; x > baseDistanceX; x--){
    inverseKinematics(x, baseWidth, 0);
    delay( 15 );
  }
  for (float y = baseWidth; y > baseDistanceY; y--){
    inverseKinematics(baseDistanceX, y, 0);
    delay( 15 );
  }
}

void xAxis(){
    for( float x = 0; x < 50; x++) {
      inverseKinematics( x, 50, 0);
      delay( 25 );
    }
    for( float x = 50; x > 0; x-- ) {
      inverseKinematics( x, 50, 0);
      delay( 25 );
    }
}


void yAxis(){
    for( float y = 0; y < 50; y++ ) {
      inverseKinematics( 50, y, 0);
      delay( 25 );
    }
    for( float y = 50; y > 0; y-- ) {
      inverseKinematics( 50, y, 0);
      delay( 25 );
    }
}


void zAxis(){
    for( float z = 0; z < 30; z++ ) {
      inverseKinematics( 50, 50, z);
      delay( 25 );
    }
    for( float z = 30; z > 0; z-- ) {
      inverseKinematics( 50, 50, z);
      delay( 25 );
    }
}
 
void circle(){
  float radius = 15;
  float baseX = 80;
  float baseY = 0;
  float x,y,z;
  for( float angle = 0; angle < 360; angle += 2 ) {
      x = radius * sin( radians( angle )) + baseX;
      y = radius * cos( radians( angle )) + baseY;
      inverseKinematics( x, y, 0);
      delay( 25 );
  }
}
