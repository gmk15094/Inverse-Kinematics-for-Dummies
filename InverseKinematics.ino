// This program calculate the Inverse Kinematics of XY tables points - 15/08/2019
#include <Servo.h>
Servo Srv1, Srv2;

int i; 
int g=32;                                                     // interval between the two servos
float  A1x=-16, A1y=128, A2x=A1x+g, A2y=A1y;                  // Values of servos positions
float  a1=96, c1=32, a2=a1, c2=c1;                            // Values of leg sides lengths

// Drawing a rectangle
int Px[]= {-16,-14,-12,-10, -8, -6, -4, -2,  0,  2,  4,  6,  8, 10, 12, 14, 16,   // Bottom
            16, 14, 12, 10,  8,  6,  4,  2,  0, -2, -4, -6, -8,-10,-12,-14,-16};  // Top
int Py[]= {  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,   // Bottom
            15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};  // Top
int lgTab = sizeof(Px)/sizeof(int);

void  setup() {
  Serial.begin(9600);
  pinMode(0,INPUT_PULLUP);          //start button attachment
  Srv1.attach(3);                   //Left servo attachment
  Srv2.attach(4);                   //Right servo attachment
  Srv1.write(90);                   //set servo to middle point
  Srv2.write(90);                   //set servo to middle point

  while( digitalRead(0) );                                    // waiting for start button pressed
//  for( i=0; i < lgTab; i++) InverseKinematics();            // uncomment for testing with DEBUG
}

void  loop(){
  for( i=0; i < lgTab; i++) InverseKinematics();              // comment when testing with DEBUG
}

void InverseKinematics(){
  float d1=abs(A1y-Py[i]), d2=abs(A2y-Py[i]);                       // 1- value of d1 and d2 sides of the left and right right-angle triangle
  float e1=abs(A1x-Px[i]), e2=abs(A2x-Px[i]);                       // 2- value of e1 and e2 sides of the left and right right-angle triangle
  float b1=sqrt((d1*d1)+(e1*e1)), b2=sqrt((d2*d2)+(e2*e2));         // 3- value of b1 and b2 sides (hypotenuse) of the left and right right-angle triangle 
  if ( b1>(a1+c1) || b2>(a1+c1) ){                                  // the target point must be reachable
    Serial.print("\n\t Target point Px=");Serial.print(Px[i]);Serial.print(" Py=");Serial.print(Py[i]);Serial.print(" is too far. Impossible to reach !!!");
    return;
  }
  float A_1=(acos(((b1*b1)+(c1*c1)-(a1*a1))/(2*b1*c1)))*57.296;     // 4- Value of the A angle of the left triangle and degree conversion
  float A_2=(acos(((b2*b2)+(c2*c2)-(a2*a2))/(2*b2*c2)))*57.296;     // 5- Value of the A angle of the right triangle and degree conversion
  float D1=(acos(((g*g)+(b1*b1)-(b2*b2))/(2*b1*g)))*57.296;         // 6- Value of the D left angle of the center triangle and degree conversion
  float D2=(acos(((g*g)+(b2*b2)-(b1*b1))/(2*b2*g)))*57.296;         // 7- Value of the D right angle of the center triangle and degree conversion

  int S1=round(180-A_1-D1);                                         // 8- Value of the target angle of the left servo
  int S2=round(A_2+D2);                                             // 9- Value of the target angle of the right servo
  if ( S1<5 || S2>175 ){                                            // the target point must not be on the end position of the servos because could be trouble 
    Serial.print("\n\t Target point Px= ");Serial.print(Px[i]);Serial.print(" Py= ");Serial.print(Py[i]);Serial.print(" is too far. Impossible to reach !!!");
    return;
  }
  Srv1.write(S1);                                                   // set target servo position
  Srv2.write(S2);                                                   // set target servo position
  delay(50);  

/*
  // DEBUG 1
  Serial.print(" Px=");Serial.print(Px[i]);Serial.print("\tPy=");Serial.print(Py[i]);
  Serial.print("\t\tS1=");Serial.print(S1);Serial.print("°\tS2=");Serial.print(S2);Serial.print("°\n");
*/
 
/*
  // DEBUG 2
  Serial.print("\n\t Position to reach : Px=");Serial.print(Px[i]);Serial.print("  Py=");Serial.print(Py[i]);Serial.print("\n\n");
  Serial.print("\t Left servo \t\t Right servo\n\n");  
  Serial.print("\t A1x=");Serial.print(A1x);Serial.print("\t\t A2x=");Serial.print(A2x);Serial.print("\n");
  Serial.print("\t A1y=");Serial.print(A1y);Serial.print("\t\t A2y=");Serial.print(A2y);Serial.print("\n");
  Serial.print("\t a1=");Serial.print(a1);Serial.print("\t\t a2=");Serial.print(a2);Serial.print("\n");
  Serial.print("\t c1=");Serial.print(c1);Serial.print("\t\t c2=");Serial.print(c2);Serial.print("\n");  
  Serial.print("\t d1=");Serial.print(d1);Serial.print("\t\t d2=");Serial.print(d2);Serial.print("\n");
  Serial.print("\t e1=");Serial.print(e1);Serial.print("\t\t e2=");Serial.print(e2);Serial.print("\n");
  Serial.print("\t b1=");Serial.print(b1);Serial.print("\t\t b2=");Serial.print(b2);Serial.print("\n");
  Serial.print("\t A_1=");Serial.print(A_1);Serial.print("°\t\t A_2=");Serial.print(A_2);Serial.print("°\n");
  Serial.print("\t D1=");Serial.print(D1);Serial.print("°\t\t D2=");Serial.print(D2);Serial.print("°\n\n");
  Serial.print("\t Result of calculations, angles of the servos\n\n");
  Serial.print("\t S1=");Serial.print(S1);Serial.print("°\t\t\t S2=");Serial.print(S2);Serial.print("°\n\n\n");
*/
}

/*
// Exemples

// Drawing a rectangle
int Px[]= {-16,-14,-12,-10, -8, -6, -4, -2,  0,  2,  4,  6,  8, 10, 12, 14, 16,   // Bottom
            16, 14, 12, 10,  8,  6,  4,  2,  0, -2, -4, -6, -8,-10,-12,-14,-16};  // Top
int Py[]= {  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,   // Bottom
            15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};  // Top

// Drawing a vertical ligne
int Px[]= {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
int Py[]= {  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
            30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6,  4}; 

// Drawing an horizontal ligne
int Px[]= {-16,-14,-12,-10, -8, -6, -4, -2,  0,  2,  4,  6,  8, 10, 12, 14, 16,
            16, 14, 12, 10,  8,  6,  4,  2,  0, -2, -4, -6, -8,-10,-12,-14,-16};
int Py[]= { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
            10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};

// Simple rectangle. DEBUG 2 values can be compared with the Excel file
int Px[]= {-15,  0, 15, 15,  0,-15};
int Py[]= {  5,  5,  5, 10, 10, 10};
            
*/
