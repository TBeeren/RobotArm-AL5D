#include <Servo.h>

const int SERVOPIN = 9;
Servo servo;
String message = "";
String positionString = "";
String durationString = "";
bool emergencyStop = false;

double position = 0;
double movementInterval = 0;
double targetPosition = 0;
long duration_millis = 0;
unsigned long previousMillis = 0;
unsigned long movementStartMillis = 0;

void servo_Setup() 
{
  servo.attach(SERVOPIN);
}

bool timerDone(unsigned long interval)
{ 
  for(;;)
  {
    if(millis() - previousMillis > interval) 
    {
      previousMillis = millis();  
      return true;
    }
  }
}

void parse()
{
  char charMessage[30] = {
  };
  message.toCharArray(charMessage, 30);
  int i = 0;
  while(charMessage[i] != '$')
  {
    ++i;
  }
  for(i = 0; i < 20; ++i)
  {
    if(charMessage[i] == ':')
    {
      break;
    }
    else if(isDigit(charMessage[i]))
    {
      positionString += charMessage[i];
    }
  }
  for(i = i; i < 40; ++i)
  {
    if(charMessage[i] == '!')
    {
      break;
    }
    else if(isDigit(charMessage[i]))
    {
      durationString += charMessage[i];
    }
  }
  targetPosition = positionString.toInt();
  duration_millis = durationString.toInt();
  if(targetPosition == 0, duration_millis == 0)
  {
    emergencyStop = true; 
  }
  movementInterval = (double((position - targetPosition)) / double(duration_millis));
  //Serial.print("movementinterval to: ");
  //Serial.println(movementInterval);
  movementStartMillis = millis();
  message = "";
  positionString = "";
  durationString = "";
}

void move()
{
  if(movementInterval < 0.0)
  {
    //Serial.print("move to: ");
    //Serial.println(position);
    if(((targetPosition - 1 >= position) && (position <= targetPosition + 1)))
    {
      //Serial.println("not at the right position");
      position -= movementInterval * ((millis() - movementStartMillis)) ;
      servo.write(position);
    }
  }
  else if (movementInterval > (0.0))
  {
    //Serial.print("move to: ");
    //Serial.println(position);
    if(((targetPosition - 1 <= position) && (position >= targetPosition + 1)))
    {
      //Serial.println("not at the right position");
      position -= movementInterval * ((millis() - movementStartMillis)) ;
      servo.write(position);
    }
  }
  movementStartMillis = millis();

}

int makeAbsolute(int value)
{
  if(value < 0)
  {
    //Serial.println(value * -1);
    return (value * -1);
  }
  return value;
}


void setup() 
{
  Serial.begin(9600);
  servo_Setup();
  servo.write(position);
}

void loop() 
{
  if(!emergencyStop)
  {
    if (Serial.available() > 0) 
    {
      char incomingChar = Serial.read();
      message += incomingChar;
      if(incomingChar == '!')
      {
        Serial.println(message);
        parse();
      }
    }
    if(!emergencyStop)
    {
      move(); 
    }
  } 
}


























