typedef struct
{
  int powerMotor;
} MOTOR_INFO;

MOTOR_INFO MotorInfo;

void CheckPower(MOTOR_INFO &motorInfo)
{
  int power = motorInfo.powerMotor;

  debugPrintLine("Power=%d", motorInfo.powerMotor);

  if (motorInfo.powerMotor == -1)
  {
    debugPrintLine("struct power is -1!");
  }

  if (power == -1)
  {
    debugPrintLine("Local power is -1!");
  }
}

task main()
{
  while (true)
  {
    MotorInfo.powerMotor = -1;
    CheckPower(MotorInfo);
    wait1Msec(500);
  }
}
