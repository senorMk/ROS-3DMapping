/*
************************************************************
** Author: Penjani Mkandawire
** Date: 10/12/2021
** Email: mkandawire15@gmail.com
** Based on code from the XiaoR Geek UNO DS
** Function name ：setup().InitSteer()
** Function ：System initialization (serial port, motor, steering gear ).
************************************************************
*/

// Servo initialization (angle is the last saved value)
void InitSteer()
{
    // Read values from register 0x01 to 0x04
    ServoAngle1 = EEPROM.read(0x01);
    ServoAngle2 = EEPROM.read(0x02);
    ServoAngle3 = EEPROM.read(0x03);
    ServoAngle4 = EEPROM.read(0x04);

    // Assign saved angles to servo 1 to 4
    Servo1.write(ServoAngle1);
    Servo2.write(ServoAngle2);
    Servo3.write(ServoAngle3);
    Servo4.write(ServoAngle4);

    // Assign the saved angle to servo 10 if it equals 0xff
    Adjust = EEPROM.read(0x10);
    if(Adjust == 0xff)
    {
      EEPROM.write(0x10,1);
    }

    // Read the saved speed values from registers 0x09 and 0x0A
    LeftSpeedHold = EEPROM.read(0x09);
    RightSpeedHold = EEPROM.read(0x0A);
    
    if(LeftSpeedHold < 55 | RightSpeedHold < 55)
    {
      LeftSpeedHold = 255;
      RightSpeedHold = 255;
    }

    // Assign the speed values to L298 Enable A and B
    analogWrite(EnableB,LeftSpeedHold);
    analogWrite(EnableA,RightSpeedHold);
    
    MotorStop;
}
