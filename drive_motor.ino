void drive_motor(int pwm_width)
{
  //drive the motor in forward
  if(pwm_width>=0)
  {
      //over voltage
      if(pwm_width>100)
      {
          pwm_width = 100;
      }
      //to protect MAX14870
      if(motor_direction == 2)
      {
        protect_MAX14780();
      }

      //forward
      OCR2A = pwm_width * 255;
      digitalWrite(IN2, HIGH);
      led_g = 1;
      motor_direction = 1;
  }
  //drive the motor in reverse
  else
  {
      //calculate the absolute value
      pwm_width = -1 * pwm_width;

      //over voltage
      if(pwm_width>100)
      {
          pwm_width = 100;
      }
      //to protect 
      if(motor_direction == 1)
      {
        protect_MAX14780();
      }
      
      //reverse
      OCR2A = pwm_width * 255;
      digitalWrite(IN2, LOW);
      led_r = 1;
      motor_direction = 2;
  }
}

void protect_MAX14870()
{
  OCR2A = 0;
  digitalWrite(IN2, LOW);
  delay(0.1); //wait 100 usec
}
