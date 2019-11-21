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
        protect_MAX14870();
      }

      //forward
      OCR2A = pwm_width * 255; //pin 11
      PORTB |= _BV(IN_R);
      PORTB &= ~_BV(IN_L);
      PORTB |= _BV(led_g);
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
        protect_MAX14870();
      }
      
      //reverse
      OCR2A = pwm_width * 255; //pin 11
      PORTB &= ~_BV(IN_R);
      PORTB |= _BV(IN_L);
      PORTB |= _BV(led_r);
      motor_direction = 2;
  }
}

void protect_MAX14870()
{
  OCR2A = 0;
  PORTB &= ~_BV(IN_R);
  PORTB &= ~_BV(IN_L);
  delay(0.1); //wait 100 usec
}
