void led_init()
{
  DDRB |= _BV(led_r);
  DDRB |= _BV(led_g);
  DDRD |= _BV(led_y);

  
  PORTB &= ~_BV(led_r);
  PORTB &= ~_BV(led_g);
  PORTD &= ~_BV(led_y);
  
  delay(100);   //wait 0.1 sec
}

void led_test()
{
  PORTB |= _BV(led_r);
  PORTB |= _BV(led_g);
  PORTD |= _BV(led_y);
  delay(3000);
  PORTB &= ~_BV(led_r);
  PORTB &= ~_BV(led_g);
  PORTD &= ~_BV(led_y);
  delay(3000);
}
