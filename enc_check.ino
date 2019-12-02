//=========================================================
//Rotary encoder polling function
//It takes 4usec. (NUCLEO-F401RE 84MHz)
//=========================================================
//割り込み関数
void rotary_encoder_check()
{
    volatile static int code;
    //check the movement
    //code = ( (code<<2) + ((PORTD & _BV(ENCA)) >> 1) + ((PORTD & _BV(ENCB)) >> 3) ) & 0x3f ;
    code = ( (code<<2) + (digitalRead(2) << 1) + ( digitalRead(3) )) & 0x3f ;
    //Serial.println(encoder_value);     
    //データ例外ではないかチェック
    if((bitRead(code, 5) == bitRead(code, 1))&&(bitRead(code, 4) == bitRead(code, 0)))
      return;
    //update the encoder value
    int value = -1 * table[(code & 0xf)];
    encoder_value += value;
    return;
}
