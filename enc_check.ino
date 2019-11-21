//=========================================================
//Rotary encoder polling function
//It takes 4usec. (NUCLEO-F401RE 84MHz)
//=========================================================
void rotary_encoder_check()
{
    volatile static int code;
    //check the movement
    code = ( (code<<2) + (((PORTD & _BV(ENCA)) !=0 )<<1) + ((PORTD & _BV(ENCB)) !=0 ) ) & 0x3f ;
    //データ例外ではないかチェック
    if((bitRead(code, 5) == bitRead(code, 1))&&(bitRead(code, 4) == bitRead(code, 0)))
      return;
    //update the encoder value
    int value = -1 * table[code];
    encoder_value += value;
    return;
}

//割り込み関数
void enc_changedPinA()
{
  rotary_encoder_check();
}

void enc_changedPinB()
{
  rotary_encoder_check();
}
