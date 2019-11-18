//=========================================================
//Rotary encoder polling function
//It takes 4usec. (NUCLEO-F401RE 84MHz)
//=========================================================
void rotary_encoder_check()
{
    static int code;
    //check the movement
    code = ( (code<<2) +  int(encoder_bus) ) & 0xf ;
    //update the encoder value
    int value = -1 * table[code];
    encoder_value += value;
    return;
}
