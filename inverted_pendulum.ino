//=========================================================
//arduiono nano
//LSM6
//MAX14870 Single Brushed DC Motor Driver Carrier
//=========================================================
#include <Wire.h>
#include <LSM6.h>

//#define DEBUG_FLG

//=========================================================
//IMU
LSM6 imu;//raw data

//=========================================================
//Port Setting
#define ENCA PD2 //エンコーダA相(blue) pin 2
#define ENCB PD3 //エンコーダB相(purple) pin3 
#define IN_R PB4 //右モータドライバのDIRに接続 pin 12
#define IN_PWM 10 //モータドライバのPWMに接続　 (OC2A) pin 10
#define IN_L PB3 //モータドライバのDIRに接続 pin 11
#define led_r PB1 //Red LED pin 9
#define led_g PB0 //Green LED pin 8
#define led_y PD7 //Yellow LED pin7

//=========================================================
//Ticker
#include <FlexiTimer2.h>

//=========================================================
#define max_voltage 10.0 //モータの最大出力電圧

//=========================================================
//Accelerometer and gyro statistical data
#define sample_num 100
#define meas_interval 10.0
float theta_mean; //平均
float theta_variance; //角度分散
float theta_dot_mean; //角速度平均
float theta_dot_variance; //角速度分散

//=========================================================
//Rotary encoder variables
//int rotary_encoder_update_rate = 25; //usec //タイマー割り込みでなく外部割り込みを使う
//int rotary_encoder_resolution = 100;
int rotary_encoder_resolution = 159; //エンコーダ1回転あたりのパルス数
int encoder_value = 0;
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
float pre_theta2 = 0;

//=========================================================
//Kalman filter (for angle estimation) variables
//Update rate
float theta_update_freq = 400; //Hz
float theta_update_interval = 1.0f/theta_update_freq;
//State vector
//[[theta(degree)], [offset of theta_dot(degree/sec)]]
float theta_data_predict[2][1];
float theta_data[2][1];
//Covariance matrix
float P_theta_predict[2][2];
float P_theta[2][2];
//"A" of the state equation
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}};
//"B" of the state equation
float B_theta[2][1] = {{theta_update_interval}, {0}};
//"C" of the state equation
float C_theta[1][2] = {{1, 0}};

//=========================================================
//Kalman filter (for all system estimation) variables
//State vector
//[[theta1(rad)], [theta1_dot(rad/s)], [theta2(rad)]. [theta2_dot(rad/s)]]
float x_data_predict[4][1];
float x_data[4][1];
//Covariance matrix
float P_x_predict[4][4];
float P_x[4][4];
//"A" of the state equation (update freq = 100 Hz)
float A_x[4][4] = {
{1.00195896e+00,1.00065335e-02,0.00000000e+00,1.00021480e-04},
{3.91381536e-01,1.00195896e+00,0.00000000e+00,1.97302912e-02},
{-7.96877557e-04,-2.67469469e-06,1.00000000e+00,9.58496277e-03},
{-1.57192498e-01,-7.96877557e-04,0.00000000e+00,9.18154381e-01},
};
//"B" of the state equation (update freq = 100 Hz)
float B_x[4][1] = {
{-2.6048e-04},
{-5.138297e-02},
{1.08087e-03},
{2.1314796e-01}
};
//"C" of the state equation (update freq = 100 Hz)
float C_x[4][4] = {
{1, 0, 0, 0},
{0, 1, 0, 0},
{0, 0, 1, 0},
{0, 0, 0, 1}
};

//measurement noise
float measure_variance_mat[4][4];
//System noise
float voltage_error = 0.01; //volt
float voltage_variance = voltage_error * voltage_error;

//=========================================================
//Motor control variables
float feedback_rate = 0.0025; //sec
float motor_value = 0;
int pwm_width = 0;
int motor_direction = 2;
float motor_offset = 0.38; //volt オフセット電圧

//=========================================================
//Gain vector for the state feedback
//(R=1000, Q = diag(1, 1, 10, 10), f=100Hz)
//float Gain[4] = {29.87522919, 4.59857246, 0.09293, 0.37006248};
//float Gain[4] = {34.07653093, 5.61162729, 0.09385255, 0.7993937};
float Gain[4] = {33.1442468, 5.45205260, 0.0139280243, 0.768819931};
//float Gain[4] = {0, 0, 0.0139280243, 0.768819931};
//float Gain[4] = {0.0, 1.61162729, 0.0, 0.0};


//=========================================================
// Acc & Gyro
float theta_deg; //degree
float y_data; //degree/sec
int time_temp;


void setup() {
  //Serial.begin(9600);
  //Serial.print("Initialising...");
  //noInterrupts(); //割り込み禁止

  //-------------------------------------------
  //LED
  //-------------------------------------------
  led_init();
  PORTD |= _BV(led_y); //turns on the yellow LED
  //Serial.println("LED initialization done");

  Wire.begin();

  //imu設定
  if (!imu.init())
  {
    //Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  //Serial.println("imu initialization done");

  //-------------------------------------------
  //Rotary encoder initialization
  //-------------------------------------------
  encoder_value = 0;
  //DDRD |= _BV(ENCA);
  //DDRD |= _BV(ENCB);
  //PORTD |= _BV(ENCA); //内部プルアップを有効にする
  //PORTD |= _BV(ENCB);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, rotary_encoder_check, CHANGE);
  attachInterrupt(1, rotary_encoder_check, CHANGE);

  //Serial.println("Rotary encoder initialization done");

  //-------------------------------------------
  //Motor driver intialization
  //モータ出力と Fast PWM 設定
  //-------------------------------------------
  DDRB |= _BV(IN_R);
  // DDRB |= _BV(IN_PWM);
  pinMode(IN_PWM, OUTPUT);
  DDRB |= _BV(IN_L);
  /*
  TCCR2A  = bit(COM2A1);  // OC2A None-inverted Mode (1 0)
  TCCR2A |= bit(COM2B1); // OC2B None-Inverted Mode  (1 0)
  TCCR2A |= bit(WGM11)  | bit(WGM10);  // Fast PWM (0xFF)
  TCCR2B = bit(CS21) | bit(CS20); // clock/32
  OCR1B = 0; //pin 10 (PWM)
  */
  analogWrite(IN_PWM, 0);
  PORTB &= ~_BV(IN_R);
  PORTB &= ~_BV(IN_L);

  //Serial.println("Motor driver initialization done");


  #ifdef DEBUG_FLG
  #else

  //-------------------------------------------
  //Accelerometer & Gyro initialization
  //-------------------------------------------
  //Serial.println("Accelerometer & Gyro Initializing...");
  acc_init();
  gyro_init();
  //Serial.println("Accelerometer & Gyro Initialization done");
  #endif
}

void loop() {
  #ifdef DEBUG_FLG
    //Serial.println("DEBUG");
    led_test();
  #else
    //=========================================================
    // Main
    //=========================================================

    //-------------------------------------------
    //Kalman filter (angle) initialization
    //-------------------------------------------
    //initial value of theta_data_predict
    theta_data_predict[0][0] = 0;
    theta_data_predict[1][0] = theta_dot_mean;

    //initial value of P_theta_predict
    P_theta_predict[0][0] = 1;
    P_theta_predict[0][1] = 0;
    P_theta_predict[1][0] = 0;
    P_theta_predict[1][1] = theta_dot_variance;

    //-------------------------------------------
    //Kalman filter (all system) variables
    //-------------------------------------------
    //variable for measurement data
    float y[4][1];

    //variables for Kalman gain calculation
    float theta1_dot_temp;
    float tran_C_x[4][4];
    float P_CT[4][4];
    float G_temp1[4][4];
    float G_temp2[4][4];
    float G_temp2_inv[4][4];
    float G[4][4];

    //variables for x_hat estimation
    float C_x_x[4][1];
    float delta_y[4][1];
    float delta_x[4][1];

    //variables for covariance matrix calculation
    float GC[4][4];
    float I4[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    float I4_GC[4][4];

    //variables for x prediction
    float Vin;
    float A_x_x[4][1];
    float B_x_Vin[4][1];

    //variables for covariance prediction
    float tran_A_x[4][4];
    float AP[4][4];
    float APAT[4][4];
    float BBT[4][4];
    float tran_B_x[1][4];
    float BUBT[4][4];

    //-------------------------------------------
    //Kalman filter (all system) initialization
    //-------------------------------------------
    //initial value of x_data_predict
    for(int i=0; i<4; i++)
    {
        x_data_predict[i][0] = 0;
    }

    //initial value of P_x_predict
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            P_x_predict[i][j] = 0;
        }
    }
    for(int i=0; i<4; i++)
    {
        P_x_predict[i][i] = 1e-4;
    }

    //measurement noise matrix
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            measure_variance_mat[i][j] = 0;
        }
    }
    float deg_rad_coeff = DEG_TO_RAD * DEG_TO_RAD; //(3.14*3.14)/(180*180);
    measure_variance_mat[0][0] = theta_variance * deg_rad_coeff;
    measure_variance_mat[1][1] = theta_dot_variance * deg_rad_coeff;
    float encoder_error = 0.1f*2*3.14f/(4*rotary_encoder_resolution);
    measure_variance_mat[2][2] = encoder_error * encoder_error;
    float encoder_rate_error = encoder_error / feedback_rate;
    measure_variance_mat[3][3] = encoder_rate_error * encoder_rate_error;

    //-------------------------------------------
    //Timer
    //-------------------------------------------
    //timer1: rotary encoder polling, 40 kHz
    //timer1.attach_us(&rotary_encoder_check, rotary_encoder_update_rate);
    //timer2: Kalman filter (theta & theta_dot), 400 Hz
    //timer2.attach(&update_theta, theta_update_interval);
    //FlexiTimer2::set(1, 1.0/400, update_theta);
    //FlexiTimer2::start();

    //-------------------------------------------
    //initialization done
    //-------------------------------------------
    //Serial.println("done");
    //PORTD &= ~_BV(led_y);
    //interrupts(); //割り込み許可
    PORTD &= ~_BV(led_y);
    

    //===========================================
    //Main loop
    //it takes 700 usec (calculation)
    //===========================================
    while(1)
    {
      //imu.read();
      //get_acc_data();
      //get_gyro_data();
        //imu.readAcc();
        //imu.readGyro();
        //PORTB |= _BV(led_g);
        //imu.read();
        //FlexiTimer2::stop();
        update_theta();
        //stop theta update process
        //timer2.detach();
        //noInterrupts(); //割り込み禁止
        

        //turn off LEDs
        PORTB &= ~_BV(led_g);
        PORTB &= ~_BV(led_r);

        //---------------------------------------
        //Kalman Filter (all system)
        //---------------------------------------
        //measurement data
        //測定　データ
        y[0][0] = (theta_data[0][0] - 90) * DEG_TO_RAD; //本体回転角度[rad]
        theta1_dot_temp = float(imu.g.y) / 131.0; //[degree/sec] 
        y[1][0] = (theta1_dot_temp - theta_data[1][0]/*角速度オフセット*/) * DEG_TO_RAD; //本体の角速度[rad/sec]
        y[2][0] = encoder_value * (2*3.14f)/(4*rotary_encoder_resolution); //エンコーダー回転角度[rad]
        y[3][0] = (y[2][0] - pre_theta2)/feedback_rate; //エンコーダ角速度[rad/sec]
/*
        //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
        mat_tran(C_x[0], tran_C_x[0], 4, 4);//C^T
        mat_mul(P_x_predict[0], tran_C_x[0], P_CT[0], 4, 4, 4, 4);//P'C^T
        mat_mul(C_x[0], P_CT[0], G_temp1[0], 4, 4, 4, 4);//CPC^T
        mat_add(G_temp1[0], measure_variance_mat[0], G_temp2[0], 4, 4);//W+CP'C^T
        mat_inv(G_temp2[0], G_temp2_inv[0], 4, 4);//(W+CP'C^T)^-1
        mat_mul(P_CT[0], G_temp2_inv[0], G[0], 4, 4, 4, 4); //P'C^T(W+CP'C^T)^-1

        //x_data estimation: x = x'+G(y-Cx')
        mat_mul(C_x[0], x_data_predict[0], C_x_x[0], 4, 4, 4, 1);//Cx'
        mat_sub(y[0], C_x_x[0], delta_y[0], 4, 1);//y-Cx'
        mat_mul(G[0], delta_y[0], delta_x[0], 4, 4, 4, 1);//G(y-Cx')
        mat_add(x_data_predict[0], delta_x[0], x_data[0], 4, 1);//x'+G(y-Cx')

        //calculate covariance matrix: P=(I-GC)P'
        mat_mul(G[0], C_x[0], GC[0], 4, 4, 4, 4);//GC
        mat_sub(I4[0], GC[0], I4_GC[0], 4, 4);//I-GC
        mat_mul(I4_GC[0], P_x_predict[0], P_x[0], 4, 4, 4, 4);//(I-GC)P'

        //predict the next step data: x'=Ax+Bu
        Vin = motor_value;
        if(motor_value > max_voltage)
        {
            Vin = max_voltage;
        }
        if(motor_value < -max_voltage)
        {
            Vin = -max_voltage;
        }
        mat_mul(A_x[0], x_data[0], A_x_x[0], 4, 4, 4, 1);//Ax_hat
        mat_mul_const(B_x[0], Vin , B_x_Vin[0], 4, 1);//Bu
        mat_add(A_x_x[0], B_x_Vin[0], x_data_predict[0], 4, 1);//Ax+Bu

        //predict covariance matrix: P'=APA^T + BUB^T
        mat_tran(A_x[0], tran_A_x[0], 4, 4);//A^T
        mat_mul(A_x[0], P_x[0], AP[0], 4, 4, 4, 4);//AP
        mat_mul(AP[0], tran_A_x[0], APAT[0], 4, 4, 4, 4);//APA^T
        mat_tran(B_x[0], tran_B_x[0], 4, 1);//B^T
        mat_mul(B_x[0], tran_B_x[0], BBT[0], 4, 1, 1, 4);//BB^T
        mat_mul_const(BBT[0], voltage_variance, BUBT[0], 4, 4);//BUB^T
        mat_add(APAT[0], BUBT[0], P_x_predict[0], 4, 4);//APA^T+BUB^T
*/

        //---------------------------------------
        //Motor control
        //---------------------------------------
        //reset
        motor_value = 0;

        for(int i=0; i<4; i++)
        {
            x_data[i][0] = y[i][0];
        }
        //Serial.println(y[1][0]);
        //Serial.println(pwm_width);

        //calculate Vin
        for(int i=0; i<4; i++)
        {
            motor_value += Gain[i] * x_data[i][0];
        }

        //offset
        if(motor_value > 0)
        {
            motor_value += motor_offset ;
        }
        if(motor_value < 0)
        {
            motor_value -= motor_offset;
        }

        //calculate PWM pulse width
        pwm_width = int( motor_value*100.0f/max_voltage );

        drive_motor(pwm_width);
        //Serial.println(pwm_width);

        // prepare for the next calculation of theta2_dot
        pre_theta2 = y[2][0];
        // start the angle update process
        //timer2.attach(&update_theta, theta_update_interval);
        //interrupts(); //割り込み許可
        //FlexiTimer2::start();
        //Serial.println(micros()- time_temp);
        // wait
        //delay(feedback_rate * 1000);
        //Serial.println(pwm_width);
        delayMicroseconds(578);  //2.5ms 周期
    }
    //===========================================
    //Main loop (end)
    //===========================================
  #endif
}
