//=========================================================
//Kalman filter for "theta" & "theta_dot_bias"
//It takes 650 usec. (NUCLEO-F401RE 84MHz, BMX055)
//=========================================================
void update_theta()
{
    //detach the rotary encoder polling
    //timer1.detach();

    //measurement data
    float y = get_acc_data(); //degree

    //input data
    float theta_dot_gyro = get_gyro_data(); //degree/sec

    //calculate Kalman gain: G = P'C^T(W+CP'C^T)^-1
    float P_CT[2][1] = {};
    float tran_C_theta[2][1] = {};
    mat_tran(C_theta[0], tran_C_theta[0], 1, 2);//C^T
    mat_mul(P_theta_predict[0], tran_C_theta[0], P_CT[0], 2, 2, 2, 1);//P'C^T
    float G_temp1[1][1] = {};
    mat_mul(C_theta[0], P_CT[0], G_temp1[0], 1,2, 2,1);//CP'C^T
    float G_temp2 = 1.0f / (G_temp1[0][0] + theta_variance);//(W+CP'C^T)^-1
    float G[2][1] = {};
    mat_mul_const(P_CT[0], G_temp2, G[0], 2, 1);//P'C^T(W+CP'C^T)^-1

    //theta_data estimation: theta = theta'+G(y-Ctheta')
    float C_theta_theta[1][1] = {};
    mat_mul(C_theta[0], theta_data_predict[0], C_theta_theta[0], 1, 2, 2, 1);//Ctheta'
    float delta_y = y - C_theta_theta[0][0];//y-Ctheta'
    float delta_theta[2][1] = {};
    mat_mul_const(G[0], delta_y, delta_theta[0], 2, 1);
    mat_add(theta_data_predict[0], delta_theta[0], theta_data[0], 2, 1);

    //calculate covariance matrix: P=(I-GC)P'
    float GC[2][2] = {};
    float I2[2][2] = {{1,0},{0,1}};
    mat_mul(G[0], C_theta[0], GC[0], 2, 1, 1, 2);//GC
    float I2_GC[2][2] = {};
    mat_sub(I2[0], GC[0], I2_GC[0], 2, 2);//I-GC
    mat_mul(I2_GC[0], P_theta_predict[0], P_theta[0], 2, 2, 2, 2);//(I-GC)P'

    //predict the next step data: theta'=Atheta+Bu
    float A_theta_theta[2][1] = {};
    float B_theta_dot[2][1] = {};
    mat_mul(A_theta[0], theta_data[0], A_theta_theta[0], 2, 2, 2, 1);//Atheta
    mat_mul_const(B_theta[0], theta_dot_gyro, B_theta_dot[0], 2, 1);//Bu
    mat_add(A_theta_theta[0], B_theta_dot[0], theta_data_predict[0], 2, 1);//Atheta+Bu

    //predict covariance matrix: P'=APA^T + BUB^T
    float AP[2][2] = {};
    float APAT[2][2] = {};
    float tran_A_theta[2][2] = {};
    mat_tran(A_theta[0], tran_A_theta[0], 2, 2);//A^T
    mat_mul(A_theta[0], P_theta[0], AP[0], 2, 2, 2, 2);//AP
    mat_mul(AP[0], tran_A_theta[0], APAT[0], 2, 2, 2, 2);//APA^T
    float BBT[2][2];
    float tran_B_theta[1][2] = {};
    mat_tran(B_theta[0], tran_B_theta[0], 2, 1);//B^T
    mat_mul(B_theta[0], tran_B_theta[0], BBT[0], 2, 1, 1, 2);//BB^T
    float BUBT[2][2] = {};
    mat_mul_const(BBT[0], theta_dot_variance, BUBT[0], 2, 2);//BUB^T
    mat_add(APAT[0], BUBT[0], P_theta_predict[0], 2, 2);//APA^T+BUB^T

    //attach a timer for the rotary encoder (40 kHz)
    
    //timer1.attach_us(&rotary_encoder_check, rotary_encoder_update_rate);
    
}
