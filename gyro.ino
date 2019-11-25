float get_gyro_data()
{
  imu.readGyro();

  //int y_data = imu.g.y;

  y_data = float(imu.g.y) / 131;
  return y_data; //degree/sec
}

void gyro_init()
{
    //get data
    float theta_dot_array[sample_num];
    for(int i=0;i<sample_num;i++)
    {
      theta_dot_array[i] = get_gyro_data();
      delay(meas_interval);
    }

    //calculate mean　平均
    theta_dot_mean = 0;
    for(int i=0;i<sample_num;i++)
    {
        theta_dot_mean += theta_dot_array[i];
    }
    theta_dot_mean /= sample_num;

    //calculate variance
    float temp;
    theta_dot_variance = 0;
    for(int i=0; i<sample_num; i++)
    {
        temp = theta_dot_array[i] - theta_dot_mean;
        theta_dot_variance += temp*temp;
    }
    theta_dot_variance /= sample_num;
    return;
}
