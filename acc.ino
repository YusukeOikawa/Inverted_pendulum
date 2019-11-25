float get_acc_data()
{
  imu.readAcc();

  //int x_data = imu.a.x;
  //int z_data = imu.a.z;
  //float theta_deg = atan2(z_data, x_data); //radian

  theta_deg = atan2(imu.a.z, imu.a.x) * RAD_TO_DEG;
  return theta_deg; //thea_deg[degree]
}

void acc_init()
{
    //get data
    float theta_array[sample_num];
    for(int i=0; i<sample_num; i++)
    {
        theta_array[i] = get_acc_data();
        delay( meas_interval );
    }

    //calculate mean
    theta_mean = 0;
    for(int i=0; i<sample_num; i++)
    {
            theta_mean += theta_array[i];
    }
    theta_mean /= sample_num;

    //calculate variance
    float temp;
    theta_variance = 0;
    for(int i=0; i<sample_num; i++)
    {
            temp = theta_array[i] - theta_mean;
            theta_variance += temp*temp;
    }
    theta_variance /= sample_num;
    return;
}
