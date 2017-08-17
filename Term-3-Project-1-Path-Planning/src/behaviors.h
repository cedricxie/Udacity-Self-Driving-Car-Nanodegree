#ifndef ADD_BEHAVIOR_INCLUDED
#define ADD_BEHAVIOR_INCLUDED

void lane_keeping(vector<vector<double>>  sensor_car_list_current, double car_s, double prev_s, double s_buffer,
   double &v_init, double &v_end, double car_speed, bool &flag){

  if ( abs(car_speed-v_end) < 0.5 && flag == false) {
       cout << setw(25) << "Target speed reached: " << car_speed << endl;
       flag = true;
       car_v_init_global = car_speed;
       car_v_end_global = car_speed;
       v_init = car_v_init_global;
       v_end = car_v_end_global;
  }

  double idx = -1;

  for(int i=0; i<sensor_car_list_current.size(); i++){

    if(dflag>=dflag_sensor_details){cout << "All cars in the CURRENT lane: " << sensor_car_list_current[sensor_car_list_current.size()-i-1][1]-car_s << endl;}

    if(sensor_car_list_current[sensor_car_list_current.size()-i-1][1]>car_s){

      cout << "Cars in front: " << sensor_car_list_current[sensor_car_list_current.size()-i-1][1]-car_s << endl;

      if (sensor_car_list_current[sensor_car_list_current.size()-i-1][1]<prev_s+ s_buffer){

        double car_v_target = sensor_car_list_current[sensor_car_list_current.size()-i-1][3];
        double car_s_target = sensor_car_list_current[sensor_car_list_current.size()-i-1][1];

        //cout << car_v_target << " " << ((car_v_target-v_end)< 0.5*lane_keeping_buffer_v) << " " << (car_v_target < car_speed_max) << " " << flag << endl;

        if (idx == -1) {

          idx = sensor_car_list_current.size()-i-1;

          //Scenario 1:
          //  a. not during status change;
          //  b. target speed larger than speed limit;
          //  c. current speed target less than speed limit
          //  Action: increase current speed target to speed limit
          //Scenario 2:
          //  either,
          //    a. not during status change;
          //  or,
          //    a. during status change, and target speed is speed limit
          //  and,
          //    b. target speed less than speed limit
          //    c. current speed target larger than target speed - 0.5*speed_buffer
          //  Action: reduce current speed target to target speed - speed_buffer
          //Scenario 3:
          //  a. not during status change;
          //  b. target speed less than speed limit;
          //  c. current speed target less than target speed - 1.5*speed_buffer
          //  Action: increase current speed target to speed - speed_buffer
          //Scenario 4:
          //  a. not during status change;
          //  b. no cars found in front;
          //  Action: increase current speed target to speed limit

          if ( (flag == true) && (car_s_target-car_s<2.0*lane_keeping_buffer) ){
            cout << setw(25) << "Attension: Car too close "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            cout << setw(25) << "Decrease speed to: " << car_v_target - lane_keeping_buffer_v*3.0  << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v*3.0;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if (flag == true && (car_v_target>car_speed_max) && (v_end<car_speed_max)) {
            cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            cout << setw(25) << "Increase speed to: " << car_speed_max  << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_speed_max;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if(  (((flag==false)&&(v_end==car_speed_max))||(flag==true)) && (car_v_target<car_speed_max)&&((car_v_target-v_end)<0.5*lane_keeping_buffer_v) )
          {
            cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            cout << setw(25) << "Decrease speed to: " << car_v_target-lane_keeping_buffer_v << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if(flag == true && car_v_target<car_speed_max && car_v_target-v_end > 1.5*lane_keeping_buffer_v){
            cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            cout << setw(25) << "Increase speed to: " << car_v_target-lane_keeping_buffer_v << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
        }

      }
    }
  }

  if (flag == true && idx == -1 && v_end < car_speed_max - lane_keeping_buffer_v){
    cout << setw(25) << "Attension: detected NO car in front"  << endl;
    cout << setw(25) << "Increase speed to: " << car_speed_max  << endl;
    flag = false;
    car_v_init_global = car_speed;
    car_v_end_global = car_speed_max;
    v_init = car_v_init_global;
    v_end = car_v_end_global;
    s_history.erase( s_history.begin()+10, s_history.end() );
    d_history.erase( d_history.begin()+10, d_history.end() );
  }
}

void lane_changing(double car_s, double prev_s,
  double &v_init, double &v_end, double car_speed,
  double car_d, double &d_init, double &d_end, int direction, bool &flag){

    if (flag == true){
      if (direction < 0){
        cout << setw(25) << "Attension: changing lane to LEFT"  << endl;
        //cout << setw(25) << "Change d to: " << d_end - car_lane_width  << endl;
        car_d_init_global = car_d;
        if (car_d > 2.0*car_lane_width){car_d_end_global = car_d_init-0.05;}
        else {car_d_end_global = car_d_init- car_lane_width+0.35;}
      }
      else{
        cout << setw(25) << "Attension: changing lane to RIGHT"  << endl;
        //cout << setw(25) << "Change d to: " << d_end + car_lane_width  << endl;
        car_d_init_global = car_d;
        if (car_d > 1.0*car_lane_width){car_d_end_global = car_d_init + car_lane_width - 0.75;}
        else {car_d_end_global = car_d_init-0.05;}
      }
      flag = false;
      d_init = car_d_init_global;
      d_end = car_d_end_global;
      if (s_history.size()>10){
        s_history.erase( s_history.begin()+10, s_history.end() );
        d_history.erase( d_history.begin()+10, d_history.end() );
      }
    }

  if ( abs(car_d-car_d_end_global) < 0.01*car_lane_width && flag == false) {
       cout << setw(25) << "Target lane reached: " << car_d << endl;
       flag = true;
       //car_d_init_global = car_d;
       //car_d_end_global = car_d;
       //d_init = car_v_init_global;
       //d_end = car_v_end_global;
  }
}

#endif // ADD_BEHAVIOR_INCLUDED
