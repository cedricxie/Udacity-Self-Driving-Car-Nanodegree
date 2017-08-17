#ifndef ADD_COST_INCLUDED
#define ADD_COST_INCLUDED

double comfort = 0.04;
double collision = 1.0;
double efficiency = 0.9;

double cost_lane_change(double d_init, double d_end){

  //cout << setw(25) << "start of calculating cost of lane chagnge " << endl;
  if( abs(d_init - d_end)< 0.01*car_lane_width ){
    return 0.0;
  }
  else{
    return comfort;
  }
}

double cost_speed(double car_s, double car_speed, vector<vector<double>> sensor_car_list_end){

  double car_speed_expected;
  int idx = -1;

  for(int i=0; i<sensor_car_list_end.size(); i++){
    if(sensor_car_list_end[sensor_car_list_end.size()-i-1][1]>car_s && sensor_car_list_end[sensor_car_list_end.size()-i-1][1]<car_s+3.0*car_speed*t_inc){
      if(idx == -1){idx = sensor_car_list_end.size()-i-1;}
    }
  }
  if (idx != -1){
    car_speed_expected = sensor_car_list_end[idx][3];
    if (car_speed_expected>car_speed_max) {car_speed_expected=car_speed_max;}
  }
  else{
    car_speed_expected = car_speed_max;
  }

  if (dflag>=dflag_cost_details) {cout << "Expected car speed: " << car_speed_expected << endl;}

  return efficiency*(1.0-car_speed_expected/car_speed_max);
}

double cost_collision(double car_s, double car_speed, vector<vector<double>> sensor_car_list_end){

  //cout << setw(25) << "start of calculating cost of collision " << endl;
  int idx = -1;

  for(int i=0; i<sensor_car_list_end.size(); i++){

    //cout << "All cars in the lane: " << sensor_car_list_end[sensor_car_list_end.size()-i-1][1]-car_s << endl;
    if(sensor_car_list_end[sensor_car_list_end.size()-i-1][1]>car_s){
      if (idx == -1) {idx = sensor_car_list_end.size()-i-1;}
    }
  }
  //cout << setw(25) << "cost_collision idx: " << idx << " out of " << sensor_car_list_end.size() << endl;

  if ((idx==-1) && (sensor_car_list_end.size()!=0) ){
    double s1 = sensor_car_list_end[0][1];
    double v1 = sensor_car_list_end[0][3];
    if( s1+v1*t_inc+0.5*lane_change_buffer > car_s+car_speed*t_inc){
      cout << "CAUTIOUS!!! " << s1 - car_s << " " << v1 - car_speed << endl;
      return collision;
    }
    else {return 0.0;}
  }
  else if( (idx==sensor_car_list_end.size()-1) && (sensor_car_list_end.size()!=0)){
    double s2 = sensor_car_list_end[idx][1];
    double v2 = sensor_car_list_end[idx][3];
    if( s2+v2*t_inc < car_s+car_speed*t_inc+lane_change_buffer ){
      cout << "CAUTIOUS!!! " << s2 - car_s << " " << v2 - car_speed << endl;
      return collision;
    }
    else {return 0.0;}
  }
  else if(idx != -1){
    double s1 = sensor_car_list_end[idx+1][1];
    double v1 = sensor_car_list_end[idx+1][3];
    double s2 = sensor_car_list_end[idx][1];
    double v2 = sensor_car_list_end[idx][3];
    if( s1+v1*t_inc+0.5*lane_change_buffer > car_s+car_speed*t_inc){
      cout << "CAUTIOUS!!! " << s1 - car_s << " " << v1 - car_speed << endl;
      return collision;
    }
    else if( s2+v2*t_inc < car_s+car_speed*t_inc+lane_change_buffer ){
      cout << "CAUTIOUS!!! " << s2 - car_s << " " << v2 - car_speed << endl;
      return collision;
    }
    else {return 0.0;}
  }
  else{return 0.0;}
}

double cost_evaluation(vector<double> car_states, vector<vector<double>> sensor_car_list_current){

  double cost_total=0;
  double cost_comfort=0, cost_safety=0, cost_efficiency=0;

  cost_comfort = cost_lane_change(car_states[1], car_states[2]);
  if (dflag>=dflag_cost) {cout << setw(25) << "cost for comfort: " << cost_comfort << endl;}
  cost_efficiency = cost_speed(car_states[0], car_states[3], sensor_car_list_current);
  if (dflag>=dflag_cost) {cout << setw(25) << "cost for efficiency: " << cost_efficiency << endl;}
  if (abs(car_states[1]-car_states[2]) > 0.01*car_lane_width )
  {cost_safety = cost_collision(car_states[0], car_states[3], sensor_car_list_current);}
  if (dflag>=dflag_cost) {cout << setw(25) << "cost for safety: " << cost_safety << endl;}

  cost_total = cost_comfort + cost_safety + cost_efficiency;

  return cost_total;
}

int next_state(double car_s, double car_d, double car_speed, vector<vector<double>> sensor_car_list_left,
  vector<vector<double>> sensor_car_list_mid, vector<vector<double>> sensor_car_list_right){

    vector<double> car_states;
    double cost_lane_change_left, cost_lane_change_right, cost_lane_keeping;

    if (car_d>2.0*car_lane_width){

      if(dflag>=dflag_cost) {cout << setw(25) << "Currently in right lane: "<< endl;}

      if(dflag>=dflag_cost_details) {cout << setw(25) << "Cost details for lane changing to left: " << endl;}
      car_states = {car_s, car_d, car_d-car_lane_width, car_speed};
      cost_lane_change_left = cost_evaluation(car_states, sensor_car_list_mid);
      if(dflag>=dflag_cost_details) {cout << setw(40) << "Cost total for lane changing to left: " << cost_lane_change_left << endl;}

      if(dflag>=dflag_cost_details) {cout << setw(25) << "Cost details for lane keeping: " << endl;}
      car_states = {car_s, car_d, car_d, car_speed};
      cost_lane_keeping = cost_evaluation(car_states, sensor_car_list_right);
      if(dflag>=dflag_cost_details) {cout << setw(40) << "Cost total for lane keeping: " << cost_lane_keeping << endl;}

      if (cost_lane_change_left < cost_lane_keeping){return -1;}
      else{return 0;}
    }
    else if(car_d>1.0*car_lane_width){

      if(dflag>=dflag_cost) {cout << setw(25) << "Currently in mid lane: "<< endl;}

      if(dflag>=dflag_cost_details) {cout << setw(25) << "Cost details for lane changing to left: " << endl;}
      car_states = {car_s, car_d, car_d-car_lane_width, car_speed};
      cost_lane_change_left = cost_evaluation(car_states, sensor_car_list_left);
      if(dflag>=dflag_cost_details) {cout << setw(40) << "Cost total for lane changing to left: " << cost_lane_change_left << endl;}

      if(dflag>=dflag_cost_details) {cout << setw(25) << "Cost details for lane keeping: " << endl;}
      car_states = {car_s, car_d, car_d, car_speed};
      cost_lane_keeping = cost_evaluation(car_states, sensor_car_list_mid);
      if(dflag>=dflag_cost_details) {cout << setw(40) << "Cost total for lane keeping: " << cost_lane_keeping << endl;}

      if(dflag>=dflag_cost_details) {cout << setw(25) << "Cost details for lane changing to right: " << endl;}
      car_states = {car_s, car_d, car_d+car_lane_width, car_speed};
      cost_lane_change_right = cost_evaluation(car_states, sensor_car_list_right);
      if(dflag>=dflag_cost_details) {cout << setw(40) << "Cost total for lane changing to right: " << cost_lane_change_right << endl;}

      if (cost_lane_change_left < cost_lane_keeping && cost_lane_change_left < cost_lane_change_right){return -1;}
      else if(cost_lane_change_right < cost_lane_keeping && cost_lane_change_right < cost_lane_change_left){return 1;}
      else{return 0;}
    }
    else{
      if(dflag>=dflag_cost) {cout << setw(25) << "Currently in left lane: "<< endl;}

      if(dflag>=dflag_cost_details) {cout << setw(25) << "Cost details for lane keeping: " << endl;}
      car_states = {car_s, car_d, car_d, car_speed};
      cost_lane_keeping = cost_evaluation(car_states, sensor_car_list_left);
      if(dflag>=dflag_cost_details) {cout << setw(40) << "Cost total for lane keeping: " << cost_lane_keeping << endl;}

      if(dflag>=dflag_cost_details) {cout << setw(25) << "Cost details for lane changing to right: " << endl;}
      car_states = {car_s, car_d, car_d+car_lane_width, car_speed};
      cost_lane_change_right = cost_evaluation(car_states, sensor_car_list_mid);
      if(dflag>=dflag_cost) {cout << setw(40) << "Cost total for lane changing to right: " << cost_lane_change_right << endl;}

      if (cost_lane_change_right < cost_lane_keeping ){return 1;}
      else{return 0;}
    }
}

#endif // ADD_COST_INCLUDED
