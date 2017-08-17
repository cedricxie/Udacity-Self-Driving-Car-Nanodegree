#ifndef ADD_SENSOR_INCLUDED
#define ADD_SENSOR_INCLUDED

using namespace std;

void sensor_processing(vector<vector<double>> sensor_fusion,
  vector<vector<double>> &sensor_car_list_left, vector<vector<double>> &sensor_car_list_mid,
  vector<vector<double>> &sensor_car_list_right, double car_s){
  //***************************************************//
  //Analyze Sensor Information
  //***************************************************//
  int sensor_list_size = sensor_fusion.size();

  //cout << "Total number of cars in sensor list: " << sensor_list_size << endl;
  for (int i=0; i<sensor_list_size; i++){
    //cout << sensor_fusion[i][0] << endl;
    double sensor_id = sensor_fusion[i][0];
    double sensor_vx = sensor_fusion[i][3];
    double sensor_vy = sensor_fusion[i][4];
    double sensor_s = sensor_fusion[i][5];
    double sensor_d = sensor_fusion[i][6];
    double sensor_v = sqrt(sensor_vx*sensor_vx+sensor_vy*sensor_vy);

    if (sensor_d>8.0){
      bool flag = true;
      for(int j=0; j<sensor_car_list_right.size(); j++){
        if(sensor_s>sensor_car_list_right[j][1]){
            sensor_car_list_right.insert(sensor_car_list_right.begin()+j, {sensor_id, sensor_s, sensor_d, sensor_v});
            flag = false;
            break;
        }
      }
      if (flag==true){sensor_car_list_right.push_back({sensor_id, sensor_s, sensor_d, sensor_v});}
    }
    else if(sensor_d>4.0){
      bool flag = true;
      for(int j=0; j<sensor_car_list_mid.size(); j++){
        if(sensor_s>sensor_car_list_mid[j][1]){
            sensor_car_list_mid.insert(sensor_car_list_mid.begin()+j, {sensor_id, sensor_s, sensor_d, sensor_v});
            flag = false;
            break;
        }
      }
      if (flag==true){sensor_car_list_mid.push_back({sensor_id, sensor_s, sensor_d, sensor_v});}
    }
    else{
      bool flag = true;
      for(int j=0; j<sensor_car_list_left.size(); j++){
        if(sensor_s>sensor_car_list_left[j][1]){
            sensor_car_list_left.insert(sensor_car_list_left.begin()+j, {sensor_id, sensor_s, sensor_d, sensor_v});
            flag = false;
            break;
        }
      }
      if (flag==true){sensor_car_list_left.push_back({sensor_id, sensor_s, sensor_d, sensor_v});}
    }
  }

  if(dflag>=dflag_sensor_details)
  {  cout << setw(25) << "Sensor Info for Right Lane" << endl;
    for(int i=0; i<sensor_car_list_right.size(); i++){
      //for(int j=0; j<4; j++){ cout << sensor_car_list_right[i][j] << " ";}
      cout << sensor_car_list_right[i][1] - car_s<< " " <<sensor_car_list_right[i][2]<< " " <<sensor_car_list_right[i][3];
      cout << endl;
    }
    cout << setw(25) << "Sensor Info for Mid Lane" << endl;
    for(int i=0; i<sensor_car_list_mid.size(); i++){
      //for(int j=0; j<4; j++){cout << sensor_car_list_mid[i][j] << " ";}
      cout << sensor_car_list_mid[i][1] - car_s << " " <<sensor_car_list_mid[i][2]<< " " <<sensor_car_list_mid[i][3];
      cout << endl;
    }
    cout << setw(25) << "Sensor Info for Left Lane" << endl;
    for(int i=0; i<sensor_car_list_left.size(); i++){
      //for(int j=0; j<4; j++){ cout << sensor_car_list_left[i][j] << " ";}
      cout << sensor_car_list_left[i][1] - car_s<< " " <<sensor_car_list_left[i][2]<< " " <<sensor_car_list_left[i][3];
      cout << endl;
    }}

}

#endif // ADD_SENSOR_INCLUDED
