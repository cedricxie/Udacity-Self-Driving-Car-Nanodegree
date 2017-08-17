#ifndef ADD_MAP_INCLUDED
#define ADD_MAP_INCLUDED

using namespace std;

//***************************************************//
//User Defined Parameters Definition
//***************************************************//
int path_interpolate = 30;

void map_read_in(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
  vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy){

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
      istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      map_waypoints_x.push_back(x);
      map_waypoints_y.push_back(y);
      map_waypoints_s.push_back(s);
      map_waypoints_dx.push_back(d_x);
      map_waypoints_dy.push_back(d_y);
    }
}

void sortXY( vector< double> &X_tmp, vector< double> &Y_tmp, int flag)
{
  vector<double> results[2];
  if (flag == 1){
    for (int i = 0; i<4; i++){
      double smallest_X = X_tmp[i];
      int smallest_idx = i;
      for (int j = i+1; j<4; j++){
        if (X_tmp[j] < smallest_X){
          smallest_X = X_tmp[j];
          smallest_idx = j;
        }
      }
      if(smallest_idx!=i){
        double tmp = X_tmp[i];
        X_tmp[i] = X_tmp[smallest_idx];
        X_tmp[smallest_idx] = tmp;
        tmp = Y_tmp[i];
        Y_tmp[i] = Y_tmp[smallest_idx];
        Y_tmp[smallest_idx] = tmp;
      }
    }
  }
  else{
    for (int i = 0; i<4; i++){
      double smallest_Y = Y_tmp[i];
      int smallest_idx = i;
      for (int j = i+1; j<4; j++){
        if (Y_tmp[j] < smallest_Y){
          smallest_Y = Y_tmp[j];
          smallest_idx = j;
        }
      }
      if(smallest_idx!=i){
        double tmp = X_tmp[i];
        X_tmp[i] = X_tmp[smallest_idx];
        X_tmp[smallest_idx] = tmp;
        tmp = Y_tmp[i];
        Y_tmp[i] = Y_tmp[smallest_idx];
        Y_tmp[smallest_idx] = tmp;
      }
    }
  }
}

void map_refine(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
  vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy,
  vector<double> &map_waypoints_x_tmp, vector<double> map_waypoints_y_tmp, vector<double> map_waypoints_s_tmp,
  vector<double> map_waypoints_dx_tmp, vector<double> map_waypoints_dy_tmp){

    double max_s = 6945.554;

    vector<double> X_tmp(4), Y_tmp(4);
    int i1, i2, i3, i4;
    tk::spline s_tmp;

    for (int i =0; i<map_waypoints_x.size();  i++){
      i2 = i;
      i1 = i2 - 1;
      if (i1<0){i1 += map_waypoints_x.size();}
      i3 = i2 + 1;
      if (i3 >= map_waypoints_x.size()){ i3 -= map_waypoints_x.size();}
      i4 = i2 + 2;
      if (i4 >= map_waypoints_x.size()){ i4 -= map_waypoints_x.size();}
      X_tmp[0] = map_waypoints_x[i1];
      X_tmp[1] = map_waypoints_x[i2];
      X_tmp[2] = map_waypoints_x[i3];
      X_tmp[3] = map_waypoints_x[i4];
      Y_tmp[0] = map_waypoints_y[i1];
      Y_tmp[1] = map_waypoints_y[i2];
      Y_tmp[2] = map_waypoints_y[i3];
      Y_tmp[3] = map_waypoints_y[i4];

      if (abs(map_waypoints_x[i2]-map_waypoints_x[i3])/map_waypoints_x[i2] > 0.005){
        sortXY(X_tmp, Y_tmp, 1);
        //cout << setw(25) << "sorted X:  "<< X_tmp[0] << " " << X_tmp[1] << " " << X_tmp[2] << " "<< X_tmp[3] << " " << endl;
        //cout << setw(25) << "sorted Y:  "<< Y_tmp[0] << " " << Y_tmp[1] << " " << Y_tmp[2] << " "<< Y_tmp[3] << " " << endl;
        s_tmp.set_points(X_tmp, Y_tmp);
        for(int i = 0; i<path_interpolate; i++){
          double x = map_waypoints_x[i2] + (map_waypoints_x[i3] - map_waypoints_x[i2])/path_interpolate*i;
          //vector<double> fitting_results = s_tmp(x);
          //double y = fitting_results[0];
          double y = s_tmp(x);
          map_waypoints_x_tmp.push_back(x);
          map_waypoints_y_tmp.push_back(y);
          double s;
          if (map_waypoints_s[i2] < 6900.0){
            s = (map_waypoints_s[i2]*(path_interpolate-i) + map_waypoints_s[i3]*i)/path_interpolate;
          }
          else{
            s = (map_waypoints_s[i2]*(path_interpolate-i) + max_s*i)/path_interpolate;
          }
          map_waypoints_s_tmp.push_back(s);
          //double slope = fitting_results[1];
          //double direct = atan(slope);
          //double direct_perp = atan(slope) - pi()/2;
          double d_x_tmp = (map_waypoints_dx[i2]*(path_interpolate-i) + map_waypoints_dx[i3]*i)/path_interpolate;
          double d_y_tmp = (map_waypoints_dy[i2]*(path_interpolate-i) + map_waypoints_dy[i3]*i)/path_interpolate;
          map_waypoints_dx_tmp.push_back(d_x_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          map_waypoints_dy_tmp.push_back(d_y_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          //if (dflag >= dflag_fitting) {cout << setw(25) << "fitting:  "<< x << " " << y << " " << s << " " << d_x_tmp << " " << d_y_tmp << endl;}
        }
      }
      else{
        sortXY(X_tmp, Y_tmp, 2);
        //cout << setw(25) << "sorted X:  "<< X_tmp[0] << " " << X_tmp[1] << " " << X_tmp[2] << " "<< X_tmp[3] << " " << endl;
        //cout << setw(25) << "sorted Y:  "<< Y_tmp[0] << " " << Y_tmp[1] << " " << Y_tmp[2] << " "<< Y_tmp[3] << " " << endl;
        s_tmp.set_points(Y_tmp, X_tmp);
        for(int i = 0; i<path_interpolate; i++){
          double y = map_waypoints_y[i2] + (map_waypoints_y[i3] - map_waypoints_y[i2])/path_interpolate*i;
          //vector<double> fitting_results = s_tmp(y);
          //double x = fitting_results[0];
          double x = s_tmp(y);
          map_waypoints_x_tmp.push_back(x);
          map_waypoints_y_tmp.push_back(y);
          double s;
          if (map_waypoints_s[i2] < 6900.0){
            s = (map_waypoints_s[i2]*(path_interpolate-i) + map_waypoints_s[i3]*i)/path_interpolate;
          }
          else{
            s = (map_waypoints_s[i2]*(path_interpolate-i) + max_s*i)/path_interpolate;
          }
          map_waypoints_s_tmp.push_back(s);
          //double slope = fitting_results[1];
          //double direct = atan(slope);
          //double direct_perp = atan(slope) - pi()/2;
          double d_x_tmp = (map_waypoints_dx[i2]*(path_interpolate-i) + map_waypoints_dx[i3]*i)/path_interpolate;
          double d_y_tmp = (map_waypoints_dy[i2]*(path_interpolate-i) + map_waypoints_dy[i3]*i)/path_interpolate;
          map_waypoints_dx_tmp.push_back(d_x_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          map_waypoints_dy_tmp.push_back(d_y_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          //if (dflag >= dflag_fitting) {cout << setw(25) << "fitting:  "<< x << " " << y << " " << s << " " << d_x_tmp << " " << d_y_tmp << endl;}
        }
      }
    }
    //if (dflag >= dflag_fitting) {cout << setw(25) << "fitting summary:  "<< map_waypoints_x.size() << " " << map_waypoints_x[0] << " " << map_waypoints_y[0] << " " << map_waypoints_s[0] << endl;}
}

void spline_fitting(tk::spline &s_x, tk::spline &s_y, tk::spline &s_dx, tk::spline &s_dy,
vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy){

  s_x.set_points(maps_s, maps_x);
  s_y.set_points(maps_s, maps_y);
  s_dx.set_points(maps_s, maps_dx);
  s_dy.set_points(maps_s, maps_dy);

}

void prev_list_read_in(){
  /*
  if(path_size == 0)
  {
      prev_x = car_x;
      prev_y = car_y;
      prev_yaw_rad = deg2rad(car_yaw);
      prev_s = car_s;
      prev_d = car_d;
      prev_speed = car_speed;
  }
  else if (path_size == 1)
  {
      prev_x = previous_path_x[path_size-1];
      prev_y = previous_path_y[path_size-1];
      prev_yaw_rad = atan2(prev_y-car_y,prev_x-car_x);
      tmp_status = getFrenet(prev_x, prev_y, prev_yaw_rad, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      prev_s = tmp_status[0];
      prev_d = tmp_status[1];
      prev_speed = (prev_s - car_s)/(t_inc/t_n);
  }
  else
  {
      prev_x = previous_path_x[path_size-1];
      prev_y = previous_path_y[path_size-1];
      double prev_x2 = previous_path_x[path_size-2];
      double prev_y2 = previous_path_y[path_size-2];
      double prev_x3 = previous_path_x[path_size-3];
      double prev_y3 = previous_path_y[path_size-3];
      prev_yaw_rad = atan2(prev_y-prev_y2,prev_x-prev_x2);
      tmp_status = getFrenet(prev_x, prev_y, prev_yaw_rad, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      prev_s = tmp_status[0];
      double prev_yaw_rad2 = atan2(prev_y2-prev_y3,prev_x2-prev_x3);
      tmp_status = getFrenet(prev_x2, prev_y2, prev_yaw_rad2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      double prev_s2 = tmp_status[0];
      prev_speed = (prev_s - prev_s2)/(t_inc/t_n);
      cout << setw(25) << "==================================================================" << endl;
      //cout << setw(25) << "prev_s:  "<< prev_s << " " << "prev_s2:  "<< prev_s2 << endl;
      //cout << setw(25) << "prev_yaw:  "<< prev_yaw_rad << " " << "prev_yaw2:  "<< prev_yaw_rad2 << endl;
      cout << setw(25) << "all path points in the prev list: " << endl;
      for (int i = 0; i<path_size; i++) cout << previous_path_x[i] << ' ' << previous_path_y[i] << ' ' << endl;
  }*/
}

#endif // ADD_MAP_INCLUDED
