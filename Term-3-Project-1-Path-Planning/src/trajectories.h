#ifndef ADD_TRAJ_INCLUDED
#define ADD_TRAJ_INCLUDED

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS
    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE
    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

    return result;
}

double trj_evl(vector< double> coeffs, double T)
{
    return coeffs[0]+coeffs[1]*T+coeffs[2]*T*T+coeffs[3]*T*T*T+coeffs[4]*T*T*T*T+coeffs[5]*T*T*T*T*T;
}

double s_evl(double car_s_current, double t_current, double car_speed_max, double c, double t)
{
    return car_s_current + car_speed_max/c * log((exp(c*(t_current+t))+1)/(exp(c*t_current)+1));
}

void trajectories_generation(vector<double> &start_states, vector<double> &end_states, vector<double> &s_history,
  double v_init, double v_end,
  double prev_d, vector<double> &d_history,
  double d_init, double d_end,
  int states_size, int t_inc, int t_n, double car_speed_max, double c, double c_d){

    double car_s_next;
    double car_speed_next;
    double car_a_next;

    double car_d_next;
    double car_d_v_next;
    double car_d_a_next;

    double t_current;
    double t_next;

    double car_s_current = start_states[0];
    double car_speed_current = start_states[1];
    double car_a_current = start_states[2];

    double car_d_current;
    double car_d_v_current;
    double car_d_a_current;

    if ( (car_speed_current-v_init)/(v_end - v_init) < 1.0/(1+exp(c*3.0)) ) { t_current = -3.0;}
    else if( (car_speed_current-v_init)/(v_end - v_init) > 1.0/(1+exp(-c*3.0)) ) { t_current = 3.0;}
    else{ t_current = - log((v_end - v_init)/(car_speed_current-v_init) - 1.0)/c;}

    //car_s_current = prev_s + car_speed_max/c*log((exp(c*(t_current+t_inc/t_n))+1)/(exp(c*t_current)+1));
    //car_speed_current = car_speed_max * 1.0/(1.0+exp(-c*(t_current+t_inc/t_n)));
    //car_a_current = car_speed_max*c*exp(-c*(t_current+t_inc/t_n))/(1.0+exp(-c*(t_current+t_inc/t_n)))/(1.0+exp(-c*(t_current+t_inc/t_n)));

    car_s_next = car_s_current + v_init*double(t_inc)/t_n*(t_n+1.0) + (v_end - v_init)/c * log((exp(c*(t_current+double(t_inc)/t_n*(t_n+1.0)))+1.0)/(exp(c*t_current)+1.0));
    car_speed_next = v_init + (v_end-v_init)*1.0/(1.0+exp( -c*(t_current+double(t_inc)/t_n*(t_n+1.0) ) ) );
    car_a_next = (v_end-v_init)*c*exp(-c*(t_current+double(t_inc)/t_n*(t_n+1.0)))/(1.0+exp(-c*(t_current+double(t_inc)/t_n*(t_n+1.0))))/(1.0+exp(-c*(t_current+double(t_inc)/t_n*(t_n+1.0))));

    end_states={car_s_next, car_speed_next, car_a_next};
    //cout << t_current << " " << car_speed_current << " " << v_init << " " << v_end << " " << car_speed_next << endl;

    if (abs(prev_d - d_end)<0.01){ t_current = 3.0; } //in case when prev_d and d_end is very close, to avoid dvide by zero
    else if ( (prev_d-d_init)/(d_end - d_init) < 1.0/(1+exp(c_d*3.0)) ) { t_current = -3.0;}
    else if( (prev_d-d_init)/(d_end - d_init) > 1.0/(1+exp(-c_d*3.0)) ) { t_current = 3.0;}
    else{ t_current = - log((d_end - d_init)/(prev_d-d_init) - 1.0)/c_d;}

    t_next = t_current + double(t_inc)/t_n*(t_n+1.0);

    car_d_current = prev_d;
    car_d_v_current = (d_end-d_init)* c_d*exp(-c_d*t_current) /(1.0+exp(-c_d*t_current))/(1.0+exp(-c_d*t_current));
    car_d_a_current = (d_end-d_init)* c_d*c_d*exp(-c_d*t_current)*(exp(-c_d*t_current)-1) /(1.0+exp(-c_d*t_current))/(1.0+exp(-c_d*t_current))/(1.0+exp(-c_d*t_current));

    car_d_next = d_init + (d_end-d_init) /(1.0+exp(-c_d*t_next));
    car_d_v_next= (d_end-d_init)* c_d*exp(-c_d*t_next) /(1.0+exp(-c_d*t_next))/(1.0+exp(-c_d*t_next));
    car_d_a_next= (d_end-d_init)* c_d*c_d*exp(-c_d*t_next)*(exp(-c_d*t_next)-1) /(1.0+exp(-c_d*t_next))/(1.0+exp(-c_d*t_next))/(1.0+exp(-c_d*t_next));

    //cout << setw(25) << "==================================================================" << endl;
    //cout << setw(25) << "tmp output:  "<< car_speed_max << " " << (exp(c*(t_current+double(t_inc)/t_n*(t_n+1.0)))+1.0) << " " << (exp(c*t_current)+1.0) << " " << t_current << " " << t_inc << " " << t_n << endl;
    //cout << setw(25) << "start status:  "<< start_states[0] << " " << start_states[1] << " " << start_states[2] << endl;
    //cout << setw(25) << "end status: "<< end_states[0] << " " << end_states[1] << " " << end_states[2] << endl;
    //cout << setw(25) << "==================================================================" << endl;

    vector <double> trajectory_coeffs = JMT(start_states, end_states, t_inc);
    vector <double> trajectory_coeffs_d = JMT({car_d_current, car_d_v_current, car_d_a_current}, {car_d_next, car_d_v_next, car_d_a_next}, t_inc);
    //cout << setw(25) << "JML Coefficients: ";
    //for (auto const& k : trajectory_coeffs) std::cout << k << ' ';
    //cout << endl;

    for(int i = 1; i <= states_size; i++)
    {
      //tmp_status = getXY(trj_evl(trajectory_coeffs, t_inc/t_n*i), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      //tmp_status = getXY(trj_evl(trajectory_coeffs, t_inc/t_n*i), car_d, map_waypoints_s_tmp, map_waypoints_x_tmp, map_waypoints_y_tmp);
      //tmp_status = getXY_spline(trj_evl(trajectory_coeffs, t_inc/t_n*i), car_d, s_x, s_y, s_dx, s_dy);
      //cout << setw(25) << "pushed in s:  "<< trj_evl(trajectory_coeffs, t_inc/t_n*i) << " " << tmp_status[0] << " " << tmp_status[1] << endl;
      //tmp_status = getXY(s_evl(car_s_current, t_current, car_speed_max, c, t_inc/t_n*i), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      //cout << setw(25) << "pushed in s:  "<< s_evl(car_s_current, t_current, car_speed_max, c, t_inc/t_n*i) << endl;
      //cout << setw(25) << "pushed in s:  "<< trj_evl(trajectory_coeffs, double(t_inc)/t_n*i) << endl;
      s_history.push_back(trj_evl(trajectory_coeffs, double(t_inc)/t_n*i));
      //d_history.push_back(prev_d);
      d_history.push_back(trj_evl(trajectory_coeffs_d, double(t_inc)/t_n*i));
      //next_x_vals.push_back(tmp_status[0]);
      //next_y_vals.push_back(tmp_status[1]);
    }
    //cout << setw(25) << "last point in s:  "<< s_history[s_history.size()-1] << endl;
}

#endif // ADD_TRAJ_INCLUDED
