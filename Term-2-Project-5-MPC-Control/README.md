# Model Predictive Control Project

The objective of this project is to implement Model Predictive Control (MPC) to drive the car around the track.

---
## Table of contents

1. MPC Model Overview
2. Preprocessing Steps
3. Description of N and dt
3. Latency Effect

## 1. MPC Model Overview

We will first describe the state, actuator and update equations.

### State [x,y,ψ,v,cte,eψ]
The state vector consists of four components:
1. x: the x-position
2. y: the y-position
3. ψ: the vehicle orientation
4. v: the velocity
5. cte: cross-track error
6. eψ: orientation error

### Actuators [δ, a]
Two actuators are considered in the model, namely:
1. δ: steering angle
2. a: acceleration

### Update Equations
The codes for the evolution of state components are shown here:
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## 2. Preprocessing Steps

### Define N and dt
See below for definitions of N, dt, as well as discussion on parameter tuning.

### Transform waypoints from map coordinates to vehicle coordinates
The transformation is done by the codes as follows.
```
for (int i = 0; i < ptsx.size(); i++) {
  double dtx = ptsx[i] - px;
  double dty = ptsy[i] - py;
  ptsx[i] = dtx * cos(psi) + dty * sin(psi);
  ptsy[i] = dty * cos(psi) - dtx * sin(psi);
}
```

### Define the cost function.
The following weight parameters are added into the cost function.
```
double error_epsi_adj = 500; //orientation error
double error_cte_adj = 1000; // cross-track error
double error_v_adj = 1; // velocity error
double error_delta_adj = 10; //penalize large steering angle
double error_a_adj = 10; // penalize large acceleration
double error_delta_diff_adj = 100; //penalize large steering angle change rate
double error_delta_a_adj = 10; //penalize large acceleration change rate
```
The weight factors contribute to the total cost by multiplying the respective squared deviations.


## 3. Description of N and dt
N is the number of timesteps the model predicts ahead, while dt is the length of each timestep.

### Tuning N and dt

| Trial  | N            | dt               | Observation               |
|:------:|:-------------:|:---------------:|:---------------:| 
| 1      | **10**           | **0.1**            | Car is able to complete the entire lap, but there is sometimes perturbance in the predicted path.             | 
| 2      | *40*           | 0.1            | The track predicted is more smooth, but computation takes more time.            | 
| 3      | *4*          | 0.1            | The car went off the road due to insufficient information in prediction.            | 
| 4      | 10           | *0.2*            | The car is not able to respond to the curvature in the road in time and went off the road.             | 
| 5      | 10           | *0.01*            | The total predicted time is too small therefore the car oscillates around the track heavily.            | 

As a trade-off between performance and efficiency, N=10 and dt=0.1 is the final choice. More complete evaluation of the effect of N, dt, such as using **Twiddle**, is planned to be tried once time permits.

## 4. Latency Effect
The latency of **100ms** is added into the model so that our kinematic model would be able to predict its status taking into the delay in response of the actuators into account.

```
  Eigen::VectorXd state(6);            
  double dt = 0.1;      
  // assuming x = y = psi = 0
  double predicted_x = v * dt;
  double predicted_y = 0;
  double predicted_psi = - v * delta / Lf * dt;
  double predicted_v = v + a * dt;
  double predicted_cte = cte + v * sin(epsi) * dt;
  double predicted_epsi = epsi - v * delta / Lf * dt;
  state << predicted_x, predicted_y, predicted_psi, predicted_v, predicted_cte, predicted_epsi;
```
