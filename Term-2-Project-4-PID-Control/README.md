# Reflection

## Design Matrix


| Trial  | P             | I               | D               |
|:------:|:-------------:|:---------------:|:---------------:| 
| 1      | **0.1**           | **0.01**            | **3.0**             | 
| 2      | *1.0*           | 0.01            | 3.0             | 
| 3      | *0.01*          | 0.01            | 3.0             | 
| 4      | 0.1           | *0.2*            | 3.0             | 
| 5      | 0.1           | *0.0001*            | 3.0             | 
| 6      | 0.1           | 0.01            | *5.0*             | 
| 7      | 0.1           | 0.01            | *1.0*             | 

## Effect of P Component: 

1. Helps the car to steer back into lane in proportion to the crosstrack error
2. Might cause overshoot and/or oscillation
3. Three different values of P coefficient are studied:
* P = 1.0: constantly overshooting and not able to stay in the lane.
* P = 0.01: too slow adjustment to the error
* P = 0.1: final choice

## Effect of I Component

1. Helps the car to steer back to correct position even when there is a bias
2. Three different values of I coefficient are studied:
* I = 0.2: too much steering due to bias
* I = 0.0001: too little adjustment for bias
* I = 0.005: final choice

## Effect of D Component: 

1. Helps the car to reduce overshooting and/or oscillation
2. Three different values of D coefficient are studied:
* D = 5.0: too much counter-steering
* D = 1.0: too little counter-steering 
* D = 3.0: final choice

## Choices of Hyperparameters 

The final hyperparameters for P, I and D as (0.1, 0.005, 3.0) were chosen manually.

