#include "kinematics.h"
#include <math.h>

TaskSpace forwardKinematics(JointSpace state) {
    // Initializes a TaskSpace variable called point
    TaskSpace point; 

    // TODO 1: Modify the two lines below to use the forward kinematics equations you derived. 
    // You may need the variables: L1, L2, state.theta1, state.theta2
    // as well as the functions: double cos(double x), double sin(double x), double tan(double x).
    // These variables and functions are already defined, you don't need to define them here.
    point.x = L1*cos(state.theta1) + L2*cos(state.theta1 + state.theta2);
    point.y = L1*sin(state.theta1) + L2*sin(state.theta1 + state.theta2);

    return point;
}

JointSpace inverseKinematics(TaskSpace point) {
    // Initializes a JointSpace variable called state
    JointSpace state; 

    // TODO 2: Modify the two lines below to use the inverse kinematics equations you derived. 
    // You may need the variables: L1, L2, point.x, point.y
    // as well as the functions: double atan2(double x, double y), double pow(double x, 2), double sqrt(double x)
    // These variables and functions are already defined, you don't need to define them here.
    // If there are two possible configurations, you may choose either.
    double x2 = pow(point.x, 2);
    double y2 = pow(point.y, 2);
    double sum = L1 + L2;
    double diff = L1 - L2;
    state.theta2 =  2*atan2(sqrt(pow(sum, 2) - (x2 + y2)), sqrt(x2 + y2 - pow(diff, 2)));
    state.theta1 =  atan2(point.y, point.x) - atan2(L2*sin(state.theta2), L1 + L2*cos(state.theta2));
    
    // Makes sure that theta1 and theta2 are in [-M_PI/2, M_PI/2]
    state.theta1 = atan2(sin(state.theta1), cos(state.theta1));
    state.theta2 = atan2(sin(state.theta2), cos(state.theta2));

    return state;
}