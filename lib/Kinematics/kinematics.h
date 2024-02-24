#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
# define L_HOLE 2.4 // (cm) distance between large holes
# define L1 L_HOLE*7 // cm
# define L2 L_HOLE*9 // cm

#define THETA1_OFFSET M_PI/2.0

struct JointSpace {
    double theta1; // radians
    double theta2; // radians
};

struct TaskSpace {
    double x; // cm 
    double y; // cm
};

TaskSpace forwardKinematics(JointSpace state);
JointSpace inverseKinematics(TaskSpace point);

#endif