#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

# define L1 16.9 // cm
# define L2 20.9 // cm

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