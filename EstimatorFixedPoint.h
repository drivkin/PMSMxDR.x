/* 
 * File:   EstimatorFixedPoint.h
 * Author: Dmitriy
 *
 * Created on February 18, 2015, 5:13 PM
 */

#ifndef ESTIMATORFIXEDPOINT_H
#define	ESTIMATORFIXEDPOINT_H

#define HALL_ESTIMATOR

//#define ENCODER_POSITION
//#define ENCODER_VELOCITY
//#define ENCODER_ACCELERATION

//#define TEST_ENCODER_OFFSET
//#define ESTIMATOR_TEST


#define BLOCK 0
#define SVM 1

typedef struct{
    float iAlpha;
    float iBeta;
} ABCurrents;



// returns mechanical rotor angle
float getRotorAngleMechanical(void);

//get rotor angle in electrical degrees. Currently, the motor has 2 pole pairs,
// ie theta_elec = 2*theta_mech
float getRotorAngleElectrical(void);

// Returns the rotor velocity in radians/second mechanical.
float getRotorSpeed(void);

//returns rotor acceleration in rad/s^2 mechanical
float getRotorAcceleration(void);

//returns a struct which contains the alpha and beta (stator frame) components of
// current
ABCurrents getABCurrents(void);

// 1 if the estimator is tracking the position
int isTracking(void);


//Initializes position estimator and current estimator
void initEstimators(void);

void updateHalls(void);

int getDriveStatus(void);

void setDriveStatus(int status);



#endif	/* ESTIMATORFIXEDPOINT_H */

