/* 
 * File:   estimator.h
 * Author: Dmitriy
 *
 * Contains position estimator and current estimator. Make sure you initialize
 * the various modules used for estimation (such as encoder, ADC, etc. before
 * initializing the estimators).
 *
 * Created on February 3, 2015, 12:10 PM
 */

#ifndef ESTIMATOR_H
#define	ESTIMATOR_H

#define HALL_ESTIMATOR

//#define ENCODER_POSITION
//#define ENCODER_VELOCITY
//#define ENCODER_ACCELERATION

//#define TEST_ENCODER_OFFSET
//#define ESTIMATOR_TEST

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



#endif	/* ESTIMATOR_H */

