/* 
 * File:   SVM_Torque_Control.h
 * Author: Dmitriy
 *
 * Created on February 10, 2015, 4:00 PM
 */

#ifndef SVM_TORQUE_CONTROL_H
#define	SVM_TORQUE_CONTROL_H

//initializes some constants
void initSVM(void);

//reads position, controls  acceleration, updates duty cycles. torque is in mNm.
void updateTorqueController (float torque);
#endif	/* SVM_TORQUE_CONTROL_H */

