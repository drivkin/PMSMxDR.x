/* fixed point estimator library. In order to not lose too much to quantization
 *  error
 * we have to do some scaling, so (_t indicates true value):
 * TClk = Tclk_t*2^22;
 * ClkCount = ClkCount_t*2^-16;
 * t = t_t * 2^6;
 */


#include "EstimatorFixedPoint.h"
#include <stdint.h>
#include <xc.h>
#include <math.h>
#include "PMSMBoard.h"
#include <libq.h>
#include <stdlib.h>
#include <stdfix.h>
#include "SVM_Torque_Control.h"




#define T_EST 8562 //  2^13 Hz
//#define T_EST_S .001
#define ROT_THRESH .005
#define PI 3.14159
#define T_CLK 0.05981077504 // 1.426*10^-8 * 2^22


#define MINUS_PI -3.14159
#define TWO_PI 6.28318530718

#define ALPHA_MAX 50

#define FILT_SIZET 10

#define CCW 0
#define CW 1

int guard = 0;

int tracking = 0; // keeps track of whether the estimator is doing a decent job
_Q16 trackThresh; // allowable error magnitude on predicted time of next hall event
_Q16 predictedDeltaTheta = 0;

float encoderZero = 0;

int pHA = 0;
int pHB = 0;
int pHC = 0;

int direction = 0;

int driveStatus = 0;

_Q16 rotorAngleElectrical = 0;
_Q16 rotorAngleMechanical = 0;
_Q16 rotorSpeed = 0;
_Q16 rotorAcceleration = 0;

float c1 = 0.00009643569; // constant to convert from adjusted encoder DC to 0-2pi

// for velocity and acceleration estimators

float tNextData = 0;
int newDataFlag = 0;

ABCurrents iAB;

_Q16 MAVfiltT(_Q16 dat);

uint32_t getEncoderOffset(void);
void computeRotorParameters(void);
float MAVfiltP(float dat);
float MAVfiltV(float dat);
float computeRotorAngleMechanical(void);
float computeRotorSpeed(void);
float computeRotorAcceleration(void);
void TrapUpdate(uint16_t torque, uint16_t direction);
void differentialEstimate(void);
void feedbackEstimate(_Q16 e);
ABCurrents computeABCurrents(void);

int getDriveStatus(void) {
    return driveStatus;
}

void setDriveStatus(int status) {
    driveStatus = status;
}

float getRotorAngleMechanical(void) {
    return _itofQ16(rotorAngleMechanical);
}

float getRotorAngleElectrical(void) {
    return _itofQ16(rotorAngleElectrical);
}

float getRotorSpeed(void) {
    return _itofQ16(rotorSpeed);
}

float getRotorAcceleration(void) {
    return _itofQ16(rotorAcceleration);
}

int isTracking() {
    return tracking;
}


_Q16 filterSize;
_Q16 PIover3;
_Q16 twoPIover3;
_Q16 threePIover3;
_Q16 fourPIover3;
_Q16 fivePIover3;
_Q16 sixPIover3;

void initConstants(void) {
    trackThresh = _Q16ftoi(1.5);
    filterSize = _Q16ftoi(FILT_SIZET);
    PIover3 = _Q16ftoi(1.0471975512);
    twoPIover3 = _Q16ftoi(2.09439510239);
    threePIover3 = _Q16ftoi(3.14159265359);
    fourPIover3 = _Q16ftoi(4.18879020479);
    fivePIover3 = _Q16ftoi(5.23598775598);
    sixPIover3 = _Q16ftoi(6.28318530718);
}

void initEstimators(void) {
    initConstants();
    //for testing
    //    float hi = 0.00001460224 * 1287;
    //    float hello = T_CLK * 1287;
    //estimators use timer nine
    T9CONbits.TON = 0;
    T9CONbits.TCS = 0;
    T9CONbits.TGATE = 0;
    T9CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR9 = 0x0000;
    PR9 = T_EST;
    IFS3bits.T9IF = 0;
    IPC13bits.T9IP = 0x02;
    IEC3bits.T9IE = 1;
    T9CONbits.TON = 1;
    //T9CONbits.TON = 0; // FOR TEST

#ifdef HALL_ESTIMATOR
    pHA = HALL1;
    pHB = HALL2;
    pHC = HALL3;

    //setup 32 bit timer with 4&5
    T5CONbits.TON = 0;
    T4CONbits.TON = 0;
    T4CONbits.T32 = 1;
    T4CONbits.TCS = 0;
    T4CONbits.TGATE = 0;
    T4CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    T5CONbits.TCKPS = 0b00;
    TMR5 = 0x0000;
    TMR4 = 0X0000;
    PR5 = 0xFFFF;
    PR4 = 0xffff;
    IPC7bits.T5IP = 0x02;
    IFS1bits.T5IF = 0;
    IEC1bits.T5IE = 0;
    T4CONbits.TON = 1;

#endif

    // c1 = 2 * 3.14159265359 / 65154.144; // constant to convert from adjusted encoder DC to 0-2pi

    //encoderZero = ((float) getEncoderOffset() - 255) * c1; // the 255 is so that a zero offset makes encoderZero = 0
    iAB.iAlpha = 0;
    iAB.iBeta = 0;
}

//make sure the drv is initialized already when this is called.
//Current flows in through phase A and out through the other two. This causes
// the rotor to align to theta_elec = 0. Read the encoder value at this position
// to calculate encoder offset

uint32_t getEncoderOffset(void) {

    GH_A_DC = 500;
    GH_B_DC = 0;
    GH_C_DC = 0;
    uint32_t i;
    LED3 = 1;
    //wait a bit
    for (i = 0; i < 70000000; i++);
    LED3 = 0;
    return getEncDC();
}

//
int printPre = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T9Interrupt(void) {
    computeRotorParameters();
    //        if (isTracking()) {
    //            if (driveStatus == BLOCK) {
    //                initSVMCom();
    //                driveStatus = SVM;
    //            }
    updateTorqueController(0.5);
    IFS3bits.T9IF = 0; //Clear Timer3 interrupt flag
    //        }
    //#ifdef ESTIMATOR_TEST
    //            if (printPre < 4) {
    //                printPre++;
    //            } else {
    //            _Q16 first = _Q16ftoi(50);
    //            _Q16 second = _Q16ftoi(-50);
    //            _Q16 third = _Q16ftoi(49.43);
    //                static float out[3];
    //                out[0] = _itofQ16(first);
    //                out[1] = _itofQ16(second);
    //                out[2] = _itofQ16(_Q16div(third,first));
    //                PrintWithTimestamp(out, 3);
    //                printPre = 0;
    //            }

    // #endif
}




_Q16 MAVfiltT(_Q16 dat) {

    static _Q16 data[FILT_SIZET];
    static int first = 1;
    static int index = 0;
    static _Q16 avg = 1;
    if (first) {
        int i;
        _Q16 a = _Q16div(dat, filterSize);
        for (i = 0; i < FILT_SIZET; i++) {
            data[i] = a;
        }
        avg = dat;
        first = 0;
    }
    dat = _Q16div(dat, filterSize);
    avg = avg - data[index] + dat;
    data[index] = dat;
    index++;
    if (index == FILT_SIZET) {
        index = 0;
    }
    return avg;
}

//#define FILT_SIZEV 20
//
//float MAVfiltV(float dat) {
//
//    static float data[FILT_SIZEV];
//    static int first = 1;
//    static int index = 0;
//    static float avg = 0;
//    if (first) {
//        int i;
//        for (i = 0; i < FILT_SIZEV; i++) {
//            data[i] = 0;
//        }
//        first = 0;
//    }
//    dat = dat / FILT_SIZEV;
//    avg = avg - data[index] + dat;
//    data[index] = dat;
//    index++;
//    if (index == FILT_SIZEV) {
//        index = 0;
//    }
//    return avg;
//}
//
//#define FILT_SIZEA 20
//
//float MAVfiltA(float dat) {
//
//    static float data[FILT_SIZEA];
//    static int first = 1;
//    static int index = 0;
//    static float avg = 0;
//    if (first) {
//        int i;
//        for (i = 0; i < FILT_SIZEA; i++) {
//            data[i] = 0;
//        }
//        first = 0;
//    }
//    dat = dat / FILT_SIZEA;
//    avg = avg - data[index] + dat;
//    data[index] = dat;
//    index++;
//    if (index == FILT_SIZEA) {
//        index = 0;
//    }
//    return avg;
//}

void differentialEstimate(void) {

    static _Q16 prevV = 0;
    static int first = 1;
    static _Q16 bigThreshPos = 0;
    static _Q16 smallThreshPos = 0;
    static _Q16 bigThreshNeg = 0;
    static _Q16 smallThreshNeg = 0;
    _Q16 filteredTime = 0;
    static _Q16 realTime = 1;

    uint16_t lsw = TMR4;
    uint16_t msw = TMR5HLD;
    int32_t time = msw;
    time = time << 16;
    time = time | lsw;

    TMR5HLD = 0x0000;
    TMR4 = 0x0000;

    _Q16 t = time;


    if (first) {
        bigThreshPos = _Q16mpy(_Q16ftoi(1.0471975512), trackThresh);
        smallThreshPos = _Q16div(_Q16ftoi(1.0471975512), trackThresh);
        bigThreshNeg = _Q16mpy(_Q16ftoi(-1.0471975512), trackThresh);
        smallThreshNeg = _Q16div(_Q16ftoi(-1.0471975512), trackThresh);

        first = 0;
    } else {
        //timeInFixed = _Q16ftoi((float) time);
        realTime = _Q16mpy(_Q16ftoi(T_CLK), t);
        filteredTime = MAVfiltT(realTime);
        if (direction == CCW) {
            rotorSpeed = _Q16div(PIover3 << 6, filteredTime); // shift by 6 to account for time being too large
            if (smallThreshPos < predictedDeltaTheta && predictedDeltaTheta < bigThreshPos) {
                tracking = 1;
            } else {
                tracking = 0;
            }

        } else {
            rotorSpeed = _Q16div(_Q16neg(PIover3) << 6, filteredTime); // shift by 6 to account for time being too large
            if (bigThreshNeg < predictedDeltaTheta && predictedDeltaTheta < smallThreshNeg) {
                tracking = 1;
            } else {
                tracking = 0;
            }
        }

    }
    rotorAcceleration = _Q16div((rotorSpeed - prevV) << 6, filteredTime); // shift by 6 to account for time being too large

    //        rotorSpeed = realTime;
    //         rotorAcceleration = timeInFloat;
    prevV = rotorSpeed;
}

void feedbackEstimate(_Q16 e) {
    static _Q16 integral = 0;
    static _Q16 prevE = 0;
    static _Q16 kp;
    static _Q16 ki;
    static _Q16 kd;
    static int first1 = 1;
    if (first1) {
        kp = _Q16ftoi(100);
        ki = _Q16ftoi(50);
        kd = _Q16ftoi(50);
        first1 = 0;
    }
    integral = integral + e;
    rotorSpeed = _Q16mpy(kp, e) + _Q16mpy(ki, integral) - _Q16mpy(kd, e - prevE);
    prevE = e;
}

#ifdef HALL_ESTIMATOR

void hallUpdate() {
    if (HALL1 != pHA || HALL2 != pHB || HALL3 != pHC) {
        uint16_t lsw = TMR4;
        uint16_t msw = TMR5HLD;
        int32_t time = msw;
        time = time << 16;
        time = time | lsw;
        static int first =1;
        guard = 1;

        //        if (hallCount < 11) {
        //            hallCount++;
        //            if (predictedDeltaTheta > residual) {
        //                residual = predictedDeltaTheta;
        //            }
        //            predictedDeltaTheta = 0;
        //        } else {
        //            TMR5HLD = 0x0000;
        //            TMR4 = 0x0000;
        //            hallCount = 0;
        //            if (predictedDeltaTheta > residual) {
        //                residual = predictedDeltaTheta;
        //            }
        //            predictedDeltaTheta = 0;


        //            if (direction == CCW) {
        //                residual = _Q16ftoi(1.0471975512) - predictedDeltaTheta;
        //            } else {
        //                residual = _Q16ftoi(-1.0471975512) - predictedDeltaTheta;
        //            }
        //            feedbackEstimate(residual);
        //            //residual = 0;
        //            predictedDeltaTheta = 0;
        //        }




        static _Q16 ang;

        if (HALL1) {
            LED1 = 1;
        } else {
            LED1 = 0;
        }
        if (HALL2) {
            LED2 = 1;
        } else {
            LED2 = 0;
        }

        if (HALL3) {
            LED3 = 1;
        } else {
            LED3 = 0;
        }

        //works
        if (HALL1 != pHA) {
            if (HALL1) {
                if (HALL3) {
                    direction = CCW;
                    ang = 0;
                    //                ang = 4.71238898038;
                } else {
                    direction = CW;
                    ang = threePIover3; //3 * 2pi/6
                    //                ang = 1.57079632679;
                }
            } else {
                if (HALL3) {
                    direction = CW;
                    ang = 0;
                    //                ang = 4.71238898038;
                } else {
                    direction = CCW;
                    ang = threePIover3; //3 * 2pi/6
                    //                ang = 1.57079632679;

                }
            }
        } else if (HALL2 != pHB) {
            if (HALL2) {
                if (HALL1) {
                    direction = CCW;
                    ang = twoPIover3; // 2 * 2pi/6
                    //                ang = 0.52359877559;
                } else {
                    direction = CW;
                    ang = fivePIover3; // 5*2pi/6
                    //                ang = 3.66519142919;
                }
            } else {
                if (HALL1) {
                    direction = CW;
                    ang = twoPIover3; // 2 * 2pi/6
                    //                ang = 0.52359877559;
                } else {
                    direction = CCW;
                    ang = fivePIover3; // 5*2pi/6
                    //                ang = 3.66519142919;
                }
            }
        } else if (HALL3 != pHC) {
            if (HALL3) {
                if (HALL2) {
                    direction = CCW;
                    ang = fourPIover3; //4*2pi/6
                    //                ang = 2.61799387799;

                } else {
                    direction = CW;
                    ang = PIover3; //2PI/6
                    //                ang = 5.75958653158;
                }
            } else {
                if (HALL2) {
                    direction = CW;
                    ang = fourPIover3; //4*2pi/6
                    //                ang = 2.61799387799;
                } else {
                    direction = CCW;
                    ang = PIover3; //2PI/6
                    //                ang = 5.75958653158;
                }
            }
        }

        static int d1, d2;
        static int prev;
        if(first){
            d1 = direction;
            d2 = direction;
            prev = direction;
            first = 0;
        }else{
            if((d1 && d2 && direction)||(!d1 && !d2 && !direction)){ // if the three are consistent
                prev = direction;
            }
            d2 = d1;
            d1 = direction;
            direction = prev;
        }


        rotorAngleElectrical = ang;
        differentialEstimate();
        //        int hallState = 0;
        //        hallState = hallState | HALL1;
        //        hallState = hallState | HALL2 << 1;
        //        hallState = hallState | HALL3 << 2;
        //        static float out[3];
        //        out[0] = _itofQ16(predictedDeltaTheta);
        //        out[1] = _itofQ16(rotorSpeed);
        //        out[2] = (float)tracking;
        //        PrintWithTimestamp(out, 3);
        predictedDeltaTheta = 0;

        //rotorSpeed = _Q16ftoi(1000);


        pHA = HALL1;
        pHB = HALL2;
        pHC = HALL3;
        guard = 0;
    }
}

#ifdef TORQUE

void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void) {

    hallUpdate();
    //        if (!isTracking()) {
    //                if (driveStatus == SVM) {
    //                    InitBlockCom();
    //                    driveStatus = BLOCK;
    //                }
    //              TrapUpdate(300, CCW);
    //        }
    IFS1bits.CNIF = 0; // Clear CN interrupt
}
#endif

// When using hall effects, this function interpolates position and veloctiy

void computeRotorParameters(void) {
    // if (!guard) {
    //rotorSpeed = rotorSpeed + (rotorAcceleration >> 13);
    if(abs(predictedDeltaTheta)< PIover3){
    predictedDeltaTheta = predictedDeltaTheta + (rotorSpeed >> 13);
    rotorAngleElectrical = rotorAngleElectrical + (rotorSpeed >> 13);
    if (rotorAngleElectrical > sixPIover3) {
        rotorAngleElectrical = rotorAngleElectrical - _Q16ftoi(6.28318530718);
    } else {
        if (rotorAngleElectrical < 0) {
            rotorAngleElectrical = rotorAngleElectrical + _Q16ftoi(6.28318530718);
        }
    }
    // }
    }
}
#endif

void TrapUpdate(uint16_t torque, uint16_t direction) {
    if (torque > PTPER) {
        torque = PTPER;
    }

    if (direction == CW) {
        if ((HALL1 && HALL2 && !HALL3)) {
            GH_A_DC = 0;
            GL_A_DC = 0;
            GH_B_DC = torque;
            GL_B_DC = 0;
            GH_C_DC = 0;
            GL_C_DC = torque;
            LED1 = 1;
            LED2 = 1;
            LED3 = 0;
        } else if ((!HALL1 && HALL2 && !HALL3)) {
            GH_A_DC = 0;
            GL_A_DC = torque;
            GH_B_DC = torque;
            GL_B_DC = 0;
            GH_C_DC = 0;
            GL_C_DC = 0;
            LED1 = 0;
            LED2 = 1;
            LED3 = 0;
        } else if ((!HALL1 && HALL2 && HALL3)) {
            GH_A_DC = 0;
            GL_A_DC = torque;
            GH_B_DC = 0;
            GL_B_DC = 0;
            GH_C_DC = torque;
            GL_C_DC = 0;
            LED1 = 0;
            LED2 = 1;
            LED3 = 1;
        } else if ((!HALL1 && !HALL2 && HALL3)) {
            GH_A_DC = 0;
            GL_A_DC = 0;
            GH_B_DC = 0;
            GL_B_DC = torque;
            GH_C_DC = torque;
            GL_C_DC = 0;
            LED1 = 0;
            LED2 = 0;
            LED3 = 1;
        } else if ((HALL1 && !HALL2 && HALL3)) {
            GH_A_DC = torque;
            GL_A_DC = 0;
            GH_B_DC = 0;
            GL_B_DC = torque;
            GH_C_DC = 0;
            GL_C_DC = 0;
            LED1 = 1;
            LED2 = 0;
            LED3 = 1;
        } else if ((HALL1 && !HALL2 && !HALL3)) {
            GH_A_DC = torque;
            GL_A_DC = 0;
            GH_B_DC = 0;
            GL_B_DC = 0;
            GH_C_DC = 0;
            GL_C_DC = torque;
            LED1 = 1;
            LED2 = 0;
            LED3 = 0;
        }
    } else if (direction == CCW) {
        if ((HALL1 && HALL2 && !HALL3)) {
            GH_A_DC = 0;
            GL_A_DC = 0;
            GH_B_DC = 0;
            GL_B_DC = torque;
            GH_C_DC = torque;
            GL_C_DC = 0;
            LED1 = 1;
            LED2 = 1;
            LED3 = 0;
        } else if ((!HALL1 && HALL2 && !HALL3)) {
            GH_A_DC = torque;
            GL_A_DC = 0;
            GH_B_DC = 0;
            GL_B_DC = torque;
            GH_C_DC = 0;
            GL_C_DC = 0;
            LED1 = 0;
            LED2 = 1;
            LED3 = 0;
        } else if ((!HALL1 && HALL2 && HALL3)) {
            GH_A_DC = torque;
            GL_A_DC = 0;
            GH_B_DC = 0;
            GL_B_DC = 0;
            GH_C_DC = 0;
            GL_C_DC = torque;
            LED1 = 0;
            LED2 = 1;
            LED3 = 1;
        } else if ((!HALL1 && !HALL2 && HALL3)) {
            GH_A_DC = 0;
            GL_A_DC = 0;
            GH_B_DC = torque;
            GL_B_DC = 0;
            GH_C_DC = 0;
            GL_C_DC = torque;
            LED1 = 0;
            LED2 = 0;
            LED3 = 1;
        } else if ((HALL1 && !HALL2 && HALL3)) {
            GH_A_DC = 0;
            GL_A_DC = torque;
            GH_B_DC = torque;
            GL_B_DC = 0;
            GH_C_DC = 0;
            GL_C_DC = 0;
            LED1 = 1;
            LED2 = 0;
            LED3 = 1;
        } else if ((HALL1 && !HALL2 && !HALL3)) {

            GH_A_DC = 0;
            GL_A_DC = torque;
            GH_B_DC = 0;
            GL_B_DC = 0;
            GH_C_DC = torque;
            GL_C_DC = 0;
            LED1 = 1;
            LED2 = 0;
            LED3 = 0;
        }
    }
}


