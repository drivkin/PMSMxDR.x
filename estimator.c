#include "estimator.h"
#include <stdint.h>
#include <xc.h>
#include <math.h>
#include "PMSMBoard.h"


#define T_EST 274 //  256 prescaler
#define T_EST_S .001
#define ROT_THRESH .005
#define PI 3.14159
#define T_CLK 0.00001460224 // 1.426*10^-8 * 1024


#define MINUS_PI -3.14159
#define TWO_PI 6.28318530718

#define ALPHA_MAX 50


#define CCW 0
#define CW 1

int guard = 0;

int tracking = 0; // keeps track of whether the estimator is doing a decent job
float trackThresh = 1.5; // allowable error magnitude on predicted time of next hall event
float predictedDeltaTheta = 0;

float encoderZero = 0;

int pHA = 0;
int pHB = 0;
int pHC = 0;

int direction = 0;

float rotorAngleElectrical = 0;
float rotorAngleMechanical = 0;
float rotorSpeed = 0;
float rotorAcceleration = 0;

float c1 = 0.00009643569; // constant to convert from adjusted encoder DC to 0-2pi

// for velocity and acceleration estimators

float tNextData = 0;
int newDataFlag = 0;

ABCurrents iAB;


uint32_t getEncoderOffset(void);
void computeRotorParameters(void);
float MAVfiltP(float dat);
float MAVfiltV(float dat);
float computeRotorAngleMechanical(void);
float computeRotorSpeed(void);
float computeRotorAcceleration(void);
ABCurrents computeABCurrents(void);

float getRotorAngleMechanical(void) {
    return rotorAngleMechanical;
}

float getRotorAngleElectrical(void) {
    return rotorAngleElectrical;
}

float getRotorSpeed(void) {
    return rotorSpeed;
}

float getRotorAcceleration(void) {
    return rotorAcceleration;
}

int isTracking(){
    return tracking;
}

void initEstimators(void) {
    //for testing
    //    float hi = 0.00001460224 * 1287;
    //    float hello = T_CLK * 1287;

    //estimators use timer nine
    T9CONbits.TON = 0;
    T9CONbits.TCS = 0;
    T9CONbits.TGATE = 0;
    T9CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR9 = 0x0000;
    PR9 = T_EST;
    IFS3bits.T9IF = 0;
    IPC13bits.T9IP = 0x01;
    IEC3bits.T9IE = 1;
    T9CONbits.TON = 1;

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
    TMR5 = 0x0000;
    TMR4 = 0X0000;
    PR5 = 0xFFFF;
    PR4 = 0xffff;
    IPC7bits.T5IP = 0x01;
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
    IFS3bits.T9IF = 0; //Clear Timer3 interrupt flag
    computeRotorParameters();
    //    rotorAngleMechanical = computeRotorAngleMechanical();
    //    rotorSpeed = computeRotorSpeed();
    //    rotorAcceleration = computeRotorAcceleration();
    //   iAB = computeABCurrents();
    //#ifdef ESTIMATOR_TEST
    if (printPre < 3) {
        printPre++;
    } else {
        static float out[3];
        out[0] = getRotorAngleElectrical();
        out[1] = (float)direction;
        out[2] = (float)tracking;
        PrintWithTimestamp(out, 3);
        printPre = 0;
    }

    // #endif
}
#define FILT_SIZEP 8

float MAVfiltP(float dat) {

    static float data[FILT_SIZEP];
    static int first = 1;
    static int index = 0;
    static float avg = 1;
    if (first) {
        int i;
        float a = avg / FILT_SIZEP;
        for (i = 0; i < FILT_SIZEP; i++) {
            data[i] = a;
        }
        first = 0;
    }
    dat = dat / FILT_SIZEP;
    avg = avg - data[index] + dat;
    data[index] = dat;
    index++;
    if (index == FILT_SIZEP) {
        index = 0;
    }
    return avg;
}

#define FILT_SIZEV 20

float MAVfiltV(float dat) {

    static float data[FILT_SIZEV];
    static int first = 1;
    static int index = 0;
    static float avg = 0;
    if (first) {
        int i;
        for (i = 0; i < FILT_SIZEV; i++) {
            data[i] = 0;
        }
        first = 0;
    }
    dat = dat / FILT_SIZEV;
    avg = avg - data[index] + dat;
    data[index] = dat;
    index++;
    if (index == FILT_SIZEV) {
        index = 0;
    }
    return avg;
}

#define FILT_SIZEA 20

float MAVfiltA(float dat) {

    static float data[FILT_SIZEA];
    static int first = 1;
    static int index = 0;
    static float avg = 0;
    if (first) {
        int i;
        for (i = 0; i < FILT_SIZEA; i++) {
            data[i] = 0;
        }
        first = 0;
    }
    dat = dat / FILT_SIZEA;
    avg = avg - data[index] + dat;
    data[index] = dat;
    index++;
    if (index == FILT_SIZEA) {
        index = 0;
    }
    return avg;
}

#ifdef HALL_ESTIMATOR

void hallUpdate() {
    static int first = 1;
    guard = 1;

    // for determining if the estimator is tracking
    static float bigThreshPos = 0;
    static float smallThreshPos = 0;
    static float bigThreshNeg = 0;
    static float smallThreshNeg = 0;


    static float realTime = 1;
    static float ang;
    static float prevV = 0;
    uint16_t lsw = TMR4;
    uint16_t msw = TMR5HLD;
    uint32_t time = msw;
    time = time << 16;
    time = time | lsw;
    time = time >> 10; // to help with floating point stuff



    TMR5HLD = 0x0000;
    TMR4 = 0x0000;

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
                ang = 3.14159265359; //3 * 2pi/6
                //                ang = 1.57079632679;
            }
        } else {
            if (HALL3) {
                direction = CW;
                ang = 0;
                //                ang = 4.71238898038;
            } else {
                direction = CCW;
                ang = 3.14159265359; //3 * 2pi/6
                //                ang = 1.57079632679;

            }
        }
    } else if (HALL2 != pHB) {
        if (HALL2) {
            if (HALL1) {
                direction = CCW;
                ang = 2.09439510239; // 2 * 2pi/6
                //                ang = 0.52359877559;
            } else {
                direction = CW;
                ang = 5.23598775598; // 5*2pi/6
                //                ang = 3.66519142919;
            }
        } else {
            if (HALL1) {
                direction = CW;
                ang = 2.09439510239; // 2 * 2pi/6
                //                ang = 0.52359877559;
            } else {
                direction = CCW;
                ang = 5.23598775598; // 5*2pi/6
                //                ang = 3.66519142919;
            }
        }
    } else if (HALL3 != pHC) {
        if (HALL3) {
            if (HALL2) {
                direction = CCW;
                ang = 4.18879020479; //4*2pi/6
                //                ang = 2.61799387799;

            } else {
                direction = CW;
                ang = 1.0471975512; //2PI/6
                //                ang = 5.75958653158;
            }
        } else {
            if (HALL2) {
                direction = CW;
                ang = 4.18879020479; //4*2pi/6
                //                ang = 2.61799387799;
            } else {
                direction = CCW;
                ang = 1.0471975512; //2PI/6
                //                ang = 5.75958653158;
            }
        }
    }
    rotorAngleElectrical = ang;


    if (first) {
        bigThreshPos = 1.0471975512 * trackThresh;
        smallThreshPos = 1.0471975512 / trackThresh;
        bigThreshNeg = -1.0471975512 * trackThresh;
        smallThreshNeg = -1.0471975512 / trackThresh;

        first = 0;
    } else {
        float timeInFloat = (float) time;
        realTime = T_CLK * timeInFloat;
        float filteredTime = MAVfiltP(realTime);
        if (direction == CCW) {
            rotorSpeed = 1.0471975512 / filteredTime;
            if (smallThreshPos < predictedDeltaTheta && predictedDeltaTheta < bigThreshPos) {
                tracking = 1;
            } else {
                tracking = 0;
            }

        } else {
            rotorSpeed = -1.0471975512 / filteredTime;
            if (bigThreshNeg < predictedDeltaTheta && predictedDeltaTheta < smallThreshNeg) {
                tracking = 1;
            } else {
                tracking = 0;
            }
        }
        predictedDeltaTheta = 0;
        rotorAcceleration = (rotorSpeed - prevV) / filteredTime;

        //        rotorSpeed = realTime;
        //         rotorAcceleration = timeInFloat;
        prevV = rotorSpeed;
    }

    pHA = HALL1;
    pHB = HALL2;
    pHC = HALL3;
    guard = 0;
}

#ifdef TORQUE

void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void) {
    hallUpdate();

    IFS1bits.CNIF = 0; // Clear CN interrupt
}
#endif

// When using hall effects, this function interpolates position and veloctiy

void computeRotorParameters(void) {
    if (!guard) {
        rotorSpeed = rotorSpeed + T_EST_S*rotorAcceleration;
        predictedDeltaTheta = predictedDeltaTheta + T_EST_S*rotorSpeed;
        rotorAngleElectrical = rotorAngleElectrical + T_EST_S*rotorSpeed;
        if(rotorAngleElectrical > 6.28318530718){
            rotorAngleElectrical = rotorAngleElectrical - 6.28318530718;
        }else{
            if(rotorAngleElectrical<0){
                rotorAngleElectrical = rotorAngleElectrical + 6.28318530718;
            }
        }
    }
}
#endif


#ifdef ENCODER_POSITION

void computeRotorParameters(void) {
    /* according to the encoder datasheet, the minimum duty cycle is 16/4119 and the
     * maximum is 4111/4119, so these numbers correspond to zero and 2*pi. The DC is
     * returned by the encoder module as a number between 0 and 2^16-1. */

    //get position from encoder
    uint32_t encDC = getEncDC();
    encDC = encDC - 255; //now encDC of zero corresponds to theta = 0
    float ang = ((float) encDC) * c1;

    ang = ang - encoderZero;
    if (ang < 0) {
        ang = 6.28318530718 + ang;
    }
    rotorAngleMechanical = ang;


    // compute velocity
    static float prevPos = 0;
    static float prevDiff = 0;
    static float time = 1;
    static float v = 0;
    static float prevVDiff = 0;
    static float prevV = 0;
    static float prevA = 0;
    static float a = 0;

    float currPos = ang;
    float diff = currPos - prevPos;

    if (diff > PI) { // signals wrap around
        diff = diff - TWO_PI;
    } else {
        if (diff < MINUS_PI) {
            diff = diff + TWO_PI;
        }
    }
    diff = MAVfiltP(diff);
    v = diff / T_EST_S;
    prevPos = currPos;

    a = MAVfiltV((v - prevV) / T_EST_S);
    prevV = v;

    rotorSpeed = v;
    rotorAcceleration = a;


    //    time = time + T_EST_S;
    //
    //    float currPos = ang;
    //
    //    float diff = currPos - prevPos;
    //
    //    if (diff > PI) { // signals wrap around
    //        diff = diff - TWO_PI;
    //    } else {
    //        if (diff < MINUS_PI) {
    //            diff = diff + TWO_PI;
    //        }
    //    }
    //
    //    if (fabsf(diff) > ROT_THRESH) { // if you get a new position datapoint, get velocity by dividing by the time it took to get there
    //        newDataFlag = 1;
    //        v = diff / time;
    //    } else {
    ////        if (time > tNextData) { // if you don't get a new position datapoint by the time you thought you would based on your velocity evaluation, slow down the velocity estimate
    ////            v = prevDiff / time;
    ////        }
    //    }
    //
    ////       prevDiff = diff;
    ////        prevPos = currPos;
    ////        tNextData = time;
    //
    //
    //    if (newDataFlag) {
    //        float vDiff = v - prevV;
    //        a = vDiff / time;
    //        if (fabsf(a) > ALPHA_MAX) { // if a is greater than the max allowed, ignore the datapoint for v and a;
    //            a = prevA;
    //            rotorSpeed = prevV+a*time;
    //        } else { // if acceleration value is acceptable
    //            prevV = v;
    //            rotorSpeed = v;
    //            prevA = a;
    //            prevPos = currPos;
    //            prevVDiff = vDiff;
    //            time = 0;
    //        }
    //    } else {
    //        if (time > tNextData) {
    ////            a = prevVDiff / time;
    //        }
    //    }
    //    //rotorSpeed = v;
    //    rotorAcceleration = a;

}

/* according to the encoder datasheet, the minimum duty cycle is 16/4119 and the
 * maximum is 4111/4119, so these numbers correspond to zero and 2*pi. The DC is
 * returned by the encoder module as a number between 0 and 2^16-1. */
float computeRotorAngleMechanical(void) {
    uint32_t encDC = getEncDC();

    encDC = encDC - 255; //now encDC of zero corresponds to theta = 0
    float ang = ((float) encDC) * c1;

    ang = ang - encoderZero;
    if (ang < 0) {
        ang = 6.28318530718 + ang;
    }
    return ang;
}

float computeRotorSpeed(void) {
    static float prevPos = 0;
    static float prevDiff = 0;
    static float time = 1;
    static float v = 0;

    time = time + T_EST_S;

    float currPos = getRotorAngleMechanical();

    float diff = currPos - prevPos;

    if (diff > PI) { // signals wrap around
        diff = diff - TWO_PI;
    } else {
        if (diff < MINUS_PI) {
            diff = diff + TWO_PI;
        }
    }

    if (fabsf(diff) > ROT_THRESH) { // if you get a new position datapoint, get velocity by dividing by the time it took to get there
        newDataFlag = 1;
        v = diff / time;
        prevDiff = diff;
        prevPos = currPos;
        tNextData = time;
        time = 0;
    } else {
        if (time > tNextData) { // if you don't get a new position datapoint by the time you thought you would based on your velocity evaluation, slow down the velocity estimate
            v = prevDiff / time;
        }
    }
    return v;
}

float computeRotorAcceleration(void) {
    static float prevVDiff = 0;
    static float prevV = 0;
    static float time = 1;
    static float a = 0;

    time = time + T_EST_S;

    if (newDataFlag) {
        float v = getRotorSpeed();
        float vDiff = v - prevV;
        a = vDiff / time;
        prevV = v;
        prevVDiff = vDiff;
        newDataFlag = 0;
        time = 0;
    } else {
        if (time > tNextData) {
            a = prevVDiff / time;
        }
    }
    return a;
}


#endif




#ifdef TEST_ENCODER_OFFSET
#include "DMA_Transfer.h"

int main(void) {
    ClockInit();
    UART2Init();
    PinInit();
    MagEncoderInit();
    static uint8_t out[64];
    static uint16_t size = 0;
    while (1) {
        size = sprintf((char*) out, "%u \r\n", getEncDC() << 16);
        DMA0_UART2_Transfer(size, out);
    }
}
#endif

#ifdef ESTIMATOR_TEST

int main(void) {
    ClockInit();
    UART2Init();
    PinInit();
    MagEncoderInit();

    //estimators use timer nine
    T9CONbits.TON = 0;
    T9CONbits.TCS = 0;
    T9CONbits.TGATE = 0;
    T9CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR9 = 0x0000;
    PR9 = T_EST;
    IFS3bits.T9IF = 0;
    IPC13bits.T9IP = 0x01;
    IEC3bits.T9IE = 1;
    T9CONbits.TON = 1;


    //t6 does not generate interrupts, is used for timestamps
    T6CONbits.TON = 0;
    T6CONbits.TCS = 0;
    T6CONbits.TGATE = 0;
    T6CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR6 = 0x0000;
    PR6 = 0xffff;
    IFS2bits.T6IF = 0;
    IEC2bits.T6IE = 0;
    T6CONbits.TON = 1;
    while (1);
}



#endif



