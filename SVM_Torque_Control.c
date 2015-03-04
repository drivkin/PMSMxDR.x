
#include <xc.h>
#include "SVM_Torque_Control.h"
#include "PMSMBoard.h"
#include <libq.h>

#ifdef TORQUE
#include "EstimatorFixedPoint.h"
#include "TrigData.h"


#define CCW 0
#define CW 1

#define SQRT_3_2 0.86602540378
#define SQRT_3 1.732050807568877

typedef struct {
    float Vr1;
    float Vr2;
    float Vr3;
} InvClarkOut;

typedef struct {
    _Q16 Va;
    _Q16 Vb;
} InvParkOut;

typedef struct {
    uint8_t sector;
    uint16_t T0;
    uint16_t Ta;
    uint16_t Tb;
    _Q16 Va;
    _Q16 Vb;
} TimesOut;


void SpaceVectorModulation(TimesOut sv);
InvClarkOut InverseClarke(InvParkOut pP);
InvParkOut InversePark(float Vd, float Vq, int16_t position);
TimesOut SVPWMTimeCalc(InvParkOut pP);
//void TrapUpdate(uint16_t torque, uint16_t direction);

_Q16 cOne;
_Q16 c2;
_Q16 c3;
_Q16 c4;
_Q16 PWMPer;
void initSVM(void){
    cOne = _Q16ftoi(-0.57735026919);
    c2 = _Q16ftoi(0.57735026919);
    c3 = _Q16ftoi(1.1546);
    c4 = _Q16ftoi(-1.1547);
    PWMPer = _Q16ftoi(1000); // value of pwm period registers
}

void updateTorqueController(float torque) {
    //controls torque in an open loop fashion by computing the back EMF from the
    //velocity and then setting Vq some amount above that as determined by the torque command
    static int vqUpdatePrescaler = 0;
    static float vq = 0;
    static float vd = 0;
    static float iq = 0;
    static float Ls  = .001940;
    static float ki = 0.055;
    static float ke = 0.00904795852; //remember the velocity we get is electrical
    static float cf = 0.00006445775; // dynamic friction
    static float r = 28.2;
    float ang = getRotorAngleElectrical();
    if (vqUpdatePrescaler < 10) {
        vqUpdatePrescaler++;
    } else {
        iq = (torque + cf * getRotorSpeed()) * ki;
        vd = -getRotorSpeed()*Ls*iq;
        vq = (iq*r + ke * getRotorSpeed()) / 8.164; // make sure to change the 10 to a measured battery voltage
        vqUpdatePrescaler = 0;
        if(vq>1){
            vq = 1;
        }

        static float out[4];
        out[0] = ang;
        out[1] = getRotorSpeed();;
        out[2] = vq;
        out[3] = vd;
        PrintWithTimestamp(out, 4);
    }
    ang = ang * 325.949323452; //2048/twopi
    int16_t index = (int16_t) ang;
    SpaceVectorModulation(SVPWMTimeCalc(InversePark(vq, vd, index)));
}

/****************************   Private Stuff   *******************************/

void SpaceVectorModulation(TimesOut sv) {
    static _Q16 a = 0;
    static _Q16 b = 0;
    //    static float toSend[3];
    //    toSend[0] = sv.Va;
    //    toSend[1] = sv.Vb;
    //    toSend[2] = getRotorAngleElectrical();
    //    PrintWithTimestamp(toSend, 3);
    switch (sv.sector) {
            // additions at the end for dead time
        case 1:
            a = sv.Va + _Q16mpy(c2, sv.Vb);
            b = _Q16mpy(c3, sv.Vb);
            GH_A_DC = ((uint16_t) _itofQ16(_Q16mpy(PWMPer, a))) + 10;
            GH_B_DC = ((uint16_t) _itofQ16(_Q16mpy(PWMPer, b))) + 10;
            GH_C_DC = 0;
            break;

        case 2:

            a = _Q16neg(sv.Va) + _Q16mpy(c2, sv.Vb);
            b = _Q16neg(sv.Va) + _Q16neg(_Q16mpy(c2, sv.Vb));
            GH_A_DC = 0;
            GH_B_DC = ((uint16_t) _itofQ16(_Q16mpy(PWMPer, a))) + 10;
            GH_C_DC = ((uint16_t) _itofQ16(_Q16mpy(PWMPer, b))) + 10;
            break;

        case 3:
            a = _Q16mpy(c4, sv.Vb);
            b = sv.Va + _Q16neg(_Q16mpy(c2, sv.Vb));
            GH_A_DC = ((uint16_t) _itofQ16(_Q16mpy(PWMPer, b))) + 10;
            GH_B_DC = 0;
            GH_C_DC = ((uint16_t) _itofQ16(_Q16mpy(PWMPer, a))) + 10;
            break;
    }
    //        static int printPre = 0;
    //        static float out[7];
    //        if(printPre<10){
    //            printPre++;
    //        }else{
    //            out[0] = getRotorAngleElectrical();
    //            out[1] = (float)GH_A_DC;
    //            out[2] = (float)GH_B_DC;
    //            out[3] = (float)GH_C_DC;
    //            out[4] = (float)sv.sector;
    //            out[5] = _itofQ16(a);
    //            out[6] = _itofQ16(b);
    //            PrintWithTimestamp(out,7);
    //            printPre = 0;
    //        }

    //just for testing
    //  GH_A_DC = GH_B_DC = GH_C_DC = 0;

    //	switch (sv.sector) {
    //	case 1:
    //		GH_A_DC = ((uint16_t) PTPER * (.5 - .375 * sv.Vb - .649519 * sv.Va)) - 10;
    //		GH_B_DC = ((uint16_t) PTPER * (.5 + .375 * sv.Vb - .216506 * sv.Va)) - 10;
    //		GH_C_DC = ((uint16_t) PTPER * (.5 - .375 * sv.Vb + .216506 * sv.Va)) - 10;
    //		break;
    //	case 2:
    //		GH_A_DC = ((uint16_t) PTPER * (.5 - .433013 * sv.Va)) - 10;
    //		GH_B_DC = ((uint16_t) PTPER * (.5 + .75 * sv.Vb)) - 10;
    //		GH_C_DC = ((uint16_t) PTPER * (.5 + .433013 * sv.Va)) - 10;
    //		break;
    //	case 3:
    //		GH_A_DC = ((uint16_t) PTPER * (.5 - 0.375 * sv.Vb + .216506 * sv.Va)) - 10;
    //		GH_B_DC = ((uint16_t) PTPER * (.5 + 0.375 * sv.Vb + .216506 * sv.Va)) - 10;
    //		GH_C_DC = ((uint16_t) PTPER * (.5 - 0.375 * sv.Vb + .649519 * sv.Va)) - 10;
    //		break;
    //	default:
    //		break;
    //	}
}

InvClarkOut InverseClarke(InvParkOut pP) {
    InvClarkOut returnVal;
    returnVal.Vr1 = pP.Vb;
    returnVal.Vr2 = -.5 * pP.Vb + SQRT_3_2 * pP.Va;
    returnVal.Vr3 = -.5 * pP.Vb - SQRT_3_2 * pP.Va;
    return (returnVal);
}

InvParkOut InversePark(float Vq, float Vd, int16_t position1) {
    static int16_t position;
    static int16_t cos_position;
    InvParkOut returnVal;

    _Q16 cosine;
    _Q16 sine;

    position = position1 % 2048;
    cos_position = (position + 512) % 2048;

    //	if (position1 <= 0) {
    //		position = 2048 + (position1 % 2048);
    //		cos_position = (2048 + ((position1 + 512) % 2048)) % 2048;
    //	} else {
    //		position = position1 % 2048;
    //		cos_position = (position1 + 512) % 2048;
    //	}

    cosine = _Q16ftoi(TRIG_DATA[cos_position]);
    sine = _Q16ftoi(TRIG_DATA[position]);

    returnVal.Va = _Q16mpy(_Q16ftoi(Vd), cosine) - _Q16mpy(_Q16ftoi(Vq), sine);
    returnVal.Vb = _Q16mpy(_Q16ftoi(Vd), sine) + _Q16mpy(_Q16ftoi(Vq), cosine);
    return (returnVal);
}

TimesOut SVPWMTimeCalc(InvParkOut pP) {
    //	static uint8_t out[56];
    //	static uint8_t size;

    TimesOut t;
    // compute the sector
    if (pP.Vb > 0) {
        if (_Q16div(pP.Va, pP.Vb) > cOne) {
            t.sector = 1;
        } else {
            t.sector = 2;
        }
    } else {
        if (_Q16div(pP.Va, pP.Vb) > c2) {
            t.sector = 2;
        } else {
            t.sector = 3;
        }
    }
    //
    //	size = sprintf((char *) out, "%u\r\n", t.sector);
    //	DMA0_UART2_Transfer(size, out);

    t.Va = pP.Va;
    t.Vb = pP.Vb;

    return (t);
}

//void TrapUpdate(uint16_t torque, uint16_t direction) {
//    if (torque > PTPER) {
//        torque = PTPER;
//    }
//
//    if (direction == CW) {
//        if ((HALL1 && HALL2 && !HALL3)) {
//            GH_A_DC = 0;
//            GL_A_DC = 0;
//            GH_B_DC = torque;
//            GL_B_DC = 0;
//            GH_C_DC = 0;
//            GL_C_DC = torque;
//
//
//            ALTDTR1 = 2000;
//            ALTDTR2 = 10;
//            ALTDTR3 = 10;
//            //			LED1 = 1;
//            //			LED2 = 1;
//            //			LED3 = 0;
//        } else if ((!HALL1 && HALL2 && !HALL3)) {
//            GH_A_DC = 0;
//            GL_A_DC = torque;
//            GH_B_DC = torque;
//            GL_B_DC = 0;
//            GH_C_DC = 0;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 10;
//            ALTDTR3 = 2000;
//
//            //			LED1 = 0;
//            //			LED2 = 1;
//            //			LED3 = 0;
//        } else if ((!HALL1 && HALL2 && HALL3)) {
//            GH_A_DC = 0;
//            GL_A_DC = torque;
//            GH_B_DC = 0;
//            GL_B_DC = 0;
//            GH_C_DC = torque;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 2000;
//            ALTDTR3 = 10;
//
//            //			LED1 = 0;
//            //			LED2 = 1;
//            //			LED3 = 1;
//        } else if ((!HALL1 && !HALL2 && HALL3)) {
//            GH_A_DC = 0;
//            GL_A_DC = 0;
//            GH_B_DC = 0;
//            GL_B_DC = torque;
//            GH_C_DC = torque;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 2000;
//            ALTDTR2 = 10;
//            ALTDTR3 = 10;
//
//            //			LED1 = 0;
//            //			LED2 = 0;
//            //			LED3 = 1;
//        } else if ((HALL1 && !HALL2 && HALL3)) {
//            GH_A_DC = torque;
//            GL_A_DC = 0;
//            GH_B_DC = 0;
//            GL_B_DC = torque;
//            GH_C_DC = 0;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 10;
//            ALTDTR3 = 2000;
//            //			LED1 = 1;
//            //			LED2 = 0;
//            //			LED3 = 1;
//        } else if ((HALL1 && !HALL2 && !HALL3)) {
//            GH_A_DC = torque;
//            GL_A_DC = 0;
//            GH_B_DC = 0;
//            GL_B_DC = 0;
//            GH_C_DC = 0;
//            GL_C_DC = torque;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 2000;
//            ALTDTR3 = 10;
//
//            //			LED1 = 1;
//            //			LED2 = 0;
//            //			LED3 = 0;
//        }
//    } else if (direction == CCW) {
//        if ((HALL1 && HALL2 && !HALL3)) {
//            GH_A_DC = 0;
//            GL_A_DC = 0;
//            GH_B_DC = 0;
//            GL_B_DC = torque;
//            GH_C_DC = torque;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 2000;
//            ALTDTR2 = 10;
//            ALTDTR3 = 10;
//
//
//            //            LED1 = 1;
//            //            LED2 = 1;
//            //            LED3 = 0;
//        } else if ((!HALL1 && HALL2 && !HALL3)) {
//            GH_A_DC = torque;
//            GL_A_DC = 0;
//            GH_B_DC = 0;
//            GL_B_DC = torque;
//            GH_C_DC = 0;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 10;
//            ALTDTR3 = 2000;
//
//            //            LED1 = 0;
//            //            LED2 = 1;
//            //            LED3 = 0;
//        } else if ((!HALL1 && HALL2 && HALL3)) {
//            GH_A_DC = torque;
//            GL_A_DC = 0;
//            GH_B_DC = 0;
//            GL_B_DC = 0;
//            GH_C_DC = 0;
//            GL_C_DC = torque;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 2000;
//            ALTDTR3 = 10;
//            //            LED1 = 0;
//            //            LED2 = 1;
//            //            LED3 = 1;
//        } else if ((!HALL1 && !HALL2 && HALL3)) {
//            GH_A_DC = 0;
//            GL_A_DC = 0;
//            GH_B_DC = torque;
//            GL_B_DC = 0;
//            GH_C_DC = 0;
//            GL_C_DC = torque;
//
//            ALTDTR1 = 2000;
//            ALTDTR2 = 10;
//            ALTDTR3 = 10;
//
//            //            LED1 = 0;
//            //            LED2 = 0;
//            //            LED3 = 1;
//        } else if ((HALL1 && !HALL2 && HALL3)) {
//            GH_A_DC = 0;
//            GL_A_DC = torque;
//            GH_B_DC = torque;
//            GL_B_DC = 0;
//            GH_C_DC = 0;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 10;
//            ALTDTR3 = 2000;
//
//            //            LED1 = 1;
//            //            LED2 = 0;
//            //            LED3 = 1;
//        } else if ((HALL1 && !HALL2 && !HALL3)) {
//
//            GH_A_DC = 0;
//            GL_A_DC = torque;
//            GH_B_DC = 0;
//            GL_B_DC = 0;
//            GH_C_DC = torque;
//            GL_C_DC = 0;
//
//            ALTDTR1 = 10;
//            ALTDTR2 = 2000;
//            ALTDTR3 = 10;
//            //            LED1 = 1;
//            //            LED2 = 0;
//            //            LED3 = 0;
//        }
//    }
//}

#endif