
#include <xc.h>
#include "SVM_Torque_Control.h"
#include "PMSMBoard.h"

#ifdef TORQUE
#include "estimator.h"
#include "TrigData.h"


#define CW 0
#define CCW 1

#define SQRT_3_2 0.86602540378
#define SQRT_3 1.732050807568877

typedef struct {
    float Vr1;
    float Vr2;
    float Vr3;
} InvClarkOut;

typedef struct {
    float Va;
    float Vb;
} InvParkOut;

typedef struct {
    uint8_t sector;
    uint16_t T0;
    uint16_t Ta;
    uint16_t Tb;
    float Va;
    float Vb;
} TimesOut;


void SpaceVectorModulation(TimesOut sv);
InvClarkOut InverseClarke(InvParkOut pP);
InvParkOut InversePark(float Vd, float Vq, int16_t position);
TimesOut SVPWMTimeCalc(InvParkOut pP);

void updateTorqueController(float a) {
    //currently doesn't do much except set vq as a and vd as zero, which is super wrong
    // but lets see what it does anyway. Make sure a is between zero and 1 for now
    float ang = getRotorAngleElectrical();
//    InvParkOut testDummy;
//    testDummy.Va = .5;
//    testDummy.Vb = .6;
//    SVPWMTimeCalc(testDummy);
//    TimesOut dummy;
//    dummy.sector=2;
//    dummy.Va = -.5;
//    dummy.Vb = .3;
//    SpaceVectorModulation(dummy);

    ang = ang * 325.949323452; //2048/twopi
    int16_t index = (int16_t) ang;
    SpaceVectorModulation(SVPWMTimeCalc(InversePark(a, 0, index)));
}

/****************************   Private Stuff   *******************************/

void SpaceVectorModulation(TimesOut sv) {
    static float a = 0;
    static float b = 0;
//    static float toSend[3];
//    toSend[0] = sv.Va;
//    toSend[1] = sv.Vb;
//    toSend[2] = getRotorAngleElectrical();
//    PrintWithTimestamp(toSend, 3);
    switch (sv.sector) {
        // additions at the end for dead time
        case 1:
            a = sv.Va + 0.5774 * sv.Vb;
            b = 1.1546 * sv.Vb;
            GH_A_DC = ((uint16_t) PHASE1 * a) + 10;
            GH_B_DC = ((uint16_t) PHASE2 * b) + 10;
            GH_C_DC = 0;
            break;

        case 2:
            
            a = -1*sv.Va + 0.5774*sv.Vb;
            b = -1*sv.Va - 0.5774*sv.Vb;
            GH_A_DC = 0;
            GH_B_DC = ((uint16_t) PHASE2 * a) + 10;
            GH_C_DC = ((uint16_t) PHASE3 * b) + 10;
            break;

        case 3:
            a = -1.1547*sv.Vb;
            b = sv.Va-0.5774*sv.Vb;
            GH_A_DC = ((uint16_t) PHASE1 * b) + 10;
            GH_B_DC = 0;
            GH_C_DC = ((uint16_t) PHASE3 * a) + 10;
            break;
    }
//    static int printPre = 0;
//    static float out[7];
//    if(printPre<10){
//        printPre++;
//    }else{
//        out[0] = getRotorAngleElectrical();
//        out[1] = (float)GH_A_DC;
//        out[2] = (float)GH_B_DC;
//        out[3] = (float)GH_C_DC;
//        out[4] = (float)sv.sector;
//        out[5] = sv.Va;
//        out[6] = sv.Vb;
//        PrintWithTimestamp(out,7);
//        printPre = 0;
//    }

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

    float cosine;
    float sine;

    position = position1 % 2048;
    cos_position = (position + 512) % 2048;

    //	if (position1 <= 0) {
    //		position = 2048 + (position1 % 2048);
    //		cos_position = (2048 + ((position1 + 512) % 2048)) % 2048;
    //	} else {
    //		position = position1 % 2048;
    //		cos_position = (position1 + 512) % 2048;
    //	}

    cosine = TRIG_DATA[cos_position];
    sine = TRIG_DATA[position];

    returnVal.Va = Vd * cosine - Vq * sine;
    returnVal.Vb = Vd * sine + Vq * cosine;
    return (returnVal);
}

TimesOut SVPWMTimeCalc(InvParkOut pP) {
    //	static uint8_t out[56];
    //	static uint8_t size;

    TimesOut t;
    // compute the sector
    if (pP.Vb > 0) {
        if (pP.Va / pP.Vb > -0.57735026919) {
            t.sector = 1;
        } else {
            t.sector = 2;
        }
    } else {
        if (pP.Va / pP.Vb > 0.57735026919) {
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

void TrapUpdate(uint16_t torque, uint16_t direction)
{
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

#endif