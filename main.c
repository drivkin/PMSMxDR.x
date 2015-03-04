/*
 * File:   PMSMx.c
 * Author: Pavlo Milo Manovi
 *
 *
 * If being used with the CAN enabled PMSM Board obtained at either pavlo.me or off of
 * http://github.com/FrauBluher/ as of v 1.6 the following pins are reserved on the
 * dsPIC33EP256MC506:
 *
 * THIS NEEDS TO BE UPDATED FOR v1.9
 */

#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "PMSMBoard.h"
#include "CircularBuffer.h"
#include "DRV8301.h"
#include "DMA_Transfer.h"




#if defined (CHARACTERIZE_POSITION) || defined (CHARACTERIZE_VELOCITY)
#include "PMSM_Characterize.h"

#else

#ifdef VELOCITY
#include "PMSM_Velocity.h"
#endif

#ifdef POSITION
#include "PMSM_Position.h"
#endif

#endif


CircularBuffer uartBuffer;
uint8_t uartBuf[64];
CircularBuffer canBuffer;
uint8_t canBuf[64];
CircularBuffer spiBuffer;
uint16_t spiBuf[64];
uint16_t emptyBuf[64] = {};


ADCBuffer ADCBuff;

uint16_t events = 0;
uint16_t faultPrescalar = 0;
uint16_t faultPrescalar1 = 0;
uint16_t commutationPrescalar = 0;
uint16_t torque;

enum {
    EVENT_UART_DATA_READY = 0x01,
    EVENT_CAN_RX = 0x02,
    EVENT_SPI_RX = 0x04,
    EVENT_REPORT_FAULT = 0x08,
    EVENT_UPDATE_SPEED = 0x10,
    EVENT_ADC_DATA = 0x20,
    EVENT_QEI_RQ = 0x40
};

void EventChecker(void);
uint16_t ADC_LPF(void);

#ifdef RUN_FULL

int main(void) {

    for (torque = 0; torque < 65533; torque++) {
        Nop();
    }
    LED1 = 1;
    InitBoard(&ADCBuff, &uartBuffer, &spiBuffer, EventChecker);
    CB_Init(&uartBuffer, uartBuf, 32);
    CB_Init(&spiBuffer, (uint8_t *) spiBuf, 128);

    //EN_GATE = 0;


    //LED4 = 1;
    //    GH_A_DC = 50;
    //    GH_B_DC = 50;
    //    GH_C_DC = 50;
#ifdef POSITION
    //SetPosition(1000);
#endif

#ifdef VELOCITY
    // SetVelocity(500);
#endif

    while (1) {
//        //        if(i<5000){
//        //            i++;
//        //
//        //        }else{
//        //
//        //          // LED1^=1;
//        //            size = sprintf((char *) out, "%u\n",getEncDC()<<16);
//        //            DMA0_UART2_Transfer(size, out);
//        //            i = 0;
//        //        }
//
//        if (events & EVENT_UPDATE_SPEED) {
//#if defined (CHARACTERIZE_POSITION) || defined (CHARACTERIZE_VELOCITY)
//            CharacterizeStep();
//#else
//#ifdef VELOCITY
//            //   PMSM_Update_Velocity();
//#endif
//#ifdef POSITION
//            //PMSM_Update_Position();
//#endif
//#ifdef TORQUE
////            if (isTracking()) {
////                if (getDriveStatus() == BLOCK) {
////                    initSVMCom();
////                    setDriveStatus(SVM);
////                }
////                updateTorqueController(0.3);
////            }
//
//
//#endif
//
//#endif
//            events &= ~EVENT_UPDATE_SPEED;
//        }
//
//        if (events & EVENT_UART_DATA_READY) {
//            events &= ~EVENT_UART_DATA_READY;
//        }
//
//        if (events & EVENT_CAN_RX) {
//            events &= ~EVENT_CAN_RX;
//        }
//
//        if (events & EVENT_SPI_RX) {
//            static uint8_t message[32];
//            uint16_t size;
//            uint8_t out[56];
//            message[0] = 0xFF;
//            message[1] = 0xFF;
//            message[2] = 0xFF;
//            message[3] = 0xFF;
//            message[4] = 0xFF;
//            message[5] = 0xFF;
//            message[6] = 0xFF;
//            message[7] = 0xFF;
//
//            CB_ReadByte(&spiBuffer, &message[0]);
//            CB_ReadByte(&spiBuffer, &message[1]);
//            CB_ReadByte(&spiBuffer, &message[2]);
//            CB_ReadByte(&spiBuffer, &message[3]);
//            CB_ReadByte(&spiBuffer, &message[4]);
//            CB_ReadByte(&spiBuffer, &message[5]);
//            CB_ReadByte(&spiBuffer, &message[6]);
//            CB_ReadByte(&spiBuffer, &message[7]);
//
//            CB_Init(&spiBuffer, &spiBuf, 64);
//            LED1 ^= LED1;
//            size = sprintf((char *) out, "0x%X, 0x%X, 0x%X, 0x%X\r\n",
//                    ((message[0] << 8) | message[1]), ((message[2] << 8) | message[3]),
//                    ((message[4] << 8) | message[5]), ((message[6] << 8) | message[7]));
//            DMA0_UART2_Transfer(size, out);
//            events &= ~EVENT_SPI_RX;
//        }
//
//        if (events & EVENT_REPORT_FAULT) {
//            events &= ~EVENT_REPORT_FAULT;
//        }
//
//        if (events & EVENT_ADC_DATA) {
//            size = sprintf((char *) out, "%i, %i, %u\r\n", ADCBuff.Adc1Data[0], ADCBuff.Adc1Data[1], getEncDC() << 16);
//            DMA0_UART2_Transfer(size, out);
//            events &= ~EVENT_ADC_DATA;
//        }
    }
}
#endif

void EventChecker(void) {
#if defined (CHARACTERIZE_POSITION) || defined (CHARACTERIZE_VELOCITY)
#else
    //    Until I can make a nice non-blocking way of checking the drv for faults
    //    this will be called approximately every second and will block for 50uS
    //    Pushing the DRV to its max SPI Fcy should bring this number down a little.
    if (faultPrescalar > 15000) {
        DRV8301_UpdateStatus();
        faultPrescalar = 0;
    } else {
        faultPrescalar++;
    }
//    //    static uint16_t size;
//    //    static uint8_t out[56];
//    //    size = sprintf((char *) out, "%u\n", getEncDC() << 16);
//    //    DMA0_UART2_Transfer(size, out);
//
//    if (uartBuffer.dataSize) {
//        events |= EVENT_UART_DATA_READY;
//    }
//
//    if (canBuffer.dataSize) {
//        events |= EVENT_CAN_RX;
//    }
//
//    if (spiBuffer.dataSize > 6) {
//        //The first bit of SPI is nonsense from the DRV due to it starting up
//        //that needs to be handled in the event handler which will process this
//        //event.
//        //  events |= EVENT_SPI_RX;
//    }
//
//#endif
//    if (ADCBuff.newData) {
//        ADCBuff.newData = 0;
//        // events |= EVENT_ADC_DATA;
//    }
//#ifdef SINE
//
//#endif
//    if (commutationPrescalar > 1) {
//        events |= EVENT_UPDATE_SPEED;
//        commutationPrescalar = 0;
//    } else {
//        commutationPrescalar++;
//    }
}
#endif

uint16_t ADC_LPF(void) {
    static float rk1 = 0;
    static float rk2 = 0;
    int i;

    //LPF
    for (i = 0; i < 10; i++) {
        rk1 = .1 * (float) ADCBuff.Adc1Data[i] + .9 * rk2;
        rk2 = rk1;
    }

    return ((uint16_t) rk1);
}

#ifdef FOSC_PER_CALC

int main(void) {

    ClockInit();
    PinInit();
    //setup 32 bit timer with 4&5
    T5CONbits.TON = 0;
    T4CONbits.TON = 0;
    T4CONbits.T32 = 1;
    T4CONbits.TCS = 0;
    T4CONbits.TGATE = 0;
    T4CONbits.TCKPS = 0b11; // Select 1:1 Prescaler


    TMR5 = 0x0000;
    TMR4 = 0X0000;
    PR5 = 0x007F; //
    PR6 = 0xFFFF; //
    IPC7bits.T5IP = 0x01;
    IFS1bits.T5IF = 0;
    IEC1bits.T5IE = 1;
    T4CONbits.TON = 1;
    while (1);

}

int i = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void) {
    IFS1bits.T5IF = 0; // Clear Timer1 Interrupt Flag
    LED1 ^= 1;
}

#endif

#ifdef UART_THROUGHPUT

int main(void) {

    ClockInit();
    UART2Init();
    PinInit();

    //setup 32 bit timer with 4&5
    T5CONbits.TON = 0;
    T4CONbits.TON = 0;
    T4CONbits.T32 = 1;
    T4CONbits.TCS = 0;
    T4CONbits.TGATE = 0;
    T4CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    TMR5 = 0x0000;
    TMR4 = 0X0000;
    PR5 = 0xFFFF;
    PR4 = 0xffff;
    IPC7bits.T5IP = 0x01;
    IFS1bits.T5IF = 0;
    IEC1bits.T5IE = 1;


    //timestamp
    T6CONbits.TON = 0;
    T6CONbits.TCS = 0;
    T6CONbits.TGATE = 0;
    T6CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR6 = 0x000a;
    PR6 = 0xffff;
    IFS2bits.T6IF = 0;
    IEC2bits.T6IE = 0;
    T6CONbits.TON = 1;

    //generate test sequence


    //    static uint8_t sendbuf[64];
    static uint16_t out[2000];
    static uint16_t size = 0;
    static uint16_t loc_size = 0;
    uint16_t i = 0;
    //
    //    size = sprintf((char*) out, "GET IT ON:");
    //    DMA0_UART2_Transfer(size, out);

    T4CONbits.TON = 1;

    //works
    //    for(i=0;i<1000;i++){
    //        out[i]=i;
    //    }
    //    DMA0_UART2_Transfer(2000,out);



    for (i = 0; i < 600; i++) {
        out[0] = i;
        PrintWithTimestamp(out, 1);
    }

    uint32_t j;
    for (j = 0; j < 7000000; j++);

    for (i = 0; i < 500; i++) {
        out[0] = i;
        PrintWithTimestamp(out, 1);
    }

    while (1);

}

int i = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void) {
    IFS1bits.T5IF = 0; // Clear Timer1 Interrupt Flag
    LED1 ^= 1;
}

int intcount = 0;

void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
    static uint8_t out[64];
    static uint16_t size = 0;
    //    if (intcount == 0) {
    //        size = sprintf((char*) out, "time: %x %x", TMR5, TMR4);
    //        DMA0_UART2_Transfer(size, out);
    //        intcount++;
    //    } else {
    //        intcount++;
    //    }
}

#endif

#ifdef SAMPLER_TEST

int main(void) {

    ClockInit();
    UART2Init();
    PinInit();
    MagEncoderInit();
    //timestamp
    T6CONbits.TON = 0;
    T6CONbits.TCS = 0;
    T6CONbits.TGATE = 0;
    T6CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
    TMR6 = 0x0000;
    PR6 = 0xffff;
    IFS2bits.T6IF = 0;
    IEC2bits.T6IE = 0;
    T6CONbits.TON = 1;

    SamplerInit();
    while (1);
}


#endif