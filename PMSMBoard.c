#include "PMSMBoard.h"
#include <timer.h>
#include "Uart2.h"
#include "pps.h"
#include "DRV8301.h"
#include "PMSMBoard.h"


#if defined (CHARACTERIZE_POSITION) || defined (CHARACTERIZE_VELOCITY)
#include "PMSM_Characterize.h"
#else

#ifdef POSITION
#include "PMSM_Position.h"
#endif

#ifdef VELOCITY
#include "PMSM_Velocity.h"
#endif

#ifdef TORQUE
#include "EstimatorFixedPoint.h"
#include "SVM_Torque_Control.h"
#endif


#endif

#include "DMA_Transfer.h"
#include "SPIdsPIC.h"
#include <uart.h>
#include <qei32.h>

_FOSCSEL(FNOSC_FRC & IESO_OFF);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FICD(ICS_PGD1 & JTAGEN_OFF);

//static MotorInfo motorInformation;
static DRV8301_Info motorDriverInfo;
static InitStatus initInfo = {};



void (*eventCallbackFcn)(void);

uint32_t Capture1 = 0;
uint32_t Capture2 = 0;
uint32_t encDC = 1651651651;
uint16_t captureReading = 12;


#define ENC PORTDbits.RD2

#ifndef UART_TEST

void InitBoard(ADCBuffer *ADBuff, CircularBuffer *cB, CircularBuffer *spi_cB, void *eventCallback) {
    if (!initInfo.initReg) {

        uint32_t i;

        ClockInit();
        UART2Init();


        //
        PinInit();
        MotorInit();

        //   static uint16_t size;
        //    static uint8_t outSDSS[56];
        //       LED3=1;
        //        size = sprintf((char *) out, "%x\r\n", RCON);
        //        DMA0_UART2_Transfer(size, out);
        LED2 = 1;
        for (i = 0; i < 750000; i++) {
            Nop(); //Let the DRV catch it's breath...
        }

        SPI1_Init();
        {
            DMA2REQbits.FORCE = 1;
            while (DMA2REQbits.FORCE == 1);
            CS = 1;
        }

        DMA1_UART2_Enable_RX(cB);
        DMA3_SPI_Enable_RX(spi_cB);
        DMA6_ADC_Enable(ADBuff);
        ADCInit();
        DRV8301_Init(&motorDriverInfo);
        initEstimators();
        initSVM();
#ifdef TORQUE

#endif
        CNInit();

#ifndef SINE

#endif
#ifdef QEI
        QEIInit();
        //PMSM_Init(&motorInformation);

#endif

        EventCheckInit(eventCallback);
        TimersInit();
        //MagEncoderInit();
        //SamplerInit();


        //		if (!(initInfo.ClockInited & initInfo.EventCheckInited
        //			& initInfo.MotorInited & initInfo.PinInited
        //			& initInfo.TimersInited & initInfo.UARTInited)) {
        //			while (1);
        //		}
    } else {
        while (1); //Crash and burn.
    }
}
#endif

#ifdef UART_TEST

void InitBoard(ADCBuffer *ADBuff, CircularBuffer *cB, CircularBuffer *spi_cB, void *eventCallback) {
    if (!initInfo.initReg) {



        ClockInit();
        UART2Init();
        PinInit();

        {
            DMA2REQbits.FORCE = 1;
            while (DMA2REQbits.FORCE == 1);
            CS = 1;
        }

        //DMA1_UART2_Enable_RX(cB);
        T7CONbits.TON = 0;
        T7CONbits.TCS = 0;
        T7CONbits.TGATE = 0;
        T7CONbits.TCKPS = 0b0; // Select 1:1 Prescaler
        TMR7 = 0x00;
        PR7 = 1000; // 15015 Hz
        //  PR7 = 15000;
        IPC12bits.T7IP = 0x01;
        IFS3bits.T7IF = 0;
        IEC3bits.T7IE = 1;
        T7CONbits.TON = 1;


        while (1); //Crash and burn.
    }
}

int i = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T7Interrupt(void) {
    IFS3bits.T7IF = 0; // Clear Timer1 Interrupt Flag
    static uint16_t size;
    static uint8_t out[56];
    LED1 ^= 1;
    size = sprintf((char *) out, "%d %d %d \r\n", i, i, i);
    DMA0_UART2_Transfer(size, out);
    i++;

}
#endif

void UART2Init(void) {
    if (1) { //!(initInfo.UARTInited & 0x01)) {
        U2MODEbits.STSEL = 0; // 1-stop bit
        U2MODEbits.PDSEL = 0; // No parity, 8-data bits
        U2MODEbits.ABAUD = 0; // Auto-baud disabled
        U2MODEbits.BRGH = 1; // High speed UART mode...
        U2BRG = 18; //37 for 115200 on BRGH 0, 460800 on BRGH 1, 921600 = 18
        //BRGH = 0, BRG = 18 for 230400
        U2STAbits.UTXISEL0 = 0; // int on last character shifted out tx register
        U2STAbits.UTXISEL1 = 0; // int on last character shifted out tx register
        U2STAbits.URXISEL1 = 0; // Interrupt after one RX character is received
        U2MODEbits.UARTEN = 1; // Enable UART
        U2STAbits.UTXEN = 1; // Enable UART TX

        initInfo.UARTInited = 1;
    } else {
        while (1);
    }
}

void MotorInit() {
#ifdef SINE
    initSVMCom();
#else
    InitBlockCom();

#endif
    EN_GATE = 1;
    DC_CAL = 0;

    initInfo.MotorInited = 1;
}

void InitBlockCom(void) {

    PTCON = 0x0000;
    /* Set PWM Period on Primary Time Base */
    PTPER = 400;
    /* Set Phase Shift */
    PHASE1 = 0;
    SPHASE1 = 0;
    PHASE2 = 0;
    SPHASE2 = 0;
    PHASE3 = 0;
    SPHASE3 = 0;
    /* Set Duty Cycles */
    PDC1 = 0;
    SDC1 = 0;
    PDC2 = 0;
    SDC2 = 0;
    PDC3 = 0;
    SDC3 = 0;
    /* Set Dead Time Values */
    DTR1 = DTR2 = DTR3 = 20;
    ALTDTR1 = ALTDTR2 = ALTDTR3 = 10;
    /* Set PWM Mode to Independent */
    IOCON1 = IOCON2 = IOCON3 = 0xCC00;
    //Set unused PWM outputs as GPIO driven
    IOCON4 = 0;
    /* Set Primary Time Base, Edge-Aligned Mode and Independent Duty Cycles */
    PWMCON1 = PWMCON2 = PWMCON3 = 0x0000;
    /* Configure Faults */
    FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;
    /* 1:1 Prescaler */
    PTCON2 = 0x0006;
    /* Enable PWM Module */
    PTCON = 0x8000;
}

void initSVMCom(void) {
    //set periods
    PHASE1 = 1000;
    PHASE2 = 1000;
    PHASE3 = 1000;
    /* Set Duty Cycles */
    PDC1 = PDC2 = PDC3 = 0;
    //PDC1 = 500;
    /* Set Dead Time Values */
    DTR1 = DTR2 = DTR3 = 0; // these are ignored in this mode
    ALTDTR1 = ALTDTR2 = ALTDTR3 = 10;
    // ALTDTR3 = 2000;

    /* Set PWM Mode to Complementary */
    IOCON1 = IOCON2 = IOCON3 = 0xC000;

    /* Set Independent Time Bases, Center-Aligned mode and
     Independent Duty Cycles */
    PWMCON1 = PWMCON2 = PWMCON3 = 0x0204;

    /* Configure Faults */
    FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;

    /* 1:8 Prescaler */
    PTCON2 = 0x0000;


    //    //ADC trigger stuff.
    //    TRGCON1bits.TRGDIV = 0;
    //    TRGCON1bits.TRGSTRT = 0b111111;
    //    TRIG1 = PHASE1 - 1;


    /* Enable PWM Module */
    PTCON = 0x8000;
}

void ClockInit(void) {
    // 140.03 MHz VCO  -- 70 MIPS
    PLLFBD = 74;
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;

    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);

    __builtin_write_OSCCONL(OSCCON | 0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1);

    initInfo.ClockInited = 1;
}

void PinInit(void) {
    TRISDbits.TRISD7 = 1; //QEI_A
    TRISDbits.TRISD6 = 1; //QEI_B
    TRISDbits.TRISD5 = 1; //QEI_C

    // 0 - Output, 1 - Input
    TRIS_EN_GATE = 0;
    TRIS_DC_CAL = 0;
    TRIS_CS = 0;

    TRIS_HALL1 = 1;
    TRIS_HALL2 = 1;
    TRIS_HALL3 = 1;

    //Ensuring that SPI remapped pins' tristates are set correctly.
    TRISEbits.TRISE7 = 1; //MISO
    TRISGbits.TRISG6 = 0; //MOSI
    TRISGbits.TRISG8 = 0; //SCLK

    CNPDEbits.CNPDE7 = 1;

    //Input capture for magnetic encoder
    TRISDbits.TRISD2 = 1;

    TRIS_LED1 = 0;
    TRIS_LED2 = 0;
    TRIS_LED3 = 0;
    TRIS_LED4 = 1;

    //Right now no analog peripherals are being used, so we let digital
    //peripherals take over.
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELG = 0;

    TRISDbits.TRISD4 = 0;

    //Unlock PPS Registers
    __builtin_write_OSCCONL(OSCCON & ~(1 << 6));

    OUT_PIN_PPS_RP68 = OUT_FN_PPS_U2TX; //U2Tx
    IN_FN_PPS_U2RX = IN_PIN_PPS_RP67; //U2Rx
    OUT_PIN_PPS_RP118 = OUT_FN_PPS_SDO1; //SDO
    OUT_PIN_PPS_RP120 = OUT_FN_PPS_SCK1; //SCLK
    IN_FN_PPS_SDI1 = IN_PIN_PPS_RP87; //SDI

    IN_FN_PPS_QEI1 = IN_PIN_PPS_RP71; //QEI Index
    IN_FN_PPS_QEB1 = IN_PIN_PPS_RP70; //QEI B
    IN_FN_PPS_QEA1 = IN_PIN_PPS_RP69; //QEI A

    IN_FN_PPS_IC1 = IN_PIN_PPS_RP66; // input capture for magnetic encoder

    //Lock PPS Registers
    __builtin_write_OSCCONL(OSCCON | (1 << 6));


    initInfo.PinInited = 1;
}

void TimersInit(void) {
    //		T3CONbits.TON = 0; // Stop any 16-bit Timer3 operation
    //		T2CONbits.TON = 0; // Stop any 16/32-bit Timer3 operation
    //		T2CONbits.T32 = 1; // Enable 32-bit Timer mode
    //		T2CONbits.TCS = 0; // Select internal instruction cycle clock
    //		T2CONbits.TGATE = 0; // Disable Gated Timer mode
    //		T2CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
    //		TMR3 = 0x00; // Clear 32-bit Timer (msw)
    //		TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    //		PR3 = 0xFFFF; // Load 32-bit period value (msw)
    //		PR2 = 0xFFFF; // Load 32-bit period value (lsw)
    //		IPC2bits.T3IP = 0x03; // Set Timer3 Interrupt Priority Level
    //		IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
    //		IEC0bits.T3IE = 1; // Enable Timer3 interrupt
    //		T2CONbits.TON = 1; // Start 32-bit Timer

    //Timer 5 for ADC Triggering if not using the PWM compare ADC trigger.
    //		TMR5 = 0x0000;
    //		T5CONbits.TCKPS = 3;
    //		PR5 = 68; // Trigger ADC1at a rate of 4kHz
    //		IFS1bits.T5IF = 0; // Clear Timer5 interrupt
    //		IEC1bits.T5IE = 0; // Disable Timer5 interrupt
    //		T5CONbits.TON = 1; // Start Timer5
    //IPC7bits.T5IP = 2;

    T7CONbits.TON = 0;
    T7CONbits.TCS = 0;
    T7CONbits.TGATE = 0;
    T7CONbits.TCKPS = 0b0; // Select 1:1 Prescaler
    TMR7 = 0x00;
    PR7 = 4662; // 15015 Hz
    //  PR7 = 15000;
    IPC12bits.T7IP = 0x01;
    IFS3bits.T7IF = 0;
    IEC3bits.T7IE = 1;
    T7CONbits.TON = 1;

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

    initInfo.TimersInited = 1;
}

void CNInit(void) {
    //Set up Change Notify Interrupt
    CNENCbits.CNIEC14 = 1; // Enable RC14 pin for interrupt detection
    CNENCbits.CNIEC13 = 1; // Enable RC13
    CNENDbits.CNIED0 = 1;

    IEC1bits.CNIE = 1; // Enable CN interrupts
    IPC4bits.CNIP = 0x01;
    IFS1bits.CNIF = 0; // Reset CN interrupt
}

void QEIInit(void) {
#ifdef SINE
    Open32bitQEI1(QEI_COUNTER_QEI_MODE &
            QEI_GATE_DISABLE &
            QEI_COUNT_POSITIVE &
            QEI_INPUT_PRESCALE_1 &
            QEI_INDEX_MATCH_NO_EFFECT &
            QEI_POS_COUNT_INIT_No_EFFECT &
            QEI_IDLE_CON &
            QEI_COUNTER_ENABLE,

            QEI_QEA_POL_NON_INVERTED &
            QEI_QEB_POL_NON_INVERTED &
            QEI_INDX_POL_NON_INVERTED &
            QEI_HOM_POL_NON_INVERTED &
            QEI_QEA_QEB_NOT_SWAPPED &
            QEI_COMPARE_HIGH_OUTPUT_DISABLE &
            QEI_DIF_FLTR_PRESCALE_1 &
            QEI_DIG_FLTR_DISABLE &
            QEI_POS_COUNT_TRIG_DISABLE,

            QEI_INDEX_INTERRUPT_DISABLE &
            QEI_HOME_INTERRUPT_DISABLE &
            QEI_VELO_OVERFLOW_INTERRUPT_DISABLE &
            QEI_POS_INIT_INTERRUPT_DISABLE &
            QEI_POS_OVERFLOW_INTERRUPT_DISABLE &
            QEI_POS_LESS_EQU_INTERRUPT_ENABLE &
            QEI_POS_GREAT_EQU_INTERRUPT_ENABLE);

    ConfigInt32bitQEI1(QEI_INT_PRI_4 & QEI_INT_ENABLE);
#else
    /* Configure QEICON, QEIIOC and QEISTAT register */
    Open32bitQEI1(QEI_COUNTER_QEI_MODE &
            QEI_GATE_DISABLE &
            QEI_COUNT_POSITIVE &
            QEI_INPUT_PRESCALE_1 &
            QEI_INDEX_MATCH_NO_EFFECT &
            QEI_POS_COUNT_INIT_No_EFFECT &
            QEI_IDLE_CON &
            QEI_COUNTER_ENABLE,

            QEI_QEA_POL_NON_INVERTED &
            QEI_QEB_POL_NON_INVERTED &
            QEI_INDX_POL_NON_INVERTED &
            QEI_HOM_POL_NON_INVERTED &
            QEI_QEA_QEB_NOT_SWAPPED &
            QEI_COMPARE_HIGH_OUTPUT_DISABLE &
            QEI_DIF_FLTR_PRESCALE_8 &
            QEI_DIG_FLTR_DISABLE &
            QEI_POS_COUNT_TRIG_DISABLE,

            QEI_INDEX_INTERRUPT_ENABLE &
            QEI_HOME_INTERRUPT_ENABLE &
            QEI_VELO_OVERFLOW_INTERRUPT_DISABLE &
            QEI_POS_INIT_INTERRUPT_ENABLE &
            QEI_POS_OVERFLOW_INTERRUPT_ENABLE &
            QEI_POS_LESS_EQU_INTERRUPT_DISABLE &
            QEI_POS_GREAT_EQU_INTERRUPT_DISABLE);

    ConfigInt32bitQEI1(QEI_INT_PRI_4 & QEI_INT_DISABLE);
#endif

    QEI1GECL = 2047;
    QEI1GECH = 0;

    QEI1LECL = 0b1111100000000001;
    QEI1LECH = 0b1111111111111111;

}

void ADCInit(void) {
    ANSELBbits.ANSB1 = 1; //AN1
    ANSELBbits.ANSB12 = 1; //AN12
    ANSELBbits.ANSB13 = 1; //AN13

    //Setup ADC1 for Channel 0-3 sampling
    AD1CON1bits.FORM = 0; //Data Output Format : Integer Output
    AD1CON1bits.SSRCG = 1; //
    AD1CON1bits.SSRC = 0; //Sample Clock Source : PWM Generator 1 primary trigger compare ends sampling
    //AD1CON1bits.SSRC = 4; //Sample Clock Source : GP Timer5 starts conversion
    AD1CON1bits.ASAM = 1; // Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 1; // 12-bit ADC operation
    AD1CON1bits.SIMSAM = 0; // Samples channel 0;
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.CSCNA = 1; // Scan CH0+ Input Selections during Sample A bit
    AD1CON2bits.CHPS = 0; // Converts CH0  //This got changed
    AD1CON3bits.ADRC = 0; // ADC clock is derived from systems clock

    /*
     * ADCS is the main clock multiplier.  The result should be >= 1.6uS.
     * In a 40MHz processor the base period is 25nS.  64 * 25nS = 1.6uS.
     */
    AD1CON3bits.ADCS = 112; // 112 * 14.3nS = 1.6uS -- Conversion Clock
    AD1CON4bits.ADDMAEN = 1; // DMA Enable

    //AD1CHS0: Analog-to-Digital Input Select Register
    AD1CHS0bits.CH0SA = 1; // MUXA +ve input selection (AIN0) for CH0
    AD1CHS0bits.CH0NA = 0; // MUXA -ve input selection (VREF-) for CH0
    //AD1CHS123: Analog-to-Digital Input Select Register
    AD1CHS123bits.CH123SA = 1; // MUXA +ve input selection (AIN0) for CH1
    AD1CHS123bits.CH123NA = 0; // MUXA -ve input selection (VREF-) for CH1

    //AD1CSSH/AD1CSSL: Analog-to-Digital Input Scan Selection Register
    AD1CSSH = 0x0000;
    //AD1CSSLbits.CSS1 = 1;  //PVDD MONITORING
    AD1CSSLbits.CSS12 = 1;
    AD1CSSLbits.CSS13 = 1;

    AD1CON1bits.ADDMABM = 0; // DMA buffers are built in scatter/gather mode
    AD1CON2bits.SMPI = 1; // 2 ADC buffers
    AD1CON4bits.DMABL = 0; // Allocate one word of buffer per input.
    IFS0bits.AD1IF = 0; // Clear Analog-to-Digital Interrupt Flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable Analog-to-Digital interrupt
    AD1CON1bits.ADON = 1; // Turn on the ADC
}

void EventCheckInit(void *eventCallback) {
    eventCallbackFcn = eventCallback;

    initInfo.EventCheckInited = 1;
}

void MagEncoderInit(void) {
    IFS0bits.IC1IF = 0; // Clear the IC1 interrupt status flag
    IEC0bits.IC1IE = 1; // Enable IC1 interrupts
    IPC0bits.IC1IP = 1; // Set module interrupt priority as 2


    //IC1CON1bits.ICTSEL = 0b111; // Peripheral (FP) is the clock source for the IC1 module

    IC1CON1bits.ICTSEL = 0b001; //timer 2 is source
    IC1CON1bits.ICI = 0b00; // event on every capture
    IC1CON1bits.ICM = 0b001; // edge detect mode

    IC1CON2bits.IC32 = 0; // Cascade module operation is disabled
    IC1CON2bits.ICTRIG = 1;
    //IC1CON2bits.TRIGSTAT = 0;
    IC1CON2bits.SYNCSEL = 0b00000; // No Sync or Trigger source for the IC1 module

    //setup timer2 for IC1
    T2CONbits.TON = 0;
    T2CONbits.TCS = 0; //internal clock, fosc/2
    T2CONbits.T32 = 0;
    T2CONbits.TSIDL = 0;
    T2CONbits.TCKPS = 0b01; // 1:8 prescaler

    TMR2 = 0x0000;
    PR2 = 0xffff;
    T2CONbits.TON = 1;


}

uint32_t getEncDC(void) {
    return encDC;
}

uint16_t getCaptureReading(void) {
    return captureReading;
}


//sets up a timer whose interrupts are used to call PrintWithTimestamp and spit
//out desired data such as encoder position

void SamplerInit(void) {
    T8CONbits.TON = 0;
    T8CONbits.TCS = 0;
    T8CONbits.TGATE = 0;
    T8CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
    TMR8 = 0x00;
    PR8 = 1000; // just under 1 kHz
    //  PR7 = 15000;
    IPC12bits.T8IP = 0x01;
    IFS3bits.T8IF = 0;
    IEC3bits.T8IE = 1;
    T8CONbits.TON = 1;
}

void __attribute__((__interrupt__, no_auto_psv)) _T8Interrupt(void) {
    LED1 ^= 1;
    IFS3bits.T8IF = 0;
    static uint16_t out[10];
    out[0] = getEncDC();
    PrintWithTimestamp(out, 1);
}

void PrintWithTimestamp(float *toSend, uint16_t size) {
    static float buff1[1001];
    static float buff2[1001];
    static uint16_t sb1 = 0;
    static uint16_t sb2 = 0;
    static uint16_t currbuff = 1;
    int i;

    if (currbuff == 1) {
        if (sb1 + size + 1 <= 1000) {

            for (i = 0; i < size; i++) {
                buff1[sb1 + i] = toSend[i];
            }

            buff1[sb1 + size] = (float) TMR6; //TIMESTAMP
            sb1 = sb1 + size + 1;
        } else {
            // LED3 = 1;
            DMA0_UART2_Transfer((sb1)*4, (uint8_t*) buff1);
            sb1 = 0;
            currbuff = 2;


            for (i = 0; i < size; i++) {
                buff2[sb2 + i] = toSend[i];
            }
            buff2[sb2 + size] = (float) TMR6; //TIMESTAMP
            sb2 = sb2 + size + 1;
        }
    } else {

        if (sb2 + size + 1 <= 1000) {

            for (i = 0; i < size; i++) {
                buff2[sb2 + i] = toSend[i];
            }
            buff2[sb2 + size] = (float) TMR6; //TIMESTAMP
            sb2 = sb2 + size + 1;
        } else {
            LED2 = 1;
            DMA0_UART2_Transfer((sb2)*4, (uint8_t*) buff2);
            sb2 = 0;
            currbuff = 1;


            for (i = 0; i < size; i++) {
                buff1[sb1 + i] = toSend[i];
            }
            buff1[sb1 + size] = (float) TMR6; //TIMESTAMP
            sb1 = sb1 + size + 1;
        }
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void) {
    //LED2=ENC;
    IFS0bits.IC1IF = 0; // Reset respective interrupt flag
    if (ENC) { // if transition to high
        LED2 = 1;
        Capture1 = TMR2;
        TMR2 = 0x0000;


        encDC = (Capture2 << 16) / Capture1;
    } else { // transition to low
        LED2 = 0;
        //        LED3 = 1;

        Capture2 = TMR2;
    }
}

#ifndef UART_TEST

void __attribute__((__interrupt__, no_auto_psv)) _T7Interrupt(void) {
    eventCallbackFcn();
    IFS3bits.T7IF = 0; // Clear Timer1 Interrupt Flag
}
#endif

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    /* Interrupt Service Routine code goes here */
    IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
}
