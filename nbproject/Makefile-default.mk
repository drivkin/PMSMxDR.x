#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=PMSM_Characterize.c main.c SPIdsPIC.c DRV8301.c PMSMBoard.c CircularBuffer.c DMA_Transfer.c PMSM_Velocity.c PMSM_Position.c C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/SVM_Torque_Control.c C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/EstimatorFixedPoint.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/PMSM_Characterize.o ${OBJECTDIR}/main.o ${OBJECTDIR}/SPIdsPIC.o ${OBJECTDIR}/DRV8301.o ${OBJECTDIR}/PMSMBoard.o ${OBJECTDIR}/CircularBuffer.o ${OBJECTDIR}/DMA_Transfer.o ${OBJECTDIR}/PMSM_Velocity.o ${OBJECTDIR}/PMSM_Position.o ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o
POSSIBLE_DEPFILES=${OBJECTDIR}/PMSM_Characterize.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/SPIdsPIC.o.d ${OBJECTDIR}/DRV8301.o.d ${OBJECTDIR}/PMSMBoard.o.d ${OBJECTDIR}/CircularBuffer.o.d ${OBJECTDIR}/DMA_Transfer.o.d ${OBJECTDIR}/PMSM_Velocity.o.d ${OBJECTDIR}/PMSM_Position.o.d ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o.d ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/PMSM_Characterize.o ${OBJECTDIR}/main.o ${OBJECTDIR}/SPIdsPIC.o ${OBJECTDIR}/DRV8301.o ${OBJECTDIR}/PMSMBoard.o ${OBJECTDIR}/CircularBuffer.o ${OBJECTDIR}/DMA_Transfer.o ${OBJECTDIR}/PMSM_Velocity.o ${OBJECTDIR}/PMSM_Position.o ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o

# Source Files
SOURCEFILES=PMSM_Characterize.c main.c SPIdsPIC.c DRV8301.c PMSMBoard.c CircularBuffer.c DMA_Transfer.c PMSM_Velocity.c PMSM_Position.c C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/SVM_Torque_Control.c C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/EstimatorFixedPoint.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP256MU806
MP_LINKER_FILE_OPTION=,--script=p33EP256MU806.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/PMSM_Characterize.o: PMSM_Characterize.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSM_Characterize.o.d 
	@${RM} ${OBJECTDIR}/PMSM_Characterize.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSM_Characterize.c  -o ${OBJECTDIR}/PMSM_Characterize.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSM_Characterize.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSM_Characterize.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SPIdsPIC.o: SPIdsPIC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/SPIdsPIC.o.d 
	@${RM} ${OBJECTDIR}/SPIdsPIC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SPIdsPIC.c  -o ${OBJECTDIR}/SPIdsPIC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SPIdsPIC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/SPIdsPIC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/DRV8301.o: DRV8301.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/DRV8301.o.d 
	@${RM} ${OBJECTDIR}/DRV8301.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  DRV8301.c  -o ${OBJECTDIR}/DRV8301.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DRV8301.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/DRV8301.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PMSMBoard.o: PMSMBoard.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSMBoard.o.d 
	@${RM} ${OBJECTDIR}/PMSMBoard.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSMBoard.c  -o ${OBJECTDIR}/PMSMBoard.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSMBoard.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSMBoard.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/CircularBuffer.o: CircularBuffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/CircularBuffer.o.d 
	@${RM} ${OBJECTDIR}/CircularBuffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CircularBuffer.c  -o ${OBJECTDIR}/CircularBuffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/CircularBuffer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/CircularBuffer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/DMA_Transfer.o: DMA_Transfer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/DMA_Transfer.o.d 
	@${RM} ${OBJECTDIR}/DMA_Transfer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  DMA_Transfer.c  -o ${OBJECTDIR}/DMA_Transfer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DMA_Transfer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/DMA_Transfer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PMSM_Velocity.o: PMSM_Velocity.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSM_Velocity.o.d 
	@${RM} ${OBJECTDIR}/PMSM_Velocity.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSM_Velocity.c  -o ${OBJECTDIR}/PMSM_Velocity.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSM_Velocity.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSM_Velocity.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PMSM_Position.o: PMSM_Position.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSM_Position.o.d 
	@${RM} ${OBJECTDIR}/PMSM_Position.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSM_Position.c  -o ${OBJECTDIR}/PMSM_Position.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSM_Position.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSM_Position.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o: C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/SVM_Torque_Control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1303724136 
	@${RM} ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/SVM_Torque_Control.c  -o ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o: C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/EstimatorFixedPoint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1303724136 
	@${RM} ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/EstimatorFixedPoint.c  -o ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/PMSM_Characterize.o: PMSM_Characterize.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSM_Characterize.o.d 
	@${RM} ${OBJECTDIR}/PMSM_Characterize.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSM_Characterize.c  -o ${OBJECTDIR}/PMSM_Characterize.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSM_Characterize.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSM_Characterize.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SPIdsPIC.o: SPIdsPIC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/SPIdsPIC.o.d 
	@${RM} ${OBJECTDIR}/SPIdsPIC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SPIdsPIC.c  -o ${OBJECTDIR}/SPIdsPIC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SPIdsPIC.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/SPIdsPIC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/DRV8301.o: DRV8301.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/DRV8301.o.d 
	@${RM} ${OBJECTDIR}/DRV8301.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  DRV8301.c  -o ${OBJECTDIR}/DRV8301.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DRV8301.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/DRV8301.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PMSMBoard.o: PMSMBoard.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSMBoard.o.d 
	@${RM} ${OBJECTDIR}/PMSMBoard.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSMBoard.c  -o ${OBJECTDIR}/PMSMBoard.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSMBoard.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSMBoard.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/CircularBuffer.o: CircularBuffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/CircularBuffer.o.d 
	@${RM} ${OBJECTDIR}/CircularBuffer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CircularBuffer.c  -o ${OBJECTDIR}/CircularBuffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/CircularBuffer.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/CircularBuffer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/DMA_Transfer.o: DMA_Transfer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/DMA_Transfer.o.d 
	@${RM} ${OBJECTDIR}/DMA_Transfer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  DMA_Transfer.c  -o ${OBJECTDIR}/DMA_Transfer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DMA_Transfer.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/DMA_Transfer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PMSM_Velocity.o: PMSM_Velocity.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSM_Velocity.o.d 
	@${RM} ${OBJECTDIR}/PMSM_Velocity.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSM_Velocity.c  -o ${OBJECTDIR}/PMSM_Velocity.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSM_Velocity.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSM_Velocity.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PMSM_Position.o: PMSM_Position.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/PMSM_Position.o.d 
	@${RM} ${OBJECTDIR}/PMSM_Position.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PMSM_Position.c  -o ${OBJECTDIR}/PMSM_Position.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PMSM_Position.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/PMSM_Position.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o: C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/SVM_Torque_Control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1303724136 
	@${RM} ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/SVM_Torque_Control.c  -o ${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1303724136/SVM_Torque_Control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o: C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/EstimatorFixedPoint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1303724136 
	@${RM} ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/Dmitriy/Documents/GitHub/PMSMxDR.X/EstimatorFixedPoint.c  -o ${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -menable-large-arrays -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1303724136/EstimatorFixedPoint.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf  -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--heap=2096,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,-lq$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=2096,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,-lq$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/PMSMxDR.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
