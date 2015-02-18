/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavlo Milo Manovi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file	DMA_Transfer.h
 * @author 	Pavlo Manovi
 * @date	July 18th, 2014, 3:49 PM
 * @brief 	This library provides methods of interacting with peripherals using DMA.
 *
 * NOTE: DMA4 and DMA5 are merely placeholders for a CAN implementation.  On the SUPER-Ball
 * Bot, DMA for the eCAN module is handled in the CAN festival libraries. 
 */


#ifndef DMA_TRANSFER_H
#define	DMA_TRANSFER_H

#include "CircularBuffer.h"

/**
 *@brief Struct which contains ADC data.
 */
typedef struct {
	uint16_t Adc1Data[128];
        uint16_t newData;
} ADCBuffer;

/**
 *@brief Transmits data over UART.
 *@param size Number of bytes to send.
 *@param SendBuffer Pointer to array of bytes to send over UART.
 * 
 * No initialization other than UART is required to call this function.
 * If calling in the same scope as the send buffer, make sure that the
 * send buffer is static or not contained only within the cpu stack.
 */
void DMA0_UART2_Transfer(uint16_t size, uint8_t *SendBuffer);

/**
 * @brief Handles UART reception and places data into a circular buffer.
 * @param cB Pointer to a CircularBuffer which will contain RX data.
 */
void DMA1_UART2_Enable_RX(CircularBuffer *cB);

/**
 * @brief Handles SPI transmission of data.
 * @param size Number of words to send.
 * @param SendBuffer Pointer to an array of words to send over UART.
 *
 * Note that DMA and SPI on the dsPIC33E are not heavily integrated.
 * This SPI transfer method should be called by a separate SPI library.
 */
void DMA2_SPI_Transfer(uint16_t size, uint16_t *SendBuffer);

/**
 * @brief Handles SPI reception of data.
 * @param cB Pointer to a CircularBuffer which will contain RX data.
 */ 
void DMA3_SPI_Enable_RX(CircularBuffer *cB);

/**
 * @brief Placeholder, not used with the SUPER-Ball bot.
 */
void DMA4_CAN_Transfer(uint16_t size, uint16_t *SendBuffer);

/**
 * @brief Placeholder, not used with the SUPER-Ball bot.
 */
void DMA5_CAN_Enable_RX(CircularBuffer *cB);

/**
 * @brief Sets up the DMA transfers between the ADC and a buffer.
 * @param ADCBuff Pointer to an ADCBuffer.
 */
void DMA6_ADC_Enable(ADCBuffer *ADCBuff);

#endif	/* DMA_TRANSFER_H */

