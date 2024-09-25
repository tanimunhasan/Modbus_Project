#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "uart.h"
// Main function
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer
    configureClocks();         // Configure clocks
    configureUART();           // Configure UART
    configureTimer();          // Configure Timer for Modbus timeout
    configureRS485();          // Configure RS485 transceiver
    UARTRxTx();                // Set up UART Rx and Tx pins
    configureLED();
    __enable_interrupt();      // Enable global interrupts

    while (1) {
        switch (currentState) {
            case STATE_IDLE:
                //Can write function for background tasks like health checks, loggin, or energy-saving
                break;

            case STATE_PROCESS_FRAME:
                if (isValidFrame((const char *)sendBuffer, bufferIndex)) {
                    processModbusFrame((const char *)sendBuffer, bufferIndex);  // Process valid frame
                    currentState = STATE_TRANSMIT_BUFFER;  // Move to transmit state
                } else {
                    currentState = STATE_IDLE;  // Frame was invalid, go back to idle
                }
                break;

            case STATE_TRANSMIT_BUFFER:
                uartTransmitBufferHex((const char *)sendBuffer, bufferIndex);  // Transmit buffer as hex
                toggleLED();
                currentState = STATE_IDLE;  // Go back to idle state after transmission
                break;

            default:
                break;
        }
    }
}
