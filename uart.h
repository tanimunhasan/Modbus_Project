/*
 * uart.h
 *
 *  Created on: Sep 25, 2024
 *      Author: Sayed Tanimun Hasan
 */

#ifndef UART_H_
#define UART_H_

// Function to configure UART for communication (19200 baud, 8N1)
void configureClocks(void) {
    CSCTL0_H = CSKEY >> 8;             // Unlock CS registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;      // Set DCO to 8MHz
    CSCTL3 = DIVS__1;                  // Set SMCLK divider to 1
    CSCTL0_H = 0;                      // Lock CS registers
}

// Function to configure UART
void configureUART() {
    UCA1CTL1 |= UCSWRST;                    // Put eUSCI in reset
    UCA1CTL1 |= UCSSEL_2;                   // SMCLK as clock source
    UCA1BRW = 26;                           // Baud rate 19200 for 8MHz clock
    UCA1MCTLW = (0x54 << 8) | (0x00 << 4) | UCOS16; // Modulation oversampling
    PM5CTL0 &= ~LOCKLPM5;                   // Turn on I/O
    UCA1CTL1 &= ~UCSWRST;                   // Initialize eUSCI
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt
}


#endif /* UART_H_ */
