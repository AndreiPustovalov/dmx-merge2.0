/*
 *  ======= preiphery ========
 *
 *  Created on: 6 но€б. 2017 г.
 *  Author:     Andrew
 */

#include <msp430.h>
#include "driverlib.h"

#define THREE_BYTES 8000

void init_uarts()
{
    P1SEL |= BIT2 | BIT3 | BIT4;    	       // Set P1.2, P1.3, P1.4 to non-IO
    P1DIR |= BIT2 | BIT3 | BIT4;            // Enable UCA0RXD, UCA0TXD, UCA1RXD

    // Setup P2.2 UCA2RXD, P2.3 UCA2TXD
    P2SEL |= BIT2 | BIT3;                   // Set P2.2, P2.3 to non-IO
    P2DIR |= BIT2 | BIT3;                   // Enable UCA2RXD, UCA2TXD


    // Setup eUSCI_A0
    UCA0CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA0CTLW0 |= UCSSEL_2 | UCSPB;
    UCA0BRW_L = 0x40;                       // 250k
    UCA0BRW_H = 0x00;                       //
    UCA0MCTLW = 0x00;                       //
    UCA0CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**

    // Setup eUSCI_A1
    UCA1CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA1CTLW0 |= UCSSEL_2 | UCSPB;
    UCA1BRW_L = 0x40;                       // 250k
    UCA1BRW_H = 0x00;                       //
    UCA1MCTLW = 0x00;                       //
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**

    // Setup eUSCI_A2
    UCA2CTLW0 |= UCSWRST;                   // **Put state machine in reset**
    UCA2CTLW0 |= UCSSEL_2 | UCSPB;
    UCA2BRW_L = 0x40;                       // 250k
    UCA2BRW_H = 0x00;                       //
    UCA2MCTLW = 0x00;                       //
    UCA2CTLW0 &= ~UCSWRST;                  // **Initialize USCI state machine**

    UCA0IE |= UCRXIE | UCTXIE;              // Enable USCI_A0 RX TX interrupt
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt
    UCA2IE |= UCRXIE | UCTXIE;              // Enable USCI_A2 RX TX interrupt
}

void setup_clock() {
    //Set VCore = 2 for 16MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_2);

    //Set DCO FLL reference = REFO
    UCS_initClockSignal(
        UCS_FLLREF,
        UCS_REFOCLK_SELECT,
        UCS_CLOCK_DIVIDER_1
        );
    //Set ACLK = REFO
    UCS_initClockSignal(
        UCS_ACLK,
        UCS_REFOCLK_SELECT,
        UCS_CLOCK_DIVIDER_1
        );

    //Set Ratio and Desired MCLK Frequency  and initialize DCO
    UCS_initFLLSettle(
        16000,
        488
        );

    // Enable global oscillator fault flag
    SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
    SFR_enableInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
}

void setup_timers() {
    // Setup TA0
//    TA0CCTL0 = CCIE;                        // CCR0 interrupt enabled
//    TA0CCR0 = 32768;
//    TA0CTL = TASSEL_1 | MC_1 | TACLR;       // SMCLK, upmode, clear TAR

    // Setup TA1
    TA1CCTL0 = CCIE;                        // CCR0 interrupt enabled
    TA1CCR0 = THREE_BYTES;
    TA1CTL = TASSEL_2 | MC_1;       // SMCLK, upmode, clear TAR
}

#pragma vector=UNMI_VECTOR
__interrupt void NMI_ISR(void)
{
	static uint16_t status = STATUS_SUCCESS;
    do
    {
        // If it still can't clear the oscillator fault flags after the timeout,
        // trap and wait here.
        status = UCS_clearAllOscFlagsWithTimeout(1000);
    }
    while(status != 0);
}
