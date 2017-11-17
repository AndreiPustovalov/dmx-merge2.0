#include <msp430.h>
#include <stdint.h>

#include "driverlib.h"
#include "periphery.h"

volatile uint8_t switchA_r_buf[8] = { 0xff };
volatile uint8_t switchB_r_buf[8] = { 0xff };
volatile uint8_t *switchA_r = switchA_r_buf;
volatile uint8_t *switchB_r = switchB_r_buf;

volatile uint16_t time = 0;

volatile uint8_t currentWarm[3] = { 0x9F };
volatile uint8_t currentCold[3] = { 0x9f };
volatile uint8_t currentBra[3] = { 0 };
volatile uint8_t active = 0;

volatile uint8_t last[8] = {0,0,0,0,0,0,0,0};
volatile uint8_t last_reg[8];
volatile uint8_t r;
volatile uint8_t last_p = 0;

void processSwitch(uint8_t c, uint8_t w, uint8_t b, uint8_t index) {
	uint8_t change = 0;
	if (currentWarm[index] != w) {
		currentWarm[index] = w;
		change = 1;
	}
	if (currentCold[index] != c) {
		currentCold[index] = c;
		change = 1;
	}
	if (currentBra[index] != b) {
		currentBra[index] = b;
		change = 1;
	}
	if (change){
//		active = index;
		__no_operation();
	}
}

void main(void) {
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    setup_clock();

    init_uarts();

    setup_timers();

    P1DIR |= BIT6;                // Set pins to output. P1.6 - relay control
    P2DIR |= BIT4 | BIT5 | BIT6;  // Set pins to output.
                                  // P2.4 - UCA2 R/W
                                  // P2.5 - Yellow LED
                                  // P2,6 - Green LED

    __bis_SR_register(GIE);       // interrupts enabled
    while (1) {
    	{
 			UCA0IE &= ~UCRXIE;        // Disable USCI_A0 RX interrupt (Switch A)
			uint8_t zero = switchA_r[0];
			uint8_t warm = switchA_r[4];
			uint8_t cold = switchA_r[3];
			uint8_t bra  = switchA_r[7];
			asm (
				"        MOV.B     r13,&r+0"
			);
			last[last_p % 8] = 1;
			last_reg[last_p++ % 8] = r;
			UCA0IE |= UCRXIE;         // Enable USCI_A0 RX interrupt (Switch A)
			if (zero == 0) {
				last[last_p % 8] = 5;
				last_reg[last_p++ % 8] = r;
				processSwitch(cold, warm, bra, 0);
			}
			last[last_p % 8] = 4;
			last_reg[last_p++ % 8] = r;
    	}
//    	{
//			UCA1IE &= ~UCRXIE;        // Disable USCI_A1 RX interrupt (Switch B)
//			uint8_t zero = switchB_r[0];
//			uint8_t cold = switchB_r[3];
//			uint8_t warm = switchB_r[4];
//			uint8_t bra  = switchB_r[7];
//			UCA1IE |= UCRXIE;         // Enable USCI_A1 RX interrupt (Switch B)
//			if (zero == 0) {
//				processSwitch(cold, warm, bra, 1);
//			}
//    	}
    }
}

#define SWAP(x,y) do {   \
	volatile uint8_t * _x = x;      \
	x = y;                \
	y = _x;                \
} while(0)

// USCI_A0 - Switch A read, Dimmer write
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	asm (
		"        MOV.B     r13,&r+0"
	);
	last[last_p % 8] = 2;
	last_reg[last_p++ % 8] = r;
	static uint16_t switchA_w_cnt = 0, switchA_r_cnt = 0;
	static uint16_t read_time = 0;
	static volatile uint8_t switchA_w_buf[8];
	static volatile uint8_t *switchA_w = switchA_w_buf;

    switch (__even_in_range(UCA0IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	uint8_t x = UCA0RXBUF;
        	if (time - read_time >= 10) {
        		switchA_r_cnt = 0;
        	}
        	read_time = time;
        	if (switchA_r_cnt < 8) {
        		switchA_w[switchA_r_cnt++] = x;
        	} else {
        		++switchA_r_cnt;
        	}
        	if (8 == switchA_r_cnt) {
        		SWAP(switchA_w, switchA_r);
        	}
        }
        break;
        case USCI_UART_UCTXIFG:	            // TXIFG
        	if (!get_green_state())
        		break;
        	++switchA_w_cnt;
        	switch (switchA_w_cnt) {
        	case 1:
				UCA0TXBUF = currentCold[active];
        		break;
        	case 2:
				UCA0TXBUF = currentWarm[active];
        		break;
        	case 17:
        		break;
        	default:
        		UCA0TXBUF = 0;
        	}
        	if (switchA_w_cnt == 17) {      // TX over?
        		green_led_off();
        		switchA_w_cnt = 0;
        	}
        	break;
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default:
        	break;
    }
	asm (
		"        MOV.B     r13,&r+0"
	);
	last[last_p % 8] = 3;
	last_reg[last_p++ % 8] = r;
}

// USCI_A1 - Switch B read
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	static uint16_t switchB_r_cnt = 0;
	static uint16_t read_time = 0;
	static volatile uint8_t switchB_w_buf[8];
	static volatile uint8_t *switchB_w = switchB_w_buf;

	switch (__even_in_range(UCA1IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	uint8_t x = UCA1RXBUF;
        	if (time - read_time >= 10) {
        		switchB_r_cnt = 0;
        	}
        	read_time = time;
        	if (switchB_r_cnt < 8) {
        		switchB_w[switchB_r_cnt++] = x;
        	} else {
        		++switchB_r_cnt;
        	}
        	if (8 == switchB_r_cnt) {
        		SWAP(switchB_w, switchB_r);
        	}
        }
        break;
        case USCI_UART_UCTXIFG:	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
	last[last_p % 8] = 7;
}

// USCI_A2 - PC r\w
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
    switch (__even_in_range(UCA2IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG: break;      // RXIFG
        case USCI_UART_UCTXIFG:	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
	last[last_p % 8] = 8;
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
	++time;
	switch (time & 0x3f) {
	case 61:
		pin_down();
		break;
	case 62:
		set_uart_mode();
		break;
	case 63:
		green_led_on();
		UCA0TXBUF = 0;
		break;
	}
	last[last_p % 8] = 9;
}
