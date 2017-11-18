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

volatile uint8_t pc_rx_buf[8];
volatile uint8_t pc_tx_cnt = 0;
volatile uint8_t pc_tx_buf[8];
volatile uint8_t pc_flag = 0;
volatile uint8_t pc_rx_cnt = 0;

void pc_start_tx(uint8_t cnt) {
	uca2_write();
	pc_tx_cnt = cnt;
	UCA2TXBUF = pc_tx_buf[0];
}

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
		active = index;
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
                                  // P2.6 - Green LED

    uca2_read();

    __bis_SR_register(GIE);       // interrupts enabled
    while (1) {
    	{
 			UCA0IE &= ~UCRXIE;        // Disable USCI_A0 RX interrupt (Switch A)
			uint8_t zero = switchA_r[0];
			uint8_t warm = switchA_r[4];
			warm = switchA_r[4];
			uint8_t cold = switchA_r[3];
			uint8_t bra  = switchA_r[7];
			UCA0IE |= UCRXIE;         // Enable USCI_A0 RX interrupt (Switch A)
			if (zero == 0) {
				processSwitch(cold, warm, bra, 0);
			}
    	}
    	{
			UCA1IE &= ~UCRXIE;        // Disable USCI_A1 RX interrupt (Switch B)
			uint8_t zero = switchB_r[0];
			uint8_t cold = switchB_r[3];
			cold = switchB_r[3];
			uint8_t warm = switchB_r[4];
			uint8_t bra  = switchB_r[7];
			UCA1IE |= UCRXIE;         // Enable USCI_A1 RX interrupt (Switch B)
			if (zero == 0) {
				processSwitch(cold, warm, bra, 1);
			}
    	}
    	if (pc_flag) {
			UCA2IE &= ~UCRXIE;        // Disable USCI_A2 RX interrupt (PC)
			pc_flag = 0;
			uint8_t cmd = pc_rx_buf[0];
			uint8_t cold = pc_rx_buf[1];
			uint8_t warm = pc_rx_buf[2];
			uint8_t bra = pc_rx_buf[3];
			UCA2IE |= UCRXIE;         // Enable USCI_A2 RX interrupt (PC)
			switch (cmd) {
			case 1:
				processSwitch(cold, warm, bra, 2);
				pc_tx_buf[0] = cmd;
				pc_tx_buf[1] = 0;
				pc_start_tx(2);
				break;
			case 2:
				pc_tx_buf[0] = cmd;
				pc_tx_buf[1] = 0;
				pc_tx_buf[2] = active;
				pc_start_tx(3);
				break;
			}
    	}
    }
}

#define SWAP(x,y) do {   \
	volatile uint8_t * _x = x;    \
	x = y;               \
	y = _x;              \
} while(0)

// USCI_A0 - Switch A read, Dimmer write
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
	static uint16_t switchA_tx_cnt = 0, switchA_rx_cnt = 0;
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
        		switchA_rx_cnt = 0;
        	}
        	read_time = time;
        	if (switchA_rx_cnt < 8) {
        		switchA_w[switchA_rx_cnt++] = x;
        	} else {
        		++switchA_rx_cnt;
        	}
        	if (8 == switchA_rx_cnt) {
        		SWAP(switchA_w, switchA_r);
        	}
        }
        break;
        case USCI_UART_UCTXIFG:	            // TXIFG
        	if (!get_green_state())
        		break;
        	++switchA_tx_cnt;
        	switch (switchA_tx_cnt) {
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
        	if (switchA_tx_cnt == 17) {      // TX over?
        		green_led_off();
        		switchA_tx_cnt = 0;
        	}
        	break;
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default:
        	break;
    }
}

// USCI_A1 - Switch B read
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	static uint16_t switchB_rx_cnt = 0;
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
        		switchB_rx_cnt = 0;
        	}
        	read_time = time;
        	if (switchB_rx_cnt < 8) {
        		switchB_w[switchB_rx_cnt++] = x;
        	} else {
        		++switchB_rx_cnt;
        	}
        	if (8 == switchB_rx_cnt) {
        		SWAP(switchB_w, switchB_r);
        	}
        }
        break;
        case USCI_UART_UCTXIFG:	break;      // TXIFG
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

static volatile uint16_t pc_read_time = 0;
static volatile uint16_t switch_to_read_time = 0;

// USCI_A2 - PC r\w
#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
	static uint8_t tx_cnt = 1;
    switch (__even_in_range(UCA2IV, 4))
    {
        case USCI_NONE: break;              // No interrupt
        case USCI_UART_UCRXIFG:             // RXIFG
        {
        	uint8_t x = UCA2RXBUF;
        	pc_read_time = time;
        	if (pc_rx_cnt < 8) {
        		pc_rx_buf[pc_rx_cnt++] = x;
        	}
        }
		break;
        case USCI_UART_UCTXIFG:	            // TXIFG
        {
        	if (tx_cnt < pc_tx_cnt) {
				UCA2TXBUF = pc_tx_buf[tx_cnt++];
        	} else {
        		tx_cnt = 1;
        		switch_to_read_time = time + 20;
        	}
        }
		break;
        case USCI_UART_UCSTTIFG: break;     // TTIFG
        case USCI_UART_UCTXCPTIFG: break;   // TXCPTIFG
        default: break;
    }
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
	++time;
	if (pc_rx_cnt && (time - pc_read_time > 15)) {
		pc_rx_cnt = 0;
		pc_flag = 1;
	}
	if (time == switch_to_read_time) {
		uca2_read();
	}
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
}
