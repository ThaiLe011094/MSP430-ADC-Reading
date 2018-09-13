#include <msp430g2452.h>
#include <stdbool.h>

#define             TXD BIT1
#define             RXD BIT2
#define             ADC_CHANNELS    5                       // We will sample 5 channels

#define             Bit_time        103                     // 9600 Baud, SMCLK=MHz (1Mhz/9600 = 104)
#define             Bit_time_5      52                      // Time for half a bit

//ASCII values for the commands
#define             TEST_SPEED      0x31
#define             M_A3            0x32
#define             M_PULSE         0x35

unsigned char   BitCnt;                                 // Bit count, used when transmitting byte
unsigned int    TXByte;                                 // Value sent over UART when Transmit() is called
unsigned int    RXByte;                                 // Value recieved once hasRecieved is set

unsigned int    i;

unsigned int        samples[ADC_CHANNELS];
unsigned int        value=0;

bool                isReceiving;
bool                hasReceived;

bool                ADCDone;                                                                    // ADC Flag
unsigned            ADCValue;                                                                   // Measure ADC

/*
 * main.c
 */

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                                                                   // Stop Watch Dog Timer, Watch Dog Timer Power OR Watch Dog Timer Hold

    BCSCTL1 = CALBC1_1MHZ;                                                                      // Set range for MCU
    DCOCTL = CALDCO_1MHZ;                                                                       // SMCLK (System Machine Clock) = DCO = 1Mhz;

    P1SEL |=  TXD;                                                                              // Connected TXD to timer pin .. TXD on P1.1 .. mean as P1SEL |= 0x01
    P1DIR |=  TXD;                                                                              // Set P1.1 to input

    P1IES |=  RXD;                                                                              // RXD High/Low edge interrupt .. RXD on P1.2 .. mean as P1IES |= 0x10
    P1IFG &= ~RXD;                                                                              // Clear RXD flag before enabling interrupt
    P1IE  |=  RXD;                                                                              // Enable RXD interrupt
    P1DIR |=  BIT0;                                                                             // Set P1.0 to input -> Receive Data from P1.0
    P1OUT &= ~BIT0;                                                                             // Turn off LED at P1.0

    isReceiving = false;                                                                        // Set initial values
    hasReceived = false;
    ADCDone     = false;

    __bis_SR_register(GIE);                                                                     // interrupts enabled

    while(1)
    {
            Measure();
            if (hasReceived)                                                                    // If the device receives a value
            {
                Receive();
            }
            if (ADCDone)
            {
                ADCDone = false;                                                                // Clear flag
                TXByte = ADCValue & 0x00FF;                                                     // Set TXByte
                Transmit();
                TXByte = (ADCValue >> 0);                                                       // Set TXByte to the upper 8 bits
                TXByte = TXByte & 0x00FF;
                Transmit();
            }
            Measure();
            if (~(true && ADCDone))                                                             // Loop again if either flag is set
                __bis_SR_register(CPUOFF + GIE);                                                // LPM0. the ADC interrupt will wake the processor up

    }
}

void Receive()
{
        hasReceived = false;                                                                    // Clear the flag
        switch(RXByte)                                                                          // Switch depending on command value received
        {
        case TEST_SPEED:
            P1OUT |= BIT0;                                                                      // Turn on LED while testing
            for (i=0; i != 0x100; i++)                                                          // Loop 256 times
            {
                TXByte = i;                                                                     // Sends the counter as if it were a 1 bit value
                Transmit();
                TXByte = 0;
                Transmit();
            }
            P1OUT &= ~BIT0;                                                                     // Turn off the LED
            break;

        case M_PULSE:
            Measure(INCH_10);                                                                   // Reads data from Pulse sensor
            break;

        default:;
        }
}

void Measure(unsigned int chan)                                                                 // "chan" mean as Channel
{
        ADC10CTL0 &= ~ENC;                                                                      // Disable ADC
        ADC10CTL0 = ADC10SHT_3 + ADC10ON + ADC10IE;                                             // 16 clock ticks, ADC On, enable ADC interrupt
        ADC10CTL1 = ADC10SSEL_3 + chan;                                                         // Set 'chan', SMCLK
        ADC10CTL0 |= ENC + ADC10SC;                                                             // Enable and start conversion
}

void Transmit()
{
        while(isReceiving);                                                                     // Wait for RX completion
        TXByte |= 0x100;                               // Add stop bit to TXByte (which is logical 1)
        TXByte = TXByte << 1;                          // Add start bit (which is logical 0)
        BitCnt = 0xA;                                  // Load Bit counter, 8 bits + ST/SP

        CCTL0 = OUT;                                   // TXD Idle as Mark
        TACTL = TASSEL_2 + MC_2;                       // SMCLK, continuous mode
        CCR0 = TAR;                                    // Initialize compare register
        CCR0 += Bit_time;                              // Set time till first bit
        CCTL0 = CCIS0 + OUTMOD0 + CCIE;                // Set signal, initial value, enable interrupts
        while (CCTL0 & CCIE);                          // Wait for previous TX completion
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
        ADCValue = ADC10MEM;                           // Saves measured value
        ADCDone = true;                                // Sets flag for main loop
        __bic_SR_register_on_exit(CPUOFF);             // Enable CPU so the main while loop continues
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
        isReceiving = true;
        P1IE &= ~RXD;                                   // Disable RXD interrupt
        P1IFG &= ~RXD;                                  // Clear RXD IFG (interrupt flag)
        TACTL = TASSEL_2 + MC_2;                        // SMCLK, continuous mode
        CCR0 = OUTMOD1 + CCIE;                          // Disable TX and enable interrupts
        RXByte = 0;                                     // Initialize RXByte
        BitCnt = 0x9;                                   // Load Bit counter. 8 bits + ST
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
        if(!isReceiving)
        {
            CCR0 += Bit_time;                           // Add Offset to CCR0
            if (BitCnt == 0)                            // If all bits TXed
            {
                TACTL = TASSEL_2;                       // SMCLK, timer off (for power consumption)
                CCTL0 &= ~CCIE;                         // Disable interrupt
            }
            else
            {
                CCTL0 |= OUTMOD2;                       // Set TX bit to 0
                if (TXByte & 0x01)
                    CCTL0 &= ~OUTMOD2;                  // If it should be 1, set it to 1
                TXByte = TXByte >> 1;
                BitCnt --;
            }
        }
        else
        {
            CCR0 += Bit_time;                           // Add Offset to CCR0;
            if (BitCnt == 0)
            {
                TACTL = TASSEL_2;                       // SMCLK, timer off (for power consumption)
                CCTL0 &= ~CCIE;                         // Disable interrupt
                isReceiving = false;
                P1IFG &= ~RXD;                          // Clear RXD IFG (interrupt flag)
                P1IE |= RXD;                            // Enable RXD interrupt
                if ((RXByte & 0x201) == 0x200)          // Validate the start and stop bits are correct
                {
                    RXByte = RXByte >> 1;               // Remove start bit
                    RXByte &= 0xFF;                     // Remove stop bit
                    hasReceived = true;
                }
                __bic_SR_register_on_exit(CPUOFF);      // Enable CPU on the main while loop continues
            }
            else
            {
                if ((P1IN & RXD) == RXD)                // If bit is set?
                    RXByte |= 0x400;                    // Set the value in the RXByte
                RXByte = RXByte >> 1;                   // Shift the bit down
                BitCnt --;
            }
        }
}
