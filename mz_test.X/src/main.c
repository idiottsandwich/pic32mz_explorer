/************************************************************************
 * PIC32MZ2048EFH100 learning 
 * Written by Calvin Lin
 * 
 * Copyright (C) 2022
 * This code is intended to be a open source demo 
 * and is to be used for educational purposes only. 
 * Suggestions/constructive criticisms are welcome. 
 ************************************************************************
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * **********************************************************************/


// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FMIIEN = ON              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO = ON              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_1        // System PLL Input Divider (3x Divider)
#pragma config FPLLRNG = RANGE_5_10_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_FRC      // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT = MUL_100        // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (2x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS             // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (Communicate on PGEC2/PGED2)
#pragma config TRCEN = ON               // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3
#pragma config TSEQ = 0xFFFF            // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF            // Boot Flash Complement Sequence Number (Enter Hexadecimal value)


#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include <sys/attribs.h>
#include <sys/kmem.h>

#define SYS_FREQ 200000000UL

// DMA transfer buffer 
volatile unsigned char __attribute__((coherent)) ramBuffer[200];

void delay_us(unsigned int us) {
    // Convert microseconds us into how many clock ticks it will take
    us *= SYS_FREQ / 1000000 / 2; // Core Timer updates every 2 ticks

    _CP0_SET_COUNT(0); // Set Core Timer count to 0

    while (us > _CP0_GET_COUNT()); // Wait until Core Timer count reaches the number we calculated earlier
}

// ISR
void __ISR(_DMA4_VECTOR, IPL7SRS) DMA4_ISR(void)
{
    IFS4CLR = _IFS4_DMA4IF_MASK;
    // if finished, start again
 
    // if pattern matched, write out response
    
    U1TXREG = 'l';
}


// end ISR

char c;

int main(int argc, char** argv) 
{
          
    // configure PBCLOCK2
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    PB2DIVbits.PBDIV = 4;    // 40MHz PB2DIV
    PB2DIVbits.ON = 1;    
    SYSKEY = 0;
       
    // configure UART
    // PPS
    TRISFbits.TRISF4 = 1;
    TRISFbits.TRISF5 = 0;
    RPF5R = 1; // U1 TX
    U1RXR = 2; // U1 RX
    
    U1MODE = 0;
    U1STAbits.URXEN = 1;
    U1STAbits.UTXEN = 1;
    U1BRG = 9; //250k
    U1MODEbits.ON = 1;
    
    // configure UART interrupts
    //IPC28SET = 0x1700;
    // = _IEC3_U1RXIE_MASK;

    //configure DMA
    //IEC1CLR = 0x00010000; // disable DMA channel 0 interrupts
    //IFS1CLR = 0x00010000; // clear existing DMA channel 0 interrupt flag
    IFS4CLR = _IFS4_DMA4IF_MASK;
    IEC4CLR = _IEC4_DMA4IE_MASK;   
    
    DMACONSET = 0x00008000; // enable the DMA controller
    
    DMACONbits.ON = 1;
    DCH4CON = 0x3; // channel off, priority 3, no chaining
    //DCH4CONbits.CHAEN = 1;
    DCH4CONbits.CHPATLEN = 0; // pattern length 2 bytes
    
    DCH4ECON = 0; 
    DCH4ECONbits.PATEN = 1;
    DCH4ECONbits.CHSIRQ = _UART1_RX_VECTOR;
    DCH4ECONbits.SIRQEN = 1;
    
    DCH4DAT = '?';
//    DCH4INT.CHDDIE = 1;
    
//    // program the transfer
//    DCH4SSA = KVA_TO_PA(flashBuff); // transfer source physical address
//    DCH4DSA = KVA_TO_PA(ramBuff); // transfer destination physical address
//    DCH4SSIZ = 200; // source size 200 bytes
//    DCH4DSIZ = 200; // destination size 200 bytes
//    DCH4CSIZ = 200; // 200 bytes transferred per event
//    DCH4INTCLR = 0x00ff00ff; // clear existing events,
    
    // set dma interrupts
    DCH4SSA = KVA_TO_PA((void *)U1RXREG); // transfer source physical address
    DCH4DSA = KVA_TO_PA((void *)ramBuffer); // transfer destination physical address
    DCH4SSIZ = 1; // source size is 1 byte
    DCH4DSIZ = 200; // destination size at most 200 bytes
    DCH4CSIZ = 1;
    
    // clear priority and sub priority     
    IPC34SET = 0x160000; // priority 5, sub-priority 2 
    IEC4SET = _IEC4_DMA4IE_MASK;
    DCH4CONbits.CHEN = 1;
    
    PRISS = 0x76543210;               
 
    INTCONSET = _INTCON_MVEC_MASK;    
    
    __builtin_enable_interrupts(); 
    while(1)
    {
        
        if (U1STAbits.URXDA)
        {
            //c = U1RXREG;
        }
  
    }

    return(EXIT_SUCCESS);
}

