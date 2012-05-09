/* 
* This file is licensed under BSD. It is originally copyright Texas Instruments, 
* but has been adapted by Lars Kristian Roland
*/

/*
* Put an LED between P1.4 and GND (or VCC). Press the button on the other board,
* and your LED should turn on and off. This demo implements an ACK packet that
* goes back to the recipient. (Both devices use the same address). 
*/

#include "ti/include.h"
#include "lcd.h"

extern char paTable[];
extern char paTableLen;

char txBuffer[14]; // = {11, 0x01, 0xCD, 0x86, 0x50, 0x37, 0x26, 0x1D, 0x17};;
char rxBuffer[14];

unsigned volatile int pktcount = 0;
unsigned volatile int RXcount = 0;
unsigned volatile int ACKcount = 0;
unsigned volatile int powerLevel = 0;
unsigned volatile int P1irqs = 0;

void main (void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // delay to compensate for time to startup between MSP430 and CC1100/2500
  __delay_cycles(30000);
  __delay_cycles(30000);
  
  //setscreenmode(1);
  SPISetup();                               // Initialize SPI Display
  clear();
 
  writeString(0,0,"TX:");
  writeString(0,1,"AK:");
  writeString(0,2,"RX:");
  writeString(0,6,"PWR:000");
  writeString(0,7,"0=HIGH");
  
  RF_init();

  // Set button input for DIP board
  P1DIR &= ~(BIT1);
  P2DIR &= ~(BIT1 | BIT2);
  P1REN |= BIT1;
  P2REN |= BIT1 | BIT2;
  P1OUT |= BIT1;
  P2OUT |= BIT1 | BIT2;
  P1IFG = 0x00;
  P2IFG = 0x00;
  P1IE |= BIT1;
  P2IE |= BIT1 | BIT2;
  
  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  
}


// The ISR assumes the interrupt came from a pressed button
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
//  writeInt(0, 7, ++P1irqs,3);
//  writeInt(0, 8, P1IFG,3);
  
  // If Switch was pressed
  if(P1IFG & BIT1 || P1IFG & BIT3)
  {
    // Build packet
    txBuffer[0] = 5;                        // Packet length
    txBuffer[1] = 0x01;                     // Packet address
    txBuffer[2] = 0x11;
    txBuffer[3] = 0x32;
    txBuffer[4] = 0x33;
    txBuffer[5] = 0x34;
    txBuffer[6] = 0x35;
    txBuffer[7] = 0x36;
    txBuffer[8] = 0x37;
          
    // Very stupid debounce mechanism. 
    __delay_cycles(30000);
    __delay_cycles(30000);
    __delay_cycles(30000);
    __delay_cycles(30000);
    __delay_cycles(30000);
    __delay_cycles(30000);                   // Switch debounce
    
    writeString(0,0,"TX:");
    writeInt(35, 0, ++pktcount,3);
    
    RFSendPacket(txBuffer, 6);             // Send value over RF
  }
          
  P1IFG = 0x00; 
}

// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    
  //writeInt(0, 8, P2IFG,3);
  
  // if GDO fired
  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)
  {
    char status[2];
    char len=12;                            
    if (RFReceivePacket(rxBuffer, &len, status))  
    {              
        if (rxBuffer[1] == 0xFF) 
        {
            writeString(0,1,"AK:");
            writeInt(35, 1, ++ACKcount,3);
        }
        else 
        {
          // Send ACK
          // Build packet
          txBuffer[0] = 3;                        // Packet length
          txBuffer[1] = 0x01;                     // Packet address
          txBuffer[2] = 0xFF;
          txBuffer[3] = 0x00;

          RFSendPacket(txBuffer, 4);              // Send value over RF
          writeString(0,2,"RX: ");
          writeInt(35, 2, ++RXcount,3);
        }
      
        writeString(0,4,"RSSI:");
        int rssi_in_dBs = ( (int)( (signed char)status[0] ) - 2 * 74 + 1 ) >> 1; 
        if (rssi_in_dBs < 0) {
            writeString(0,5,"-");
            writeInt(10, 5, -rssi_in_dBs,3);
        } 
        else {
            writeString(0,4,"RSSI: ");
            writeInt(0, 5, rssi_in_dBs,5);
        }
        
     }
    else {
      //writeInt(0, 7, len,3);
      //writeInt(20, 7, status[0],3);
      //writeInt(40, 7, status[1],3);
    }
  }
  
  if(P2IFG & BIT1 || P2IFG & BIT2)
  {
    // Oi. This is a very stupid way to do debouncing... Sorry. 
    __delay_cycles(30000);
    __delay_cycles(30000);
    __delay_cycles(30000); 
    __delay_cycles(30000);
    powerLevel++;
    if (powerLevel > 9) {
      powerLevel = 0;
    }
    RF_change_Power(powerLevel);
    writeInt(60, 6, powerLevel,3);
  }

  P2IFG = 0x00;
}
