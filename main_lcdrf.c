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

char txBuffer[14];
char rxBuffer[14];

unsigned int pktcount = 0;
unsigned int RXcount = 0;
unsigned int ACKcount = 0;

void main (void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // 5ms delay to compensate for time to startup between MSP430 and CC1100/2500
  __delay_cycles(50000);
  
  SPISetup();                               // Initialize SPI Display
  clear();
  
  writeString(0,0,"TX PKT:");
  writeString(0,1,"RX ACK:");
  writeString(0,2,"RX PKT: ");
  
  TI_CC_SPISetup();                         // Initialize SPI port

  TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx
  writeRFSettings();                        // Write RF settings to config reg
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);  //Write PATABLE

  // Set button input for DIP board
  P1DIR &= ~(BIT1);
  P2DIR &= ~(BIT1 | BIT2);
  P1REN |= BIT1;
  P2REN |= BIT1 | BIT2;
  P1OUT |= BIT1;
  P2OUT |= BIT1 | BIT2;
  P1IE |= BIT1;
  P2IE |= BIT0 | BIT1;
  
  // Configure ports -- switch inputs, LEDs, GDO0 to RX packet info from CCxxxx
  TI_CC_SW_PxREN |= TI_CC_SW1;               // Enable Pull up resistor
  TI_CC_SW_PxOUT |= TI_CC_SW1;               // Enable pull up resistor
  TI_CC_SW_PxIES |= TI_CC_SW1;               // Int on falling edge
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1);            // Clr flags
  TI_CC_SW_PxIE |= TI_CC_SW1;                // Activate interrupt enables
  //TI_CC_LED_PxOUT &= ~(TI_CC_LED1); // Outputs = 0
  //TI_CC_LED_PxDIR |= TI_CC_LED1;// LED Direction to Outputs

  TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;       // Int on falling edge (end of pkt)
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear flag
  TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;        // Enable int on end of packet

  TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
                                            // When a pkt is received, it will
                                            // signal on GDO0 and wake CPU
  

  //__bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, enable interrupts
  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  
}


// The ISR assumes the interrupt came from a pressed button
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
  
  // If Switch was pressed
  if(P1IFG & BIT2 || P1IFG & BIT3)
  {
  
    // Build packet
    txBuffer[0] = 11;                        // Packet length
    txBuffer[1] = 0x01;                     // Packet address
    txBuffer[2] = 0x11;
    txBuffer[3] = 0x32;
    txBuffer[4] = 0x33;
    txBuffer[5] = 0x34;
    txBuffer[6] = 0x35;
    txBuffer[7] = 0x36;
    txBuffer[8] = 0x37;
    txBuffer[9] = 0x38;
    txBuffer[10] = 0x39;
    txBuffer[11] = 0x40;
          
    RFSendPacket(txBuffer, 12);             // Send value over RF
    __delay_cycles(5000);                   // Switch debounce
    
    writeString(0,0,"TX PKT:");
    writeInt(75, 0, ++pktcount,3);
    P1IFG &= ~(BIT2);                       // Clr flag that caused int
    P1IFG &= ~(BIT3);                       // Clr flag that caused int
  }
          
  P1IFG = 0x00; 
}

// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    // if GDO fired
  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)
  {
    char status[2];
    char len=12;                            // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
    if (RFReceivePacket(rxBuffer, &len, status))  
    {   
        // Fetch packet from CCxxxx
        //TI_CC_LED_PxOUT ^= rxBuffer[1];         // Toggle LEDs according to pkt data
           
        if (rxBuffer[1] == 0xFF) 
        {
            writeString(0,1,"RX ACK:");
            writeInt(75, 1, ++ACKcount,3);
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
          writeString(0,2,"RX PKT: ");
          writeInt(75, 2, ++RXcount,3);
        }
        
        writeString(0,3,"STATUS: ");
        writeHex(60, 3, status[0]);
        writeHex(75, 3, status[1]);
      
        
        int rssi_in_dBs = ( (int)( (signed char)status[0] ) - 2 * 74 + 1 ) >> 1; 
        if (rssi_in_dBs < 0) {
            writeString(0,4,"RSSI: -");
            writeInt(40, 4, -rssi_in_dBs,3);
        } 
        else {
            writeString(0,4,"RSSI: ");
            writeInt(30, 4, rssi_in_dBs,5);
        }
        
     }
  }

  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // After pkt RX, this flag is set.
  TI_CC_GDO0_PxIFG = 0x00;
}
