/*
* Author and copyright 2011: Lars Kristian Roland
* LarsRF by Lars Kristian Roland is licensed under a Creative Commons 
* Attribution-ShareAlike 3.0 Unported License.
* Based on a work at github.com.
* Permissions beyond the scope of this license may be available at http://lars.roland.bz/.
*/

#include <msp430.h>

#include "commands.h"
#include "spi_usci_CC1101_hal.h"
#include "various.h"
#include "rf_CC1101_hal.h"

#define PACKET_LENGTH 0x10;
    
void RF_init()
{
  /*
   Deviation = 47.607422 
 Base frequency = 867.999939 
 Carrier frequency = 867.999939 
 Channel number = 0 
 Carrier frequency = 867.999939 
 Modulated = true 
 Modulation format = GFSK 
 Manchester enable = false 
 Sync word qualifier mode = 30/32 sync word bits detected 
 Preamble count = 4 
 Channel spacing = 199.951172 
 Carrier frequency = 867.999939 
 Data rate = 99.9756 
 RX filter BW = 325.000000 
 Data format = Normal mode 
 Length config = Variable packet length mode. Packet length configured by the first byte after sync word 
 CRC enable = true 
 Packet length = 255 
 Device address = 0 
 Address config = No address check 
 CRC autoflush = false 
 PA ramping = false 
 TX power = 12 
  */
  
    RF_write_reg( FSCTRL1 ,     0x0C);      // Frequency Synthesizer Control   
    RF_write_reg( IOCFG0  ,     0x06);      // GDO0 Output Pin Configuration   
    RF_write_reg( FSCTRL0 ,     0x00);      // Frequency Synthesizer Control   
    RF_write_reg( FREQ2   ,     0x21);      // Frequency Control Word, High Byte   
    RF_write_reg( FREQ1   ,     0x62);      // Frequency Control Word, Middle Byte   
    RF_write_reg( FREQ0   ,     0x76);      // Frequency Control Word, Low Byte   
    RF_write_reg( MDMCFG4 ,     0x5B);      // Modem Configuration   
    RF_write_reg( MDMCFG3 ,     0xF8);      // Modem Configuration   
    RF_write_reg( MDMCFG2 ,     0x93);      // Modem Configuration   
    RF_write_reg( MDMCFG1 ,     0x22);      // Modem Configuration   
    RF_write_reg( MDMCFG0 ,     0xF8);      // Modem Configuration   
    RF_write_reg( CHANNR  ,     0x00);      // Channel Number   
    RF_write_reg( DEVIATN ,     0x47);      // Modem Deviation Setting   
    RF_write_reg( FREND1  ,     0xB6);      // Front End RX Configuration   
    RF_write_reg( FREND0  ,     0x10);      // Front End TX Configuration   
    RF_write_reg( MCSM0   ,     0x18);      // Main Radio Control State Machine Configuration   
    RF_write_reg( FOCCFG  ,     0x1D);      // Frequency Offset Compensation Configuration   
    RF_write_reg( BSCFG   ,     0x1C);      // Bit Synchronization Configuration   
    RF_write_reg( AGCCTRL2,     0xC7);      // AGC Control   
    RF_write_reg( AGCCTRL1,     0x00);      // AGC Control   
    RF_write_reg( AGCCTRL0,     0xB2);      // AGC Control   
    RF_write_reg( FSCAL3  ,     0xEA);      // Frequency Synthesizer Calibration   
    RF_write_reg( FSCAL2  ,     0x2A);      // Frequency Synthesizer Calibration   
    RF_write_reg( FSCAL1  ,     0x00);      // Frequency Synthesizer Calibration   
    RF_write_reg( FSCAL0  ,     0x1F);      // Frequency Synthesizer Calibration   
    RF_write_reg( FSTEST  ,     0x59);      // Frequency Synthesizer Calibration Control   
    RF_write_reg( TEST2   ,     0x81);      // Various Test Settings   
    RF_write_reg( TEST1   ,     0x35);      // Various Test Settings   
    RF_write_reg( TEST0   ,     0x09);      // Various Test Settings   
    RF_write_reg( FIFOTHR ,     0x07);      // RX FIFO and TX FIFO Thresholds   
    RF_write_reg( IOCFG2  ,     0x29);      // GDO2 Output Pin Configuration   
    RF_write_reg( PKTCTRL1,     0x04);      // Packet Automation Control   
    RF_write_reg( PKTCTRL0,     0x05);      // Packet Automation Control   
    RF_write_reg( ADDR    ,     0x00);      // Device Address   
    RF_write_reg( PKTLEN  ,     0xFF);      // Packet Length   

}

void RF_init_slow()
{
  /* Generated by SmartRF studio using the following config:
    RF_write_reg( @RN@@<<@, @<<@0x@VH@);  @<<@// @Rd@   

     Deviation = 19.042969 
     Base frequency = 867.999939 
     Carrier frequency = 867.999939 
     Channel number = 0 
     Carrier frequency = 867.999939 
     Modulated = true 
     Modulation format = GFSK 
     Manchester enable = false 
     Sync word qualifier mode = 30/32 sync word bits detected 
     Preamble count = 4 
     Channel spacing = 199.951172 
     Carrier frequency = 867.999939 
     Data rate = 9.9926 
     RX filter BW = 101.562500 
     Data format = Normal mode 
     Length config = Variable packet length mode. Packet length configured by the first byte after sync word 
     CRC enable = true 
     Packet length = 255 
     Device address = 0 
     Address config = No address check 
     CRC autoflush = false 
     PA ramping = false 
     TX power = 0 
    */
  
  /* RF settings LarsRF */

  RF_write_reg( FSCTRL1 ,     0x06);      // Frequency Synthesizer Control   
  RF_write_reg( IOCFG0  ,     0x07);      // GDO0 Output Pin Configuration   
  RF_write_reg( IOCFG2  ,     0x06);      // GDO2 Output Pin Configuration   
  RF_write_reg( FSCTRL0 ,     0x00);      // Frequency Synthesizer Control   
  RF_write_reg( FREQ2   ,     0x21);      // Frequency Control Word, High Byte   
  RF_write_reg( FREQ1   ,     0x62);      // Frequency Control Word, Middle Byte   
  RF_write_reg( FREQ0   ,     0x76);      // Frequency Control Word, Low Byte   
  RF_write_reg( MDMCFG4 ,     0xC8);      // Modem Configuration   
  RF_write_reg( MDMCFG3 ,     0x93);      // Modem Configuration   
  RF_write_reg( MDMCFG2 ,     0x13);      // Modem Configuration   
  RF_write_reg( MDMCFG1 ,     0x22);      // Modem Configuration   
  RF_write_reg( MDMCFG0 ,     0xF8);      // Modem Configuration   
  RF_write_reg( CHANNR  ,     0x00);      // Channel Number   
  RF_write_reg( DEVIATN ,     0x34);      // Modem Deviation Setting   
  RF_write_reg( FREND1  ,     0x56);      // Front End RX Configuration   
  RF_write_reg( FREND0  ,     0x10);      // Front End TX Configuration   
  RF_write_reg( MCSM0   ,     0x18);      // Main Radio Control State Machine Configuration   
  RF_write_reg( MCSM1   ,     0x00);      // Main Radio Control State Machine Configuration   
  RF_write_reg( FOCCFG  ,     0x16);      // Frequency Offset Compensation Configuration   
  RF_write_reg( BSCFG   ,     0x6C);      // Bit Synchronization Configuration   
  RF_write_reg( AGCCTRL2,     0x43);      // AGC Control   
  RF_write_reg( AGCCTRL1,     0x40);      // AGC Control   
  RF_write_reg( AGCCTRL0,     0x91);      // AGC Control   
  RF_write_reg( FSCAL3  ,     0xE9);      // Frequency Synthesizer Calibration   
  RF_write_reg( FSCAL2  ,     0x2A);      // Frequency Synthesizer Calibration   
  RF_write_reg( FSCAL1  ,     0x00);      // Frequency Synthesizer Calibration   
  RF_write_reg( FSCAL0  ,     0x1F);      // Frequency Synthesizer Calibration   
  RF_write_reg( FSTEST  ,     0x59);      // Frequency Synthesizer Calibration Control   
  RF_write_reg( TEST2   ,     0x81);      // Various Test Settings   
  RF_write_reg( TEST1   ,     0x35);      // Various Test Settings   
  RF_write_reg( TEST0   ,     0x09);      // Various Test Settings   
  RF_write_reg( FIFOTHR ,     0x07);      // RX FIFO and TX FIFO Thresholds   
  RF_write_reg( PKTCTRL1,     0x04);      // Packet Automation Control   
  RF_write_reg( PKTCTRL0,     0x05);      // Packet Automation Control   
  RF_write_reg( ADDR    ,     0x00);      // Device Address   
  RF_write_reg( PKTLEN  ,     0xFF);      // Packet Length   

}


void RF_strobe( uint8 strobe )
{
    SPI_strobe( strobe);
}

uint8 RF_read_reg( uint8 addr )
{
    return SPI_read_single( addr);
}

void RF_write_reg( uint8 addr, uint8 value)
{
    SPI_write_single( addr, value);
}

uint8 RF_read_status( uint8 addr )
{
    return SPI_read_status( addr );
}

void RF_reset()
{
  RF_strobe( SRES );
}

uint8 RF_RSSI()
{
  return RF_read_status( RSSI );
}


void RF_send_packet(uint8 *txBuffer, uint8 length)
{
    SPI_strobe(SFTX); // Flush TX fifo
    SPI_write_single(TXFIFO, length); // First write packet length
    SPI_write_burst(TXFIFO, txBuffer, length); // Fill the TX fifo with our packet
    SPI_strobe(STX); // Transmit

    uint8 txbytes = RF_read_status( TXBYTES );
    while (txbytes > 0 ) {
      txbytes = RF_read_status( TXBYTES );
    }
          
}

uint8 RF_receive_packet(uint8 *rxBuffer)
{
    long timeout = 1000;
    
    SPI_strobe(SRX);
    
    // Check if bytes are received. Should maybe check for overflow also.
    uint8 rxbytes = SPI_read_status(RXBYTES);
    while (rxbytes == 0 && timeout-- > 0) {
      __delay_cycles(100);
      uint8 marc_state = SPI_read_status(MARCSTATE);
      rxbytes = SPI_read_status(RXBYTES);
    }
    
    if (rxbytes == 0) {
      // Didn't get a packet
      uint8 marc_state = SPI_read_status(MARCSTATE);
      SPI_strobe(SFRX);
      return 0;
    }
    else if (rxbytes & BIT7) {
      // RX Overflow
      SPI_strobe(SFRX);
      return 0;
    }
    
    uint8 packet_length = RF_read_reg(RXFIFO); // first byte is packet length
    SPI_read_burst(RXFIFO, rxBuffer, packet_length); // read packet into rxbuffer
    uint8 status1 = SPI_read_single(RXFIFO); // RSSI
    uint8 status2 = SPI_read_single(RXFIFO); // CRC and Signal quality
    
    //SPI_strobe(SFRX); // only allow one packet at the time... flush the RX FIFO
    
    // If CRC ok, return packet length. If not, return 0
    if (status2 & BIT7) { 
      return packet_length;
    }
    else {
      return 0;
    }

}


uint8 RF_receive_packet_ack(uint8 *rxBuffer, uint8 *ackBuffer, uint8 acklength)
{
  SPI_strobe(SFTX);
  
  P1OUT |= BIT1;
  SPI_strobe(SIDLE);
  RF_write_reg( MCSM1 , 0x08); 

  uint8 length = RF_receive_packet(rxBuffer);
  
  if (length == 0) {
    return 0;
  }
  
  //__delay_cycles(10000);
  
  SPI_write_single(TXFIFO, length); // First write packet length
  SPI_write_burst(TXFIFO, ackBuffer, acklength); // Fill the TX fifo with our packet
  uint8 txbytes = RF_read_status( TXBYTES );
  
  uint8 marc_state = SPI_read_status(MARCSTATE);
  while (marc_state != STATE_TX) {
      marc_state = SPI_read_status(MARCSTATE);
      SPI_strobe(STX);
  }
  
  while (txbytes > 0 ) {
    txbytes = RF_read_status( TXBYTES );
  }
    
  SPI_strobe(SIDLE);  
  P1OUT |= BIT1;
  
  return length;
}

uint8 RF_send_packet_ack(uint8 *txBuffer, uint8 length)
{
  SPI_strobe(SIDLE);
  RF_write_reg( MCSM1 , 0x03); // turn on RX after TX and TX after RX (not idle)
  P1OUT |= BIT1;
  RF_send_packet(txBuffer, length);
  
  /*
  uint8 marc_state = SPI_read_status(MARCSTATE);
  while (marc_state != STATE_RX) {
      marc_state = SPI_read_status(MARCSTATE);
      SPI_strobe(SRX);
  }
  */
  P1OUT &= ~BIT1;
  P1OUT |= BIT1;
  P2OUT |= BIT2;
  uint8 ack_length = RF_receive_packet(txBuffer); 
  P1OUT &= ~BIT1;
  SPI_strobe(SIDLE);
  RF_write_reg( MCSM1 , 0x00); // go to IDLE after RX or TX
  
  P2OUT &= ~BIT2;
  
  return ack_length;
}