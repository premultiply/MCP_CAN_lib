/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.

  Author: Loovee
  Contributor: Cory J. Fowler
  2017-09-25
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"
#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
    private:

    byte   m_nExtFlg;                                                    // Identifier Type
                                                                         // Extended (29 bit) or Standard (11 bit)
    unsigned long  m_nID;                                                // CAN ID
    byte   m_nDlc;                                                       // Data Length Code
    byte   m_nDta[MAX_CHAR_IN_MESSAGE];                                  // Data array
    byte   m_nRtr;                                                       // Remote request flag
    byte   m_nfilhit;                                                    // The number of the filter that matched the message
    byte   MCPCS;                                                        // Chip Select pin number
    byte   mcpMode;                                                      // Mode to return to after configurations are performed.


/*********************************************************************************************************
 *  mcp2515 driver function
 *********************************************************************************************************/
   private:

    void mcp2515_reset(void);                                            // Soft Reset MCP2515

    byte mcp2515_readRegister(const byte address);                       // Read MCP2515 register

    void mcp2515_readRegisterS(const byte address,                       // Read MCP2515 successive registers
                                 byte values[],
                               const byte n);

    void mcp2515_setRegister(const byte address,                         // Set MCP2515 register
                             const byte value);

    void mcp2515_setRegisterS(const byte address,                        // Set MCP2515 successive registers
                              const byte values[],
                              const byte n);

    void mcp2515_initCANBuffers(void);

    void mcp2515_modifyRegister(const byte address,                      // Set specific bit(s) of a register
                                const byte mask,
                                const byte data);

    byte mcp2515_readStatus(void);                                       // Read MCP2515 Status
    byte mcp2515_setCANCTRL_Mode(const byte newmode);                    // Set mode
    byte mcp2515_configRate(const byte canSpeed,                         // Set baud rate
                             const byte canClock);

    byte mcp2515_init(const byte canIDMode,                              // Initialize Controller
                       const byte canSpeed,
                       const byte canClock);

    void mcp2515_write_mf( const byte mcp_addr,                          // Write CAN Mask or Filter
                           const byte ext,
                           const unsigned long id );

    void mcp2515_write_id( const byte mcp_addr,                          // Write CAN ID
                           const byte ext,
                           const unsigned long id );

    void mcp2515_read_id( const byte mcp_addr,                           // Read CAN ID
                byte* ext,
                                unsigned long* id );

    void mcp2515_write_canMsg( const byte buffer_sidh_addr );            // Write CAN message
    void mcp2515_read_canMsg( const byte buffer_sidh_addr);              // Read CAN message
    byte mcp2515_getNextFreeTXBuf(byte *txbuf_n);                        // Find empty transmit buffer

/*********************************************************************************************************
 *  CAN operator function
 *********************************************************************************************************/

    byte setMsg(unsigned long id, byte rtr, byte ext, byte len, byte *pData); // Set message
    byte clearMsg();                                                     // Clear all message to zero
    byte readMsg();                                                      // Read message
    byte sendMsg();                                                      // Send message

public:
    MCP_CAN(byte _CS);
    byte begin(byte idmodeset, byte speedset, byte clockset);            // Initialize controller parameters
    byte init_Mask(byte num, byte ext, unsigned long ulData);            // Initialize Mask(s)
    byte init_Mask(byte num, unsigned long ulData);                      // Initialize Mask(s)
    byte init_Filt(byte num, byte ext, unsigned long ulData);            // Initialize Filter(s)
    byte init_Filt(byte num, unsigned long ulData);                      // Initialize Filter(s)
    byte setMode(byte opMode);                                           // Set operational mode
    byte sendMsgBuf(unsigned long id, byte ext, byte len, byte *buf);    // Send message to transmit buffer
    byte sendMsgBuf(unsigned long id, byte len, byte *buf);              // Send message to transmit buffer
    byte readMsgBuf(unsigned long *id, byte *ext, byte *len, byte *buf); // Read message from receive buffer
    byte readMsgBuf(unsigned long *id, byte *len, byte *buf);            // Read message from receive buffer
    byte checkReceive(void);                                             // Check for received data
    byte checkError(void);                                               // Check for errors
    byte getError(void);                                                 // Check for errors
    byte errorCountRX(void);                                             // Get error count
    byte errorCountTX(void);                                             // Get error count
    byte enOneShotTX(void);                                              // Enable one-shot transmission
    byte disOneShotTX(void);                                             // Disable one-shot transmission
    byte abortTX(void);                                                  // Abort queued transmission(s)
    byte setGPO(byte data);                                              // Sets GPO
    byte getGPI(void);                                                   // Reads GPI
};

#endif
/*********************************************************************************************************
 *  END FILE
 *********************************************************************************************************/
