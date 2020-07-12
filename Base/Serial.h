// Part of SimCoupe - A SAM Coupe emulator
//
// Serial.cpp: Serial interface
//
//  Copyright (c) 1999-2014 Simon Owen
//  Copyright (c) 2020 Justin Skists
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#pragma once

#include "SimCoupe.h"
#include "SAMIO.h"

class CSerial : public CIoDevice
{
public:
    BYTE In(WORD wPort_) override;
    void Out(WORD wPort_, BYTE bVal_) override;
    virtual bool Update();

protected:
    bool m_fOpen = false;

protected:
    bool IsOpen() const { return false; }

    virtual bool Open(const char *device) = 0;
    virtual void Close() = 0;
    virtual void Write(BYTE* pb_) = 0;
    virtual BYTE Read() = 0;
};


class CSerialComms final : public CSerial
{
public:
    CSerialComms();

    CSerialComms(const CSerialComms&) = delete;
    void operator= (const CSerialComms&) = delete;
    ~CSerialComms() { Close(); }

    const BYTE COMMS_M1_MASK_RXRTS = 0x80;
    const BYTE COMMS_M1_MASK_RINT = 0x40;
    const BYTE COMMS_M1_MASK_ERRORM = 0x20;
    const BYTE COMMS_M1_MASK_PARITY = 0x18;
    const BYTE COMMS_M1_MASK_PARTYP = 0x04;
    const BYTE COMMS_M1_MASK_BITS_PER_CHAR = 0x03;

    const BYTE COMMS_M2_MASK_CHANNEL_MODE = 0xc0;
    const BYTE COMMS_M2_MASK_TXRTS = 0x20;
    const BYTE COMMS_M2_MASK_CTSETX = 0x10;
    const BYTE COMMS_M2_MASK_STOP_BITS = 0x0f;

    const BYTE COMMS_CSR_MASK_RX_CLOCK = 0xf0;
    const BYTE COMMS_CSR_MASK_TX_CLOCK = 0x0f;

    const BYTE COMMS_CR_MASK_CMD = 0xf0;
    const BYTE COMMS_CR_MASK_DIS_TX = 0x08;
    const BYTE COMMS_CR_MASK_ENA_TX = 0x04;
    const BYTE COMMS_CR_MASK_DIS_RX = 0x02;
    const BYTE COMMS_CR_MASK_ENA_RX = 0x01;

    const BYTE COMMS_ACR_MASK_BRGSET = 0x80;
    const BYTE COMMS_ACR_MASK_CTRMODE = 0x70;
    const BYTE COMMS_ACR_MASK_POWERD = 0x08;
    const BYTE COMMS_ACR_MASK_MP0_PIN_SEL = 0x07;

    const BYTE COMMS_SR_RBREAK = 0x80;
    const BYTE COMMS_SR_FRAERR = 0x40;
    const BYTE COMMS_SR_PARERR = 0x20;
    const BYTE COMMS_SR_OVRERR = 0x10;
    const BYTE COMMS_SR_TXEMT = 0x08;
    const BYTE COMMS_SR_TXRDY = 0x04;
    const BYTE COMMS_SR_FFULL = 0x02;
    const BYTE COMMS_SR_RXRDY = 0x01;

    const BYTE COMMS_ISR_MP1PCH = 0x80;
    const BYTE COMMS_ISR_MP1PCS = 0x40;
    const BYTE COMMS_ISR_CNTRDY = 0x10;
    const BYTE COMMS_ISR_DBREAK = 0x08;
    const BYTE COMMS_ISR_RXRDYF = 0x04;
    const BYTE COMMS_ISR_TXEMT = 0x02;
    const BYTE COMMS_ISR_TXRDY = 0x01;

public:
    bool Open(const char *device) override;
    void Close() override;
    void Write(BYTE* pb_) override;

    BYTE In(WORD wPort_) override;

    void Out(WORD wPort_, BYTE bVal_) override;

    BYTE Read() override;
    bool Update() override;

protected:
    FILE* m_hFile = nullptr;
    char* m_pszFile = nullptr;
    char m_szPath[MAX_PATH];

private:
    int m_fd = 0;

    void writeMR1(BYTE bVal_);

    void writeMR2(BYTE bVal_);

    void writeCSR(BYTE bVal_);

    void writeCR(BYTE bVal_);

    void writeTHR(BYTE bVal_);

    void writeACR(BYTE bVal_);

    void writeIMR(BYTE bVal_);


    void writeCTUR(BYTE bVal_);

    void writeCTLR(BYTE bVal_);

    BYTE readMR1();

    BYTE readMR2();

    BYTE readSR();

    BYTE readRHR();

    BYTE readISR();

    BYTE readCTUR();

    BYTE readCTLR();

    // data from http://www.samcoupe-pro-dos.co.uk/edwin/datasheets/SCC2691_2.pdf

    BYTE m_reg_mr1; //mr1 is pointed to after reset and when it is reset through the command register
    BYTE m_reg_mr2; // mr2 is pointed to after mr1 is accessed
    BYTE m_reg_csr; // clock select register
    BYTE m_reg_cr;  // command register
    BYTE m_reg_sr;  // channel status register
    BYTE m_reg_acr;  // auxillary control register
    BYTE m_reg_isr;  // interrupt status register
    BYTE m_reg_imr;  // interrupt mask register
    BYTE m_reg_ctur;  // counter/timer upper register
    BYTE m_reg_ctlr;  // counter/timer lower register

    BYTE m_reg_rhr;  // receive character holding register

    bool m_is_reg_mr2;

    bool m_rx_ready;

    bool m_is_rx_disabled;
    bool m_is_rx_enabled;
    bool m_is_tx_disabled;
    bool m_is_tx_enabled;
};

