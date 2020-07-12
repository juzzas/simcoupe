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

#include <fcntl.h>
#include <termios.h>

#include "SimCoupe.h"
#include "Serial.h"


BYTE CSerial::In(WORD wPort_)
{
    TRACE("Serial IN %04x\n", wPort_);

    return 0xff;
}


void CSerial::Out(WORD wPort_, BYTE bVal_)
{
    TRACE("Serial OUT %04x = %02x\n", wPort_, bVal_);
}

bool CSerial::Update() {
    return false;
}

CSerialComms::CSerialComms() :
        m_rx_ready(false),
        m_is_reg_mr2(false)
{}

bool CSerialComms::Open(const char *device)
{
    struct termios options;

    m_fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_fd < 0)
        return false;

    fcntl(m_fd, F_SETFL, FNDELAY);

    tcgetattr(m_fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);

    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8;    /* Select 8 data bits */

    options.c_cflag &= ~CRTSCTS; /* disable hardware flow control */

    options.c_cflag &= ~(ICANON | ECHO | ECHOE | ISIG);    /* unprocessed input */

    options.c_iflag &= ~(IXON | IXOFF | IXANY); /* disable software flow control */

    options.c_oflag &= ~OPOST; /* raw output */

    options.c_oflag = 0;                // no remapping, no delays
    options.c_cc[VMIN]  = 0;            // read doesn't block
    options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tcsetattr(m_fd, TCSANOW, &options);

    return true;
}

void CSerialComms::Close()
{
    if (m_fd > 0)
        close(m_fd);
}

void CSerialComms::writeMR1(BYTE bVal_)
{
    m_reg_mr1 = bVal_;
}

void CSerialComms::writeMR2(BYTE bVal_)
{
    m_reg_mr2 = bVal_;
}

void CSerialComms::writeCSR(BYTE bVal_)
{
    m_reg_csr = bVal_;
}

void CSerialComms::writeCR(BYTE bVal_)
{
    m_reg_cr = bVal_;

    switch (bVal_ & COMMS_CR_MASK_CMD)
    {
        case 0x00:
            break;

        case 0x10:
            m_is_reg_mr2 = false;
            break;

    }
}

void CSerialComms::writeTHR(BYTE bVal_)
{
   TRACE("Serial TX %02x\n", bVal_);

    write(m_fd, &bVal_, 1);
}


void CSerialComms::writeACR(BYTE bVal_)
{
    m_reg_acr = bVal_;
}

void CSerialComms::writeIMR(BYTE bVal_)
{
    m_reg_imr = bVal_;
}

void CSerialComms::writeCTUR(BYTE bVal_)
{
    m_reg_ctur = bVal_;
}

void CSerialComms::writeCTLR(BYTE bVal_)
{
    m_reg_ctlr = bVal_;
}

BYTE CSerialComms::readMR1()
{
    return m_reg_mr1;
}

BYTE CSerialComms::readMR2()
{
    return m_reg_mr2;
}

BYTE CSerialComms::readSR()
{
    BYTE val = 0;

    if (!m_rx_ready)
    {
        int rc = read(m_fd, &m_reg_rhr, sizeof(m_reg_rhr));
        if (rc == 1)
            m_rx_ready = true;
    }

    val |= COMMS_SR_TXRDY | COMMS_SR_TXEMT;
    val |= m_rx_ready ? COMMS_SR_RXRDY : 0;

    return val;
}

BYTE CSerialComms::readRHR()
{
    TRACE("Serial RX %02x\n", m_reg_rhr);
    m_rx_ready = false;

    return m_reg_rhr;
}

BYTE CSerialComms::readISR() {
    return 0;
}

BYTE CSerialComms::readCTUR() {
    return m_reg_ctur;
}

BYTE CSerialComms::readCTLR() {
    return m_reg_ctlr;
}


void CSerialComms::Write(BYTE *pb_)
{

}

BYTE CSerialComms::Read()
{
    return 0;
}

bool CSerialComms::Update() {
    // Call base to update time values
    if (!CSerial::Update())
        return false;

    return true;
}

BYTE CSerialComms::In(WORD wPort_) {
    BYTE val;

    switch (wPort_ & 0xff00u)
    {
        case 0x0000:
            if (!m_is_reg_mr2)
            {
                val = readMR1();
                m_is_reg_mr2 = true;
            }
            else
            {
                val = readMR2();
            }
            break;

        case 0x0100:
            val = readSR();
            break;

        case 0x0300:
            val = readRHR();
            break;

        case 0x0500:
            val = readISR();
            break;

        case 0x0600:
            val = readCTUR();
            break;

        case 0x0700:
            val = readCTLR();
            break;

        default:
            val = CSerial::In(wPort_);
    }

    return val;
}

void CSerialComms::Out(WORD wPort_, BYTE bVal_) {
    switch (wPort_ & 0xff00u) {
        case 0x0000:
            if (!m_is_reg_mr2) {
                writeMR1(bVal_);
                m_is_reg_mr2 = true;
            } else {
                writeMR2(bVal_);
            }
            break;

        case 0x0100:
            writeCSR(bVal_);
            break;

        case 0x0200:
            writeCR(bVal_);
            break;

        case 0x0300:
            writeTHR(bVal_);
            break;

        case 0x0400:
            writeACR(bVal_);
            break;

        case 0x0500:
            writeIMR(bVal_);
            break;

        case 0x0600:
            writeCTUR(bVal_);
            break;

        case 0x0700:
            writeCTLR(bVal_);
            break;

        default:
            CSerial::Out(wPort_, bVal_);
    }
}

