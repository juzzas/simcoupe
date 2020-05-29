// Part of SimCoupe - A SAM Coupe emulator
//
// Parallel.cpp: Parallel interface
//
//  Copyright (c) 1999-2014 Simon Owen
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

#include "SAMIO.h"

class CPrintBuffer : public CIoDevice
{
public:
    uint8_t In(uint16_t wPort_) override;
    void Out(uint16_t wPort_, uint8_t bVal_) override;
    void FrameEnd() override;

    bool IsFlushable() const { return !!m_uBuffer; }
    void Flush();

protected:
    bool m_fOpen = false;
    uint8_t m_bControl = 0, m_bData = 0, m_bStatus = 0;

    unsigned int m_uBuffer = 0, m_uFlushDelay = 0;
    uint8_t m_abBuffer[1024];

protected:
    bool IsOpen() const { return false; }

    virtual bool Open() = 0;
    virtual void Close() = 0;
    virtual void Write(uint8_t* pb_, size_t uLen_) = 0;
};


class CPrinterFile final : public CPrintBuffer
{
public:
    CPrinterFile() = default;
    CPrinterFile(const CPrinterFile&) = delete;
    void operator= (const CPrinterFile&) = delete;
    ~CPrinterFile() { Close(); }

public:
    bool Open() override;
    void Close() override;
    void Write(uint8_t* pb_, size_t uLen_) override;

protected:
    FILE* m_hFile = nullptr;
    char* m_pszFile = nullptr;
    char m_szPath[MAX_PATH];
};

class CPrinterDevice : public CPrintBuffer
{
public:
    CPrinterDevice();
    ~CPrinterDevice();

public:
    bool Open() override;
    void Close() override;
    void Write(uint8_t* pb_, size_t uLen_) override;

protected:
#ifdef WIN32
    HANDLE m_hPrinter;  // temporary!
#endif
};


class CMonoDACDevice : public CIoDevice
{
public:
    void Out(uint16_t wPort_, uint8_t bVal_) override;
};


class CStereoDACDevice : public CIoDevice
{
public:
    CStereoDACDevice() : m_bControl(0x00), m_bData(0x80) { }

public:
    void Out(uint16_t wPort_, uint8_t bVal_) override;

protected:
    uint8_t m_bControl, m_bData;
};

extern CPrintBuffer* pPrinterFile;
