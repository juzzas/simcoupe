// Part of SimCoupe - A SAM Coupe emulator
//
// IDEDisk.h: Platform-specific IDE direct disk access
//
//  Copyright (c) 2003-2014 Simon Owen
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

#include "HardDisk.h"

class DeviceHardDisk : public HardDisk
{
public:
    DeviceHardDisk(const char* pcszDisk_) : HardDisk(pcszDisk_) { }

public:
    bool IsOpen() const { return m_hDevice != -1; }
    bool Open(bool fReadOnly_ = false) override;
    void Close();

    bool ReadSector(unsigned int uSector_, uint8_t* pb_) override;
    bool WriteSector(unsigned int uSector_, uint8_t* pb_) override;

protected:
    int m_hDevice = -1;
};
