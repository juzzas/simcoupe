// Part of SimCoupe - A SAM Coupe emulator
//
// Video.h: Base video interface
//
//  Copyright (c) 1999-2012 Simon Owen
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

#include "Screen.h"

enum { VCAP_STRETCH = 1, VCAP_FILTER = 2, VCAP_SCANHIRES = 4 };

namespace Video
{
bool Init(bool fFirstInit_ = false);
void Exit(bool fReInit_ = false);

bool IsLineDirty(int nLine_);
void SetLineDirty(int nLine_);
void SetDirty();

bool CheckCaps(int nCaps_);

void Update(const Screen& pScreen_);
void UpdateSize();
void UpdatePalette();

void DisplayToSamSize(int* pnX_, int* pnY_);
void DisplayToSamPoint(int* pnX_, int* pnY_);
}


struct IVideoRenderer
{
    virtual ~IVideoRenderer() = default;

    virtual int GetCaps() const = 0;
    virtual bool Init() = 0;

    virtual void Update(const Screen& pScreen_, bool* pafDirty_) = 0;
    virtual void UpdateSize() = 0;
    virtual void UpdatePalette() = 0;

    virtual void DisplayToSamSize(int* pnX_, int* pnY_) = 0;
    virtual void DisplayToSamPoint(int* pnX_, int* pnY_) = 0;
};
