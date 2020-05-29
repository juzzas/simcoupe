// Part of SimCoupe - A SAM Coupe emulator
//
// Joystick.cpp: Common joystick handling
//
//  Copyright (c) 1999-2011  Simon Owen
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

#include "SimCoupe.h"
#include "Joystick.h"

namespace Joystick
{

static int anPosition[MAX_JOYSTICKS];
static uint32_t adwButtons[MAX_JOYSTICKS];


void Init(bool /*fFirstInit_*/)
{
    for (int i = 0; i < MAX_JOYSTICKS; i++)
    {
        anPosition[i] = HJ_CENTRE;
        adwButtons[i] = 0;
    }
}

void Exit(bool /*fReInit_*/)
{
}


void SetX(int nJoystick_, int nPosition_)
{
    int nLeftRight = HJ_LEFT | HJ_RIGHT;

    // Opposite directions cancel each other out
    if (!(~nPosition_ & nLeftRight))
        nPosition_ &= ~nLeftRight;

    // Update the stick state
    if (nJoystick_ < MAX_JOYSTICKS)
        (anPosition[nJoystick_] &= ~nLeftRight) |= (nPosition_ & nLeftRight);
}

void SetY(int nJoystick_, int nPosition_)
{
    int nUpDown = HJ_UP | HJ_DOWN;

    // Opposite directions cancel each other out
    if (!(~nPosition_ & nUpDown))
        nPosition_ &= ~nUpDown;

    // Update the stick state
    if (nJoystick_ < MAX_JOYSTICKS)
        (anPosition[nJoystick_] &= ~nUpDown) |= (nPosition_ & nUpDown);
}

void SetPosition(int nJoystick_, int nPosition_)
{
    int nLeftRight = HJ_LEFT | HJ_RIGHT, nUpDown = HJ_UP | HJ_DOWN;

    // Opposite directions cancel each other out
    if (!(~nPosition_ & nLeftRight)) nPosition_ &= ~nLeftRight;
    if (!(~nPosition_ & nUpDown)) nPosition_ &= ~nUpDown;

    // Update the stick state
    if (nJoystick_ < MAX_JOYSTICKS)
        anPosition[nJoystick_] = nPosition_;
}

void SetButton(int nJoystick_, int nButton_, bool fPressed_)
{
    if (nJoystick_ < MAX_JOYSTICKS)
    {
        uint32_t dwBit = 1 << nButton_;

        if (fPressed_)
            adwButtons[nJoystick_] |= dwBit;
        else
            adwButtons[nJoystick_] &= ~dwBit;
    }
}

void SetButtons(int nJoystick_, uint32_t dwButtons_)
{
    if (nJoystick_ < MAX_JOYSTICKS)
        adwButtons[nJoystick_] = dwButtons_;
}


uint8_t ReadSinclair1(int nJoystick_)
{
    uint8_t bRet = 0;

    if (nJoystick_ < MAX_JOYSTICKS)
    {
        if (anPosition[nJoystick_] & HJ_LEFT)  bRet |= 1;
        if (anPosition[nJoystick_] & HJ_RIGHT) bRet |= 2;
        if (anPosition[nJoystick_] & HJ_DOWN)  bRet |= 4;
        if (anPosition[nJoystick_] & HJ_UP)    bRet |= 8;
        if (adwButtons[nJoystick_])            bRet |= 16;
    }

    return bRet;
}

uint8_t ReadSinclair2(int nJoystick_)
{
    uint8_t bRet = 0;

    if (nJoystick_ < MAX_JOYSTICKS)
    {
        if (adwButtons[nJoystick_])            bRet |= 1;
        if (anPosition[nJoystick_] & HJ_UP)    bRet |= 2;
        if (anPosition[nJoystick_] & HJ_DOWN)  bRet |= 4;
        if (anPosition[nJoystick_] & HJ_RIGHT) bRet |= 8;
        if (anPosition[nJoystick_] & HJ_LEFT)  bRet |= 16;
    }

    return bRet;
}

uint8_t ReadKempston(int nJoystick_)
{
    uint8_t bRet = 0;

    if (nJoystick_ < MAX_JOYSTICKS)
    {
        if (anPosition[nJoystick_] & HJ_RIGHT) bRet |= 1;
        if (anPosition[nJoystick_] & HJ_LEFT)  bRet |= 2;
        if (anPosition[nJoystick_] & HJ_DOWN)  bRet |= 4;
        if (anPosition[nJoystick_] & HJ_UP)    bRet |= 8;
        if (adwButtons[nJoystick_])            bRet |= 16;
    }

    return bRet;
}

} // namespace Joystick
