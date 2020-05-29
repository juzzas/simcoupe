// Part of SimCoupe - A SAM Coupe emulator
//
// Sound.h: Common sound generation
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

#include "SAMIO.h"
#include "BlipBuffer.h"

#ifdef HAVE_LIBSAASOUND
#include "SAASound.h"
#else
#include "SAA1099.h"
#endif

#define SAMPLE_FREQ         44100
#define SAMPLE_BITS         16
#define SAMPLE_CHANNELS     2
#define SAMPLE_BLOCK        (SAMPLE_BITS*SAMPLE_CHANNELS/8)


class Sound
{
public:
    static bool Init(bool fFirstInit_ = false);
    static void Exit(bool fReInit_ = false);

    static void Silence();
    static void FrameUpdate();
};

class CSoundDevice : public CIoDevice
{
public:
    CSoundDevice();
    CSoundDevice(const CSoundDevice&) = delete;
    void operator= (const CSoundDevice&) = delete;
    virtual ~CSoundDevice() { delete[] m_pbFrameSample; }

public:
    int GetSampleCount() { return m_nSamplesThisFrame; }
    uint8_t* GetSampleBuffer() { return m_pbFrameSample; }

protected:
    int m_nSamplesThisFrame = 0;
    uint8_t* m_pbFrameSample = nullptr;
};

class CSAA final : public CSoundDevice
{
public:
    CSAA()
    {
#ifdef HAVE_LIBSAASOUND
        m_pSAASound = CreateCSAASound();
        m_pSAASound->SetSoundParameters(SAAP_NOFILTER | SAAP_44100 | SAAP_16BIT | SAAP_STEREO);
        static_assert(SAMPLE_FREQ == 44100 && SAMPLE_BITS == 16 && SAMPLE_CHANNELS == 2, "SAA parameter mismatch");
#else
        m_pSAASound = new CSAASound(SAMPLE_FREQ);
#endif
    }
    CSAA(const CSAA&) = delete;
    void operator= (const CSAA&) = delete;
    ~CSAA()
    {
#ifdef HAVE_LIBSAASOUND
        if (m_pSAASound)
            DestroyCSAASound(m_pSAASound);
#else
        delete m_pSAASound;
#endif
    }

public:
    void Update(bool fFrameEnd_);
    void FrameEnd() override;

    void Out(uint16_t wPort_, uint8_t bVal_) override;

protected:
    CSAASound* m_pSAASound = nullptr;
};


class CDAC final : public CSoundDevice
{
public:
    CDAC();

public:
    void Reset() override;

    void FrameEnd() override;

    void OutputLeft(uint8_t bVal_);
    void OutputRight(uint8_t bVal_);
    void OutputLeft2(uint8_t bVal_);
    void OutputRight2(uint8_t bVal_);
    void Output(uint8_t bVal_);
    void Output2(uint8_t bVal_);

    int GetSamplesSoFar();

protected:
    Blip_Buffer buf_left{}, buf_right{};
    Blip_Synth<blip_med_quality, 256> synth_left{}, synth_right{}, synth_left2{}, synth_right2{};
};

// Spectrum-style BEEPer
class CBeeperDevice final : public CIoDevice
{
public:
    void Out(uint16_t wPort_, uint8_t bVal_) override;
};


extern CSAA* pSAA;
extern CDAC* pDAC;
