// Part of SimCoupe - A SAM Coupe emulator
//
// Disk.cpp: C++ classes used for accessing all SAM disk image types
//
//  Copyright (c) 1999-2015 Simon Owen
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

// Notes:
//  The CFloppyDisk implementation is OS-specific, and is in Floppy.cpp

#include "SimCoupe.h"
#include "Disk.h"

#include "Drive.h"
#include "Floppy.h"
#include "Util.h"

////////////////////////////////////////////////////////////////////////////////

/*static*/ int CDisk::GetType(CStream* pStream_)
{
    if (pStream_)
    {
        // Try each type in turn to check for a match
        if (CFloppyDisk::IsRecognised(pStream_))
            return dtFloppy;
        else if (CEDSKDisk::IsRecognised(pStream_))
            return dtEDSK;
        else if (CSADDisk::IsRecognised(pStream_))
            return dtSAD;
        else if (CFileDisk::IsRecognised(pStream_))
        {
            // For now we'll only accept single files if they have a .sbt or .sbt.gz file extension
            const char* pcsz = pStream_->GetFile();
            if (strlen(pcsz) > 4 && (!strcasecmp(pcsz + strlen(pcsz) - 4, ".sbt") || !strcasecmp(pcsz + strlen(pcsz) - 4, ".sbt.gz")))
                return dtSBT;
        }

        // The MGT format has no signature, so we try it last
        if (CMGTDisk::IsRecognised(pStream_))
            return dtMGT;
    }

    // Not recognised
    return dtUnknown;
}

/*static*/ CDisk* CDisk::Open(const char* pcszDisk_, bool fReadOnly_/*=false*/)
{
    CDisk* pDisk = nullptr;

    // Fetch stream for the disk source
    CStream* pStream = CStream::Open(pcszDisk_, fReadOnly_);

    // A disk will only be returned if the stream format is recognised
    if (pStream)
    {
        switch (GetType(pStream))
        {
        case dtFloppy:  pDisk = new CFloppyDisk(pStream);   break;      // Direct floppy access
        case dtEDSK:    pDisk = new CEDSKDisk(pStream);     break;      // .DSK
        case dtSAD:     pDisk = new CSADDisk(pStream);      break;      // .SAD
        case dtMGT:     pDisk = new CMGTDisk(pStream);      break;      // .MGT
        case dtSBT:     pDisk = new CFileDisk(pStream);     break;      // .SBT (bootable SAM file on a floppy)
        }
    }

    // Unrecognised, so close the underlying stream
    if (!pDisk)
        delete pStream;

    // Return the disk pointer (or nullptr)
    return pDisk;
}

/*static*/ CDisk* CDisk::Open(void* pv_, size_t uSize_, const char* pcszDisk_)
{
    CStream* pStream = new CMemStream(pv_, uSize_, pcszDisk_);
    return pStream ? new CFileDisk(pStream) : nullptr;
}


CDisk::CDisk(CStream* pStream_, int nType_)
    : m_nType(nType_), m_nBusy(0), m_fModified(false), m_pStream(pStream_), m_pbData(nullptr)
{
}

CDisk::~CDisk()
{
    // Delete the stream object and disk data memory we allocated
    delete m_pStream;
    delete[] m_pbData;
}


// Get the header for the specified sector index
bool CDisk::GetSector(uint8_t cyl_, uint8_t head_, uint8_t index_, IDFIELD* pID_/*=nullptr*/, uint8_t* pbStatus_/*=nullptr*/)
{
    // Construct a normal ID field for the sector
    pID_->bTrack = cyl_;
    pID_->bSide = head_;
    pID_->bSector = index_ + 1;
    pID_->bSize = 2;           // 128 << 2 = 512 bytes

    // Calculate and set the CRC for the ID field, including the 3 gap bytes and address mark
    auto wCRC = CrcBlock("\xa1\xa1\xa1\xfe", 4);
    wCRC = CrcBlock(pID_, 4, wCRC);
    pID_->bCRC1 = wCRC >> 8;
    pID_->bCRC2 = wCRC & 0xff;

    // Sector OK so reset all errors
    if (pbStatus_)
        *pbStatus_ = 0;

    // Always successful
    return true;
}

////////////////////////////////////////////////////////////////////////////////

/*static*/ bool CMGTDisk::IsRecognised(CStream* pStream_)
{
    size_t uSize = pStream_->GetSize();

    // Accept 800K (10-sector) SAM disks and 720K (9-sector DOS) disks
    return uSize == MGT_IMAGE_SIZE || uSize == DOS_IMAGE_SIZE;
}

CMGTDisk::CMGTDisk(CStream* pStream_, unsigned int uSectors_/*=NORMAL_DISK_SECTORS*/)
    : CDisk(pStream_, dtMGT)
{
    // Allocate some memory and clear it, just in case it's not a complete MGT image
    m_pbData = new uint8_t[MGT_IMAGE_SIZE];
    memset(m_pbData, (uSectors_ == NORMAL_DISK_SECTORS) ? 0x00 : 0xe5, MGT_IMAGE_SIZE);

    // Read the data from any existing stream
    if (pStream_->IsOpen())
    {
        pStream_->Rewind();
        size_t uRead = pStream_->Read(m_pbData, MGT_IMAGE_SIZE);
        Close();

        // If it's an MS-DOS image, treat as 9 sectors-per-track, otherwise 10 as normal for SAM
        m_uSectors = (uRead == DOS_IMAGE_SIZE) ? DOS_DISK_SECTORS : NORMAL_DISK_SECTORS;
    }
}


// Get sector details
bool CMGTDisk::GetSector(uint8_t cyl_, uint8_t head_, uint8_t index_, IDFIELD* pID_, uint8_t* pbStatus_)
{
    // Check sector is in range
    if (cyl_ >= NORMAL_DISK_TRACKS || head_ >= NORMAL_DISK_SIDES || index_ >= m_uSectors)
        return false;

    // Fill default values
    CDisk::GetSector(cyl_, head_, index_, pID_, pbStatus_);

    return true;
}

// Read the data for the last sector found
uint8_t CMGTDisk::ReadData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    // Sector must be in range
    if (index_ >= m_uSectors)
        return RECORD_NOT_FOUND;

    // Work out the offset for the required data
    long lPos = (head_ + NORMAL_DISK_SIDES * cyl_) * (m_uSectors * NORMAL_SECTOR_SIZE) + (index_ * NORMAL_SECTOR_SIZE);

    // Copy the sector data from the image buffer
    memcpy(pbData_, m_pbData + lPos, *puSize_ = NORMAL_SECTOR_SIZE);

    // Data is always perfect on MGT images, so return OK
    return 0;
}

// Read the data for the last sector found
uint8_t CMGTDisk::WriteData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    // Sector must be in range
    if (index_ >= m_uSectors)
        return RECORD_NOT_FOUND;

    // Fail if read-only
    if (IsReadOnly())
        return WRITE_PROTECT;

    // Work out the offset for the required data (MGT and IMG use different track interleaves)
    long lPos = head_ + NORMAL_DISK_SIDES * cyl_;
    lPos = lPos * (m_uSectors * NORMAL_SECTOR_SIZE) + (index_ * NORMAL_SECTOR_SIZE);

    // Copy the sector data to the image buffer, and set the modified flag
    memcpy(m_pbData + lPos, pbData_, *puSize_ = NORMAL_SECTOR_SIZE);
    SetModified();

    // Data is always perfect on MGT images, so return OK
    return 0;
}

// Save the disk out to the stream
bool CMGTDisk::Save()
{
    size_t uSize = NORMAL_DISK_SIDES * NORMAL_DISK_TRACKS * m_uSectors * NORMAL_SECTOR_SIZE;

    // Write the image out as a single block
    if (!m_pStream->Rewind() || m_pStream->Write(m_pbData, uSize) != uSize)
        return false;

    m_pStream->Close();
    SetModified(false);

    return true;
}

// Format a track using the specified format
uint8_t CMGTDisk::FormatTrack(uint8_t cyl_, uint8_t head_, IDFIELD* paID_, uint8_t* papbData_[], unsigned int uSectors_)
{
    uint32_t dwSectors = 0;
    bool fNormal = true;
    unsigned int u;

    // Disk must be writable, same number of sectors, and within track limit
    if (IsReadOnly() || uSectors_ != m_uSectors || cyl_ >= NORMAL_DISK_TRACKS)
        return WRITE_PROTECT;

    // Make sure the remaining sectors are completely normal
    for (u = 0; u < uSectors_; u++)
    {
        // Side and track must match the ones it's being laid on
        fNormal &= (paID_[u].bSide == head_ && paID_[u].bTrack == cyl_);

        // Sector size must be the same
        fNormal &= ((128U << paID_[u].bSize) == NORMAL_SECTOR_SIZE);

        // Remember we've seen this sector number
        dwSectors |= (1 << (paID_[u].bSector - 1));
    }

    // There must be only 1 of each sector number from 1 to N (in any order though)
    fNormal &= (dwSectors == ((1UL << m_uSectors) - 1));

    // Reject tracks that are not completely normal
    if (!fNormal)
        return WRITE_PROTECT;

    // Work out the offset for the required track
    long lPos = (head_ + NORMAL_DISK_SIDES * cyl_) * (m_uSectors * NORMAL_SECTOR_SIZE);

    // Process each sector to write the supplied data
    for (u = 0; u < uSectors_; u++)
        memcpy(m_pbData + lPos + ((paID_[u].bSector - 1) * NORMAL_SECTOR_SIZE), papbData_[u], NORMAL_SECTOR_SIZE);

    SetModified();
    return 0;
}

////////////////////////////////////////////////////////////////////////////////

/*static*/ bool CSADDisk::IsRecognised(CStream* pStream_)
{
    SAD_HEADER sh{};

    // Read the header, check for the signature, and make sure the disk geometry is sensible
    bool fValid = (pStream_->Rewind() && pStream_->Read(&sh, sizeof(sh)) == sizeof(sh) &&
        !memcmp(sh.abSignature, SAD_SIGNATURE, sizeof(sh.abSignature)) &&
        sh.bSides && sh.bSides <= MAX_DISK_SIDES && sh.bTracks && sh.bTracks <= 127 &&
        sh.bSectorSizeDiv64 && (sh.bSectorSizeDiv64 <= (MAX_SECTOR_SIZE >> 6)) &&
        (sh.bSectorSizeDiv64 & -sh.bSectorSizeDiv64) == sh.bSectorSizeDiv64);

    // If we know the stream size, validate the image size
    if (fValid && pStream_->GetSize())
    {
        unsigned int uDiskSize = sizeof(sh) + sh.bSides * sh.bTracks * sh.bSectors * (sh.bSectorSizeDiv64 << 6);
        fValid &= (pStream_->GetSize() == uDiskSize);
    }

    return fValid;
}

CSADDisk::CSADDisk(CStream* pStream_, unsigned int uSides_/*=NORMAL_DISK_SIDES*/, unsigned int uTracks_/*=NORMAL_DISK_TRACKS*/,
    unsigned int uSectors_/*=NORMAL_DISK_SECTORS*/, unsigned int uSectorSize_/*=NORMAL_SECTOR_SIZE*/)
    : CDisk(pStream_, dtSAD)
{
    SAD_HEADER sh;
    sh.bSides = static_cast<uint8_t>(uSides_);
    sh.bTracks = static_cast<uint8_t>(uTracks_);
    sh.bSectors = static_cast<uint8_t>(uSectors_);
    sh.bSectorSizeDiv64 = static_cast<uint8_t>(uSectorSize_ >> 6);

    if (!pStream_->IsOpen())
        memcpy(sh.abSignature, SAD_SIGNATURE, sizeof(sh.abSignature));
    else
    {
        pStream_->Rewind();
        pStream_->Read(&sh, sizeof(sh));
    }

    m_uSides = sh.bSides;
    m_uTracks = sh.bTracks;
    m_uSectors = sh.bSectors;
    m_uSectorSize = sh.bSectorSizeDiv64 << 6;

    unsigned int uDiskSize = sizeof(sh) + m_uSides * m_uTracks * m_uSectors * m_uSectorSize;
    memcpy(m_pbData = new uint8_t[uDiskSize], &sh, sizeof(sh));
    memset(m_pbData + sizeof(sh), 0, uDiskSize - sizeof(sh));

    if (pStream_->IsOpen())
    {
        pStream_->Read(m_pbData + sizeof(sh), uDiskSize - sizeof(sh));
        pStream_->Close();
    }
}


// Get sector details
bool CSADDisk::GetSector(uint8_t cyl_, uint8_t head_, uint8_t index_, IDFIELD* pID_, uint8_t* pbStatus_)
{
    // Check sector is in range
    if (cyl_ >= m_uTracks || head_ >= m_uSides || index_ >= m_uSectors)
        return false;

    // Fill default values, then update the sector size
    CDisk::GetSector(cyl_, head_, index_, pID_, pbStatus_);
    pID_->bSize = GetSizeCode(m_uSectorSize);

    return true;
}

// Read the data for the last sector found
uint8_t CSADDisk::ReadData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    // Work out the offset for the required data
    long lPos = sizeof(SAD_HEADER) + (head_ * m_uTracks + cyl_) * (m_uSectors * m_uSectorSize) + (index_ * m_uSectorSize);

    // Copy the sector data from the image buffer
    memcpy(pbData_, m_pbData + lPos, *puSize_ = m_uSectorSize);

    // Data is always perfect on SAD images, so return OK
    return 0;
}

// Read the data for the last sector found
uint8_t CSADDisk::WriteData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    // Fail if read-only
    if (IsReadOnly())
        return WRITE_PROTECT;

    // Work out the offset for the required data
    long lPos = sizeof(SAD_HEADER) + (head_ * m_uTracks + cyl_) * (m_uSectors * m_uSectorSize) + (index_ * m_uSectorSize);

    // Copy the sector data to the image buffer, and set the modified flag
    memcpy(m_pbData + lPos, pbData_, *puSize_ = m_uSectorSize);
    SetModified();

    // Data is always perfect on SAD images, so return OK
    return 0;
}

// Save the disk out to the stream
bool CSADDisk::Save()
{
    unsigned int uDiskSize = sizeof(SAD_HEADER) + m_uSides * m_uTracks * m_uSectors * m_uSectorSize;

    if (!m_pStream->Rewind() || m_pStream->Write(m_pbData, uDiskSize) != uDiskSize)
        return false;

    SetModified(false);
    m_pStream->Close();
    return true;
}

// Format a track using the specified format
uint8_t CSADDisk::FormatTrack(uint8_t cyl_, uint8_t head_, IDFIELD* paID_, uint8_t* papbData_[], unsigned int uSectors_)
{
    uint32_t dwSectors = 0;
    bool fNormal = true;
    unsigned int u;

    // Disk must be writable, same number of sectors, and within track limit
    if (IsReadOnly() || uSectors_ != m_uSectors || cyl_ >= m_uTracks)
        return WRITE_PROTECT;

    // Make sure the remaining sectors are completely normal
    for (u = 0; u < uSectors_; u++)
    {
        // Side and track must match the ones it's being laid on
        fNormal &= (paID_[u].bSide == head_ && paID_[u].bTrack == cyl_);

        // Sector size must be the same
        fNormal &= ((128U << paID_[u].bSize) == m_uSectorSize);

        // Remember we've seen this sector number
        dwSectors |= (1 << (paID_[u].bSector - 1));
    }

    // There must be only 1 of each sector number from 1 to N (in any order though)
    fNormal &= (dwSectors == ((1UL << m_uSectors) - 1));

    // Reject tracks that are not completely normal
    if (!fNormal)
        return WRITE_PROTECT;

    // Work out the offset for the required track
    long lPos = sizeof(SAD_HEADER) + (head_ * m_uTracks + cyl_) * (m_uSectors * NORMAL_SECTOR_SIZE);

    // Process each sector to write the supplied data
    for (u = 0; u < uSectors_; u++)
        memcpy(m_pbData + lPos + ((paID_[u].bSector - 1) * m_uSectorSize), papbData_[u], m_uSectorSize);

    // Mark the disk stream as modified
    SetModified();

    return 0;
}
////////////////////////////////////////////////////////////////////////////////

/*static*/ bool CEDSKDisk::IsRecognised(CStream* pStream_)
{
    EDSK_HEADER eh;

    // Read the header, check the signature and basic geometry
    bool fValid = (pStream_->Rewind() && pStream_->Read(&eh, sizeof(eh)) == sizeof(eh) &&
        (!memcmp(eh.szSignature, EDSK_SIGNATURE, sizeof(EDSK_SIGNATURE) - 1) ||
            !memcmp(eh.szSignature, DSK_SIGNATURE, sizeof(DSK_SIGNATURE) - 1)) &&
        eh.bSides >= 1 && eh.bSides <= MAX_DISK_SIDES);

    return fValid;
}

CEDSKDisk::CEDSKDisk(CStream* pStream_, unsigned int uSides_/*=NORMAL_DISK_SIDES*/, unsigned int uTracks_/*=MAX_DISK_TRACKS*/)
    : CDisk(pStream_, dtEDSK), m_pSector(nullptr), m_pbData(nullptr)
{
    m_uSides = uSides_;
    m_uTracks = uTracks_;

    // Unformatted, initially
    memset(m_apTracks, 0, sizeof(m_apTracks));
    memset(m_abSizes, 0, sizeof(m_abSizes));

    // There's nothing more to do if we don't have a stream
    if (!pStream_->IsOpen())
        return;

    uint8_t ab[256];
    EDSK_HEADER* peh = reinterpret_cast<EDSK_HEADER*>(ab);
    uint8_t* pbSizes = reinterpret_cast<uint8_t*>(peh + 1);

    pStream_->Rewind();
    pStream_->Read(ab, sizeof(ab));

    m_uSides = peh->bSides;
    m_uTracks = std::min(peh->bTracks, static_cast<uint8_t>(MAX_DISK_TRACKS));

    bool fEDSK = peh->szSignature[0] == EDSK_SIGNATURE[0];
    uint16_t wDSKTrackSize = peh->abTrackSize[0] | (peh->abTrackSize[1] << 8);  // DSK only

    for (uint8_t cyl = 0; cyl < m_uTracks; cyl++)
    {
        for (uint8_t head = 0; head < m_uSides; head++)
        {
            // Nothing to do for empty tracks
            unsigned int size = fEDSK ? (pbSizes[cyl * m_uSides + head] << 8) : wDSKTrackSize;
            if (!size)
                continue;

            uint8_t* pb = new uint8_t[size];
            EDSK_TRACK* pt = reinterpret_cast<EDSK_TRACK*>(pb);

            // Read the track, rejecting anything but 250Kbps MFM
            if (pt && (pStream_->Read(pt, size) != size || (pt->bRate && pt->bRate != 1) || (pt->bEncoding && pt->bEncoding != 1)))
            {
                delete[] pb;
                pt = nullptr;
                size = 0;
            }

            // Save the track (or nullptr) and size MSB
            m_apTracks[head][cyl] = pt;
            m_abSizes[head][cyl] = size >> 8;
        }
    }

    pStream_->Close();
}

CEDSKDisk::~CEDSKDisk()
{
    // Free any allocated tracks
    for (uint8_t cyl = 0; cyl < m_uTracks; cyl++)
        for (uint8_t head = 0; head < m_uSides; head++)
            delete[] m_apTracks[head][cyl];
}

// Find the next sector in the current track
bool CEDSKDisk::GetSector(uint8_t cyl_, uint8_t head_, uint8_t index_, IDFIELD* pID_, uint8_t* pbStatus_)
{
    if (cyl_ >= m_uTracks || head_ >= m_uSides)
        return false;

    EDSK_TRACK* pTrack = m_apTracks[head_][cyl_];
    m_pSector = reinterpret_cast<EDSK_SECTOR*>(pTrack + 1);
    m_pbData = reinterpret_cast<uint8_t*>(m_pSector + EDSK_MAX_SECTORS);

    // Check the track exists and the sector is in range
    if (!pTrack || index_ >= pTrack->bSectors)
        return false;

    // Advance to the required sector and its data field
    for (int i = 0; i < index_; i++)
    {
        m_pbData += (m_pSector->bDatahigh << 8) | m_pSector->bDatalow;
        m_pSector++;
    }

    // Complete the ID field
    pID_->bSide = m_pSector->bSide;
    pID_->bTrack = m_pSector->bTrack;
    pID_->bSector = m_pSector->bSector;
    pID_->bSize = m_pSector->bSize;
    pID_->bCRC1 = pID_->bCRC2 = 0;

    // Calculate and set the CRC for the ID field, including the 3 gap bytes and address mark
    auto wCRC = CrcBlock("\xa1\xa1\xa1\xfe", 4);
    wCRC = CrcBlock(pID_, 4, wCRC);
    if (m_pSector->bStatus1 & ST1_765_CRC_ERROR) wCRC ^= 0x5555;  // Force an error if required
    pID_->bCRC1 = wCRC >> 8;
    pID_->bCRC2 = wCRC & 0xff;

    // Set the ID status
    *pbStatus_ = (m_pSector->bStatus1 & ST1_765_CRC_ERROR) ? CRC_ERROR : 0;

    return true;
}

// Read the data for the last sector found
uint8_t CEDSKDisk::ReadData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    IDFIELD id;
    uint8_t bStatus;

    if (!GetSector(cyl_, head_, index_, &id, &bStatus) || !m_pSector || !m_pbData)
        return RECORD_NOT_FOUND;

    // Read the data field
    unsigned int uDataSize = 128U << (m_pSector->bSize & 3);  // clip to a 3 bit size
    memcpy(pbData_, m_pbData, *puSize_ = uDataSize);

    // Form the data status
    bStatus = 0;
    if (m_pSector->bStatus2 & ST2_765_DATA_NOT_FOUND) bStatus |= RECORD_NOT_FOUND;
    if (m_pSector->bStatus2 & ST2_765_CRC_ERROR) { bStatus |= CRC_ERROR; }
    if (m_pSector->bStatus2 & ST2_765_CONTROL_MARK)   bStatus |= DELETED_DATA;

    return bStatus;
}

// Read the data for the last sector found
uint8_t CEDSKDisk::WriteData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    IDFIELD id;
    uint8_t bStatus;

    if (!GetSector(cyl_, head_, index_, &id, &bStatus) || !m_pSector || !m_pbData)
        return RECORD_NOT_FOUND;

    if (IsReadOnly())
        return WRITE_PROTECT;

    // Write the data field
    unsigned int uDataSize = 128U << (m_pSector->bSize & 3);
    memcpy(m_pbData, pbData_, *puSize_ = uDataSize);

    // Clean any CRC error on the old sector
    m_pSector->bStatus1 &= ~ST1_765_CRC_ERROR;
    m_pSector->bStatus2 &= ~ST2_765_CRC_ERROR;

    SetModified();
    return 0;
}

// Save the disk out to the stream
bool CEDSKDisk::Save()
{
    uint8_t abHeader[256] = { 0 }, cyl, head;
    EDSK_HEADER* peh = reinterpret_cast<EDSK_HEADER*>(abHeader);
    uint8_t* pbSizes = reinterpret_cast<uint8_t*>(peh + 1);

    // Complete the disk header
    memcpy(peh->szSignature, EDSK_SIGNATURE, sizeof(peh->szSignature));
    memcpy(peh->szCreator, "SimCoupe 1.1 ", sizeof(peh->szCreator)); // note: trailing space+null fills field
    peh->bTracks = m_uTracks;
    peh->bSides = m_uSides;

    // Complete the MSB size table
    for (cyl = 0; cyl < m_uTracks; cyl++)
        for (head = 0; head < m_uSides; head++)
            *pbSizes++ = m_abSizes[head][cyl];

    // Write the disk header
    bool fSuccess = m_pStream->Rewind() && m_pStream->Write(&abHeader, sizeof(abHeader)) == sizeof(abHeader);

    // Write the track data
    for (cyl = 0; fSuccess && cyl < m_uTracks; cyl++)
    {
        for (head = 0; head < m_uSides; head++)
        {
            // Nothing to write?
            if (!m_apTracks[head][cyl])
                continue;

            unsigned int uSize = m_abSizes[head][cyl] << 8;
            fSuccess &= (m_pStream->Write(m_apTracks[head][cyl], uSize) == uSize);
        }
    }

    // Any problems?
    if (!fSuccess)
        return false;

    m_pStream->Close();
    SetModified(false);

    return true;
}

// Format a track using the specified format
uint8_t CEDSKDisk::FormatTrack(uint8_t cyl_, uint8_t head_, IDFIELD* paID_, uint8_t* papbData_[], unsigned int uSectors_)
{
    unsigned int u, uDataTotal = 0;

    // Disk must be writeable and within track limit
    if (IsReadOnly() || cyl_ >= MAX_DISK_TRACKS || head_ >= MAX_DISK_SIDES)
        return WRITE_PROTECT;

    // The 256-byte track header limits the number of sectors it can hold
    if (uSectors_ > EDSK_MAX_SECTORS)
        return WRITE_PROTECT;

    // Add up the space needed for data fields
    for (u = 0; u < uSectors_; u++)
        uDataTotal += (128U << (paID_[u].bSize & 7));

    // Make sure the track contents are within the maximum size
    if ((uDataTotal + (62 + 1) * uSectors_) >= MAX_TRACK_SIZE)
        return WRITE_PROTECT;

    // Calculate the total track size, including header and padding up to the next 256-byte boundary
    uDataTotal = (sizeof(EDSK_TRACK) + uDataTotal + 0xff) & ~0xff;

    // EDSK limits the total track length to 0xff00, as only the high byte of the size is stored
    // This should always fit due to the above tests, but we may as well be sure
    if (uDataTotal > 0xff00)
        return WRITE_PROTECT;

    // Allocate space for the new track
    auto pb = new uint8_t[uDataTotal];
    memset(pb, 0, uDataTotal);

    // Set up the initial track/sector/data pointers
    EDSK_TRACK* pt = reinterpret_cast<EDSK_TRACK*>(pb);
    EDSK_SECTOR* ps = reinterpret_cast<EDSK_SECTOR*>(pt + 1);
    pb += 256;

    // Complete a suitable track header
    memcpy(pt->szSignature, EDSK_TRACK_SIGNATURE, sizeof(pt->szSignature));
    pt->bRate = 0;              // default, which is 250Kbps
    pt->bEncoding = 0;          // default, which is MFM
    pt->bTrack = cyl_;          // physical cylinder
    pt->bSide = head_;          // physical head
    pt->bSize = 2;              // dummy, we'll use 512-bytes
    pt->bSectors = uSectors_;   // sectors per track
    pt->bGap3 = 78;             // dummy, we'll use the normal EDSK value
    pt->bFill = 0x00;           // zero filler

    // Write each of the supplied sectors
    for (u = 0; u < uSectors_; u++)
    {
        // Complete the ID header, with no status conditions flagged
        ps[u].bTrack = paID_[u].bTrack;
        ps[u].bSide = paID_[u].bSide;
        ps[u].bSector = paID_[u].bSector;
        ps[u].bSize = paID_[u].bSize;
        ps[u].bStatus1 = ps[u].bStatus2 = 0;

        unsigned int uDataSize = (128U << (paID_[u].bSize & 7));
        ps[u].bDatalow = uDataSize & 0xff;
        ps[u].bDatahigh = uDataSize >> 8;

        // Copy the sector data, advancing the pointer for the next one
        memcpy(pb, papbData_[u], uDataSize);
        pb += uDataSize;
    }

    // Delete any old track, and assign the new one
    delete m_apTracks[head_][cyl_];
    m_apTracks[head_][cyl_] = pt;
    m_abSizes[head_][cyl_] = uDataTotal >> 8;

    // Update the disk extents if required
    if (cyl_ >= m_uTracks) m_uTracks = cyl_ + 1;
    if (head_ >= m_uSides) m_uSides = head_ + 1;

    // Mark the disk stream as modified
    SetModified();

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

/*static*/ bool CFloppyDisk::IsRecognised(CStream* pStream_)
{
    return CFloppyStream::IsRecognised(pStream_->GetPath());
}

CFloppyDisk::CFloppyDisk(CStream* pStream_)
    : CDisk(pStream_, dtFloppy), m_bCommand(0), m_bStatus(0)
{
    // We can assume this as only CFloppyStream will recognise the real device, and we are always the disk used
    m_pFloppy = reinterpret_cast<CFloppyStream*>(pStream_);

    m_pbData = new uint8_t[MAX_TRACK_SIZE];
    m_pTrack = reinterpret_cast<TRACK*>(m_pbData);
    m_pSector = reinterpret_cast<SECTOR*>(m_pTrack + 1);

    // Invalidate track cache
    m_pTrack->cyl = m_pTrack->head = 0xff;
}


// Find the next sector in the current track
bool CFloppyDisk::GetSector(uint8_t cyl_, uint8_t head_, uint8_t index_, IDFIELD* pID_, uint8_t* pbStatus_)
{
    // Check sector is in range
    if (cyl_ != m_pTrack->cyl || head_ != m_pTrack->head || index_ >= m_pTrack->sectors)
        return false;

    auto ps = &m_pSector[index_];

    // Construct a normal ID field for the sector
    pID_->bSide = ps->head;
    pID_->bTrack = ps->cyl;
    pID_->bSector = ps->sector;
    pID_->bSize = ps->size;

    // Calculate and set the CRC for the ID field, including the 3 gap bytes and address mark
    auto wCRC = CrcBlock("\xa1\xa1\xa1\xfe", 4);
    wCRC = CrcBlock(pID_, 4, wCRC);
    if (ps->status & CRC_ERROR) wCRC ^= 0x5555; // Force an error if required
    pID_->bCRC1 = wCRC >> 8;
    pID_->bCRC2 = wCRC & 0xff;

    *pbStatus_ = ps->status;

    return true;
}

// Read the data for the last sector found
uint8_t CFloppyDisk::ReadData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    if (cyl_ != m_pTrack->cyl || head_ != m_pTrack->head || index_ >= m_pTrack->sectors)
        return RECORD_NOT_FOUND;

    // Read from the cached track
    auto ps = &m_pSector[index_];
    memcpy(pbData_, ps->pbData, *puSize_ = (128 << (ps->size & 3)));

    return ps->status;
}

// Write the data for the last sector found
uint8_t CFloppyDisk::WriteData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    if (cyl_ != m_pTrack->cyl || head_ != m_pTrack->head || index_ >= m_pTrack->sectors)
        return RECORD_NOT_FOUND;

    // Write to the track cache, assuming it will work for now
    auto ps = &m_pSector[index_];
    memcpy(ps->pbData, pbData_, *puSize_ = (128U << (ps->size & 3)));
    ps->status &= ~CRC_ERROR;

    // Start the write command
    m_bCommand = WRITE_1SECTOR;
    return m_bStatus = m_pFloppy->StartCommand(m_bCommand, m_pTrack, index_);
}

// Save the disk out to the stream
bool CFloppyDisk::Save()
{
    // Writes are live, so there's nothing to save
    return true;
}

// Format a track using the specified format
uint8_t CFloppyDisk::FormatTrack(uint8_t cyl_, uint8_t head_, IDFIELD* paID_, uint8_t* papbData_[], unsigned int uSectors_)
{
    unsigned int uSize = uSectors_ ? (128U << (paID_->bSize & 7)) : 0, u;

    // Disk must be writeable and within track limit
    if (IsReadOnly() || cyl_ >= MAX_DISK_TRACKS)
        return WRITE_PROTECT;

    // For now, ensure all the sectors are the same size
    for (u = 1; u < uSectors_; u++)
        if (paID_[u].bSize != paID_->bSize)
            return WRITE_PROTECT;

    // Make sure the track contents aren't too big to format
    if (((62 + uSize + 1) * uSectors_) >= (MAX_TRACK_SIZE - 50))
        return WRITE_PROTECT;

    // Set up the initial track/sector/data pointers
    TRACK* pt = m_pTrack;
    SECTOR* ps = reinterpret_cast<SECTOR*>(pt + 1);
    auto pb = reinterpret_cast<uint8_t*>(ps + uSectors_);

    // Complete a suitable track header
    pt->sectors = uSectors_;
    pt->cyl = cyl_;
    pt->head = head_;

    // Prepare each of the supplied sectors
    for (u = 0; u < uSectors_; u++)
    {
        // Complete the ID header, with no status conditions flagged
        ps[u].cyl = paID_[u].bTrack;
        ps[u].head = paID_[u].bSide;
        ps[u].sector = paID_[u].bSector;
        ps[u].size = paID_[u].bSize;
        ps[u].status = 0;
        ps[u].pbData = pb;

        // Copy the sector data, advancing the pointer for the next one
        memcpy(pb, papbData_[u], uSize);
        pb += uSize;
    }

    // Start the format command
    m_bCommand = WRITE_TRACK;
    return m_bStatus = m_pFloppy->StartCommand(m_bCommand, m_pTrack);
}

uint8_t CFloppyDisk::LoadTrack(uint8_t cyl_, uint8_t head_)
{
    // Return if the track is already loaded
    if (cyl_ == m_pTrack->cyl && head_ == m_pTrack->head)
        return 0;

    // Prepare a fresh track
    m_pTrack->sectors = 0;
    m_pTrack->cyl = cyl_;
    m_pTrack->head = head_;

    // Read the track, setting busy if we've not finished yet
    m_bCommand = READ_MSECTOR;
    return m_bStatus = m_pFloppy->StartCommand(m_bCommand, m_pTrack);
}

// Get the status of the current asynchronous operation, if any
bool CFloppyDisk::IsBusy(uint8_t* pbStatus_, bool fWait_)
{
    bool fBusy = m_pFloppy->IsBusy(pbStatus_, fWait_);

    // If we've just finished a request, update the track cache
    if (!fBusy && m_bStatus == BUSY)
    {
        // Invaliate the track cache if the command failed
        if (*pbStatus_)
            m_pTrack->head = 0xff;

        // Clear the FDC status
        m_bStatus = 0;
    }

    return fBusy;
}

////////////////////////////////////////////////////////////////////////////////

/*static*/ bool CFileDisk::IsRecognised(CStream* pStream_)
{
    // Accept any file that isn't too big
    return pStream_->GetSize() <= MAX_SAM_FILE_SIZE;
}

CFileDisk::CFileDisk(CStream* pStream_)
    : CDisk(pStream_, dtFile), m_uSize(0)
{
    // Work out the maximum file size, allocate some memory for it
    unsigned int uSize = MAX_SAM_FILE_SIZE + DISK_FILE_HEADER_SIZE;
    m_pbData = new uint8_t[uSize];
    memset(m_pbData, 0, uSize);

    // Read the data from any existing stream, or show an empty disk as we don't support saving
    if (pStream_->IsOpen())
    {
        // Read in the file, leaving a 9-byte gap for the disk file header
        pStream_->Rewind();
        m_uSize = static_cast<unsigned int>(pStream_->Read(m_pbData + DISK_FILE_HEADER_SIZE, MAX_SAM_FILE_SIZE));
        pStream_->Close();

        // Create the disk file header
        m_pbData[0] = 19;                           // CODE file type
        m_pbData[1] = m_uSize & 0xff;               // LSB of size mod 16384
        m_pbData[2] = (m_uSize >> 8) & 0xff;        // MSB of size mod 16384
        m_pbData[3] = 0x00;                         // LSB of offset start
        m_pbData[4] = 0x80;                         // MSB of offset start
        m_pbData[5] = 0xff;                         // Unused
        m_pbData[6] = 0xff;                         // Unused
        m_pbData[7] = (m_uSize >> 14) & 0xff;       // Number of pages (size div 16384)
        m_pbData[8] = 0x01;                         // Starting page number

        m_uSize += DISK_FILE_HEADER_SIZE;
    }
}


// Get sector details
bool CFileDisk::GetSector(uint8_t cyl_, uint8_t head_, uint8_t index_, IDFIELD* pID_, uint8_t* pbStatus_)
{
    // Check sector is in range
    if (cyl_ >= NORMAL_DISK_TRACKS || head_ >= NORMAL_DISK_SIDES || index_ >= NORMAL_DISK_SECTORS)
        return false;

    // Fill default values
    CDisk::GetSector(cyl_, head_, index_, pID_, pbStatus_);

    return true;
}

// Read the data for the last sector found
uint8_t CFileDisk::ReadData(uint8_t cyl_, uint8_t head_, uint8_t index_, uint8_t* pbData_, unsigned int* puSize_)
{
    // Clear the sector out
    memset(pbData_, 0, *puSize_ = NORMAL_SECTOR_SIZE);

    // The first directory sector?
    if (cyl_ == 0 && head_ == 0 && index_ == 0)
    {
        // CODE file type
        pbData_[0] = 19;

        // Use a fixed filename, starting with "auto" so SimCoupe's embedded DOS boots it
        memcpy(pbData_ + 1, "autoExec  ", 10);

        // Number of sectors required
        uint16_t wSectors = (m_uSize + NORMAL_SECTOR_SIZE - 3) / (NORMAL_SECTOR_SIZE - 2);
        pbData_[11] = wSectors >> 8;
        pbData_[12] = wSectors & 0xff;

        // Starting track and sector
        pbData_[13] = NORMAL_DIRECTORY_TRACKS;
        pbData_[14] = 1;

        // Sector address map
        memset(pbData_ + 15, 0xff, wSectors >> 3);
        if (wSectors & 7)
            pbData_[15 + (wSectors >> 3)] = (1U << (wSectors & 7)) - 1;

        // Starting page number and offset
        pbData_[236] = m_pbData[8];
        pbData_[237] = m_pbData[3];
        pbData_[238] = m_pbData[4];

        // Size in pages and mod 16384
        pbData_[239] = m_pbData[7];
        pbData_[240] = m_pbData[1];
        pbData_[241] = m_pbData[2];

        // Auto-execute code
        pbData_[242] = 2;  // Normal paging (see PDPSUBR in ROM0 for details)
        pbData_[243] = m_pbData[3];
        pbData_[244] = m_pbData[4];
    }

    // Does the position fall within the file?
    else if (cyl_ >= NORMAL_DIRECTORY_TRACKS)
    {
        // Work out the offset for the required data
        unsigned int uPos = (head_ * NORMAL_DISK_TRACKS + cyl_ - NORMAL_DIRECTORY_TRACKS) *
            (NORMAL_DISK_SECTORS * (NORMAL_SECTOR_SIZE - 2)) + (index_ * (NORMAL_SECTOR_SIZE - 2));

        // Copy the file segment required
        memcpy(pbData_, m_pbData + uPos, std::min(NORMAL_SECTOR_SIZE - 2, m_uSize - uPos));

        // If there are more sectors in the file, set the chain to the next sector
        if (uPos + NORMAL_SECTOR_SIZE < m_uSize)
        {
            // Work out the next sector
            unsigned int uSector = 1 + ((index_ + 1) % NORMAL_DISK_SECTORS);
            unsigned int uTrack = ((uSector == 1) ? cyl_ + 1 : cyl_) % NORMAL_DISK_TRACKS;
            unsigned int uSide = (uTrack == 0 ? head_ + 1 : head_) % NORMAL_DISK_SIDES;

            pbData_[NORMAL_SECTOR_SIZE - 2] = uTrack + (uSide ? 0x80 : 0x00);
            pbData_[NORMAL_SECTOR_SIZE - 1] = uSector;
        }
    }

    // Data is always perfect
    return 0;
}
