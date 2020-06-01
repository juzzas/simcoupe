// Part of SimCoupe - A SAM Coupe emulator
//
// Stream.cpp: Data stream abstraction classes
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
//  Currently supports read-write access of uncompressed files, gzipped
//  files, and read-only zip archive access.
//
//  Access to real standard format disks is also supported where a
//  Floppy.cpp implementation exists.

// Todo:
//  - remove 32K file test done on zip archives (add a container layer?)
//  - maybe add support for updating zip archives

#include "SimCoupe.h"
#include "Stream.h"

#include "Disk.h"
#include "Floppy.h"
#include "Util.h"

////////////////////////////////////////////////////////////////////////////////

Stream::Stream(const char* pcszPath_, bool fReadOnly_/*=false*/)
    : m_fReadOnly(fReadOnly_)
{
    // Keep a copy of the stream source as we'll need it for saving
    m_pszPath = strdup(pcszPath_);
}

Stream::~Stream()
{
    free(m_pszPath);
    if (m_pszFile) free(m_pszFile);
}


// Identify the stream and create an object to supply data from it
/*static*/ std::unique_ptr<Stream> Stream::Open(const char* pcszPath_, bool fReadOnly_/*=false*/)
{
    // Reject empty strings immediately
    if (!pcszPath_ || !*pcszPath_)
        return nullptr;

    // Give the OS-specific floppy driver first go at the path
    if (FloppyStream::IsRecognised(pcszPath_))
        return std::make_unique<FloppyStream>(pcszPath_, fReadOnly_);

    // If the file is read-only, the stream will be read-only
    FILE* file = fopen(pcszPath_, "r+b");
    fReadOnly_ |= !file;
    if (file)
        fclose(file);

#ifdef HAVE_LIBZ
    // Try and open it as a zip file
    unzFile hfZip;
    if ((hfZip = unzOpen(pcszPath_)))
    {
        // Iterate through the contents of the zip looking for a file with a suitable size
        for (int nRet = unzGoToFirstFile(hfZip); nRet == UNZ_OK; nRet = unzGoToNextFile(hfZip))
        {
            unz_file_info sInfo;

            // Get details of the current file
            unzGetCurrentFileInfo(hfZip, &sInfo, nullptr, 0, nullptr, 0, nullptr, 0);

            // Continue looking if it's too small to be considered [strictly this shouldn't really be done here!]
            if (sInfo.uncompressed_size < 32768)
                continue;

            // Ok, so open and use the first file in the zip and use that
            if (unzOpenCurrentFile(hfZip) == UNZ_OK)
                return std::make_unique<ZipStream>(hfZip, pcszPath_, true/*fReadOnly_*/);  // ZIPs are currently read-only
        }

        // Failed to open the first file, so close the zip
        unzClose(hfZip);
    }
    else
#endif
    {
        // Open the file using the regular CRT file functions
        FILE* hf;
        if ((hf = fopen(pcszPath_, "rb")))
        {
#ifdef HAVE_LIBZ
            uint8_t abSig[sizeof(GZ_SIGNATURE)];
            if ((fread(abSig, 1, sizeof(abSig), hf) != sizeof(abSig)) || memcmp(abSig, GZ_SIGNATURE, sizeof(abSig)))
#endif
                return std::make_unique<FileStream>(hf, pcszPath_, fReadOnly_);
#ifdef HAVE_LIBZ
            else
            {
                uint8_t ab[4] = {};
                size_t uSize = 0;

                // Read the uncompressed size from the end of the file (if under 4GiB)
                if (fseek(hf, -4, SEEK_END) == 0 && fread(ab, 1, sizeof(ab), hf) == sizeof(ab))
                    uSize = ((size_t)ab[3] << 24) | ((size_t)ab[2] << 16) | ((size_t)ab[1] << 8) | (size_t)ab[0];

                // Close the file so we can open it through ZLib
                fclose(hf);

                // Try to open it as a gzipped file
                gzFile hfGZip;
                if ((hfGZip = gzopen(pcszPath_, "rb")))
                    return std::make_unique<ZLibStream>(hfGZip, pcszPath_, uSize, fReadOnly_);
            }
#endif  // HAVE_LIBZ
        }
    }

    // Couldn't handle what we were given :-/
    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////

FileStream::FileStream(FILE* hFile_, const char* pcszPath_, bool fReadOnly_/*=false*/)
    : Stream(pcszPath_, fReadOnly_), m_hFile(hFile_)
{
    struct stat st;

    if (hFile_ && !stat(pcszPath_, &st))
        m_uSize = static_cast<size_t>(st.st_size);

    for (const char* p = pcszPath_; *p; p++)
    {
        if (*p == PATH_SEPARATOR)
            pcszPath_ = p + 1;
    }

    m_pszFile = strdup(pcszPath_);
}

void FileStream::Close()
{
    if (m_hFile)
    {
        fclose(m_hFile);
        m_hFile = nullptr;
        m_nMode = modeClosed;
    }
}

bool FileStream::Rewind()
{
    if (IsOpen())
        Close();

    return true;
}

size_t FileStream::Read(void* pvBuffer_, size_t uLen_)
{
    if (m_nMode != modeReading)
    {
        // Close the file, if open for writing
        Close();

        // Open the file for writing, using compression if the source file did
        if ((m_hFile = fopen(m_pszPath, "rb")))
            m_nMode = modeReading;
    }

    size_t uRead = m_hFile ? fread(pvBuffer_, 1, uLen_, m_hFile) : 0;
    return uRead;
}

size_t FileStream::Write(void* pvBuffer_, size_t uLen_)
{
    if (m_nMode != modeWriting)
    {
        // Close the file, if open for reading
        Close();

        // Open the file for writing, using compression if the source file did
        if ((m_hFile = fopen(m_pszPath, "wb")))
            m_nMode = modeWriting;
    }

    return m_hFile ? fwrite(pvBuffer_, 1, uLen_, m_hFile) : 0;
}

////////////////////////////////////////////////////////////////////////////////

MemStream::MemStream(void* pv_, size_t uLen_, const char* pcszPath_)
    : Stream(pcszPath_, true)
{
    m_nMode = modeReading;
    m_uSize = uLen_;
    m_pbData = reinterpret_cast<uint8_t*>(pv_);
    m_pszFile = strdup(pcszPath_);
}

void MemStream::Close()
{
    m_nMode = modeClosed;
}

bool MemStream::Rewind()
{
    m_uPos = 0;
    return true;
}

size_t MemStream::Read(void* pvBuffer_, size_t uLen_)
{
    if (m_nMode != modeReading)
    {
        m_nMode = modeReading;
        m_uPos = 0;
    }

    size_t uRead = std::min(m_uSize - m_uPos, uLen_);
    memcpy(pvBuffer_, m_pbData + m_uPos, uRead);
    m_uPos += uRead;
    return uRead;
}

size_t MemStream::Write(void* /*pvBuffer_*/, size_t /*uLen_*/)
{
    m_nMode = modeWriting;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////

#ifdef HAVE_LIBZ

ZLibStream::ZLibStream(gzFile hFile_, const char* pcszPath_, size_t uSize_, bool fReadOnly_/*=false*/)
    : Stream(pcszPath_, fReadOnly_), m_hFile(hFile_), m_uSize(uSize_)
{
    for (const char* p = pcszPath_; *p; p++)
    {
        if (*p == PATH_SEPARATOR)
            pcszPath_ = p + 1;
    }

    char szFile[MAX_PATH + 7];
    strncpy(szFile, pcszPath_, MAX_PATH);
    strcat(szFile, " (gzip)");
    m_pszFile = strdup(szFile);
}

void ZLibStream::Close()
{
    if (m_hFile)
    {
        gzclose(m_hFile);
        m_hFile = nullptr;
        m_nMode = modeClosed;
    }
}


size_t ZLibStream::GetSize()
{
    // Do we need to determine the size?
    if (!m_uSize && IsOpen())
    {
        long lPos = gztell(m_hFile);
        gzrewind(m_hFile);

        uint8_t ab[512];
        int nRead;

        do {
            m_uSize += (nRead = gzread(m_hFile, ab, sizeof(ab)));
        } while (nRead > 0);

        gzseek(m_hFile, lPos, SEEK_SET);
    }

    return m_uSize;
}

bool ZLibStream::Rewind()
{
    return !IsOpen() || !gzrewind(m_hFile);
}

size_t ZLibStream::Read(void* pvBuffer_, size_t uLen_)
{
    if (m_nMode != modeReading)
    {
        // Close the file, if open for writing
        Close();

        // Open the file for writing, using compression if the source file did
        if ((m_hFile = gzopen(m_pszPath, "rb")))
            m_nMode = modeReading;
    }

    int nRead = m_hFile ? gzread(m_hFile, pvBuffer_, static_cast<unsigned>(uLen_)) : 0;
    return (nRead == -1) ? 0 : static_cast<size_t>(nRead);
}

size_t ZLibStream::Write(void* pvBuffer_, size_t uLen_)
{
    if (m_nMode != modeWriting)
    {
        // Close the file, if open for reading
        Close();

        // Open the file for writing, using compression if the source file did
        if ((m_hFile = gzopen(m_pszPath, "wb9")))
            m_nMode = modeWriting;
    }

    return m_hFile ? gzwrite(m_hFile, pvBuffer_, static_cast<unsigned>(uLen_)) : 0;
}

////////////////////////////////////////////////////////////////////////////////

ZipStream::ZipStream(unzFile hFile_, const char* pcszPath_, bool fReadOnly_/*=false*/)
    : Stream(pcszPath_, fReadOnly_), m_hFile(hFile_)
{
    unz_file_info sInfo;
    char szFile[MAX_PATH + 6];

    // Get details of the current file
    if (unzGetCurrentFileInfo(hFile_, &sInfo, szFile, MAX_PATH, nullptr, 0, nullptr, 0) == UNZ_OK)
    {
        m_uSize = sInfo.uncompressed_size;
        strcat(szFile, " (zip)");
        m_pszFile = strdup(szFile);
    }
}

void ZipStream::Close()
{
    if (m_hFile)
    {
        unzCloseCurrentFile(m_hFile);
        unzClose(m_hFile);
        m_hFile = nullptr;
        m_nMode = modeClosed;
    }
}

bool ZipStream::Rewind()
{
    // There's no zip rewind so we close and re-open the current file
    if (IsOpen())
        unzCloseCurrentFile(m_hFile);

    return unzOpenCurrentFile(m_hFile) == UNZ_OK;
}

size_t ZipStream::Read(void* pvBuffer_, size_t uLen_)
{
    return unzReadCurrentFile(m_hFile, pvBuffer_, static_cast<unsigned>(uLen_));
}

size_t ZipStream::Write(void* /*pvBuffer_*/, size_t /*uLen_*/)
{
    // Currently there's no support for zip writing (yet)
    return 0;
}

#endif  // HAVE_LIBZ
