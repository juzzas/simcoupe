// Part of SimCoupe - A SAM Coup� emulator
//
// OSD.cpp: Win32 common OS-dependant functions
//
//  Copyright (c) 1999-2001  Simon Owen
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
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#include "SimCoupe.h"

#include <mmsystem.h>

#include "OSD.h"
#include "CPU.h"
#include "Main.h"
#include "Options.h"
#include "UI.h"


namespace OSD
{
bool Init (bool fFirstInit_/*=false*/)
{
    return UI::Init(fFirstInit_);
}

void Exit (bool fReInit_/*=false*/)
{
}


// Return a DWORD containing a millisecond accurate time stamp
// Note: calling could should allow for the value wrapping by only comparing differences
DWORD GetTime ()
{
    static __int64 llFreq = 0, llNow;

    // Need to read the frequency?
    if (!llFreq)
    {
        // If the best high-resolution timer is not available, fall back to the less accurate multimedia version
        if (!QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&llFreq)))
            return timeGetTime();

        // Convert frequency to millisecond units
        llFreq /= 1000i64;
    }

    // Read the current 64-bit time value
    QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&llNow));

    // Convert to ms and return it
    return static_cast<DWORD>(llNow / llFreq);
}


// Do whatever is necessary to locate an additional SimCoupe file - The Win32 version looks in the
// same directory as the EXE, but other platforms could use an environment variable, etc.
// If the path is already fully qualified (an OS-specific decision), return the same string
const char* GetFilePath (const char* pcszFile_)
{
    static char szPath[MAX_PATH];

    // If the supplied file path looks absolute, use it as-is
    if (*pcszFile_ == '\\' || strchr(pcszFile_, ':'))
        lstrcpyn(szPath, pcszFile_, sizeof szPath);

    // Form the full path relative to the current EXE file
    else
    {
        // Get the full path of the running module
        GetModuleFileName(__hinstance, szPath, sizeof szPath);

        // Strip the module file and append the supplied file/path
        strrchr(szPath, '\\')[1] = '\0';
        strcat(szPath, pcszFile_);
    }

    // Return a pointer to the new path
    return szPath;
}

void DebugTrace (const char* pcsz_)
{
    OutputDebugString(pcsz_);
}

bool FrameSync (bool fWait_/*=true*/)
{
    bool fWait = WaitForSingleObject(g_hEvent, 0) == WAIT_TIMEOUT;

    if (fWait_)
        WaitForSingleObject(g_hEvent, INFINITE);

    return fWait;
}

};  // namespace OSD




// Win32 lacks a few of the required POSIX functions, so we'll implement them ourselves...

WIN32_FIND_DATA s_fd;
struct dirent s_dir;

DIR* opendir (const char* pcszDir_)
{
    static char szPath[MAX_PATH];

    memset(&s_dir, 0, sizeof s_dir);

    // Append a wildcard to match all files
    lstrcpy(szPath, pcszDir_);
    if (szPath[lstrlen(szPath)-1] != '\\')
        lstrcat(szPath, "\\");
    lstrcat(szPath, "*");

    // Find the first file, saving the details for later
    HANDLE h = FindFirstFile(szPath, &s_fd);

    // Return the handle if successful, otherwise NULL
    return (h == INVALID_HANDLE_VALUE) ? NULL : reinterpret_cast<DIR*>(h);
}

struct dirent* readdir (DIR* hDir_)
{
    // All done?
    if (!s_fd.cFileName[0])
        return NULL;

    // Copy the filename and set the length
    s_dir.d_reclen = lstrlen(lstrcpyn(s_dir.d_name, s_fd.cFileName, sizeof s_dir.d_name));

    // If we'd already reached the end
    if (!FindNextFile(reinterpret_cast<HANDLE>(hDir_), &s_fd))
        s_fd.cFileName[0] = '\0';

    // Return the current entry
    return &s_dir;
}

int closedir (DIR* hDir_)
{
    return FindClose(reinterpret_cast<HANDLE>(hDir_)) ? 0 : -1;
}
