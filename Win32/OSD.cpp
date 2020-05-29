// Part of SimCoupe - A SAM Coupe emulator
//
// OSD.cpp: Win32 common OS-dependant functions
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

#include "SimCoupe.h"
#include "OSD.h"

#include <winspool.h>
#include <mmsystem.h>
#include <commctrl.h>
#include <shlobj.h>

#include "CPU.h"
#include "Frame.h"
#include "Main.h"
#include "Options.h"
#include "Parallel.h"
#include "UI.h"
#include "Video.h"


VOID CALLBACK CloseTimerProc(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime);

bool fPortable = false;


bool OSD::Init(bool fFirstInit_/*=false*/)
{
    TRACE("OSD::Init(%d)\n", fFirstInit_);
    char szModule[MAX_PATH];

    // Check if our executable has a read-only attribute set
    if (GetModuleFileName(nullptr, szModule, sizeof(szModule)) &&
        (GetFileAttributes(szModule) & FILE_ATTRIBUTE_READONLY) == FILE_ATTRIBUTE_READONLY)
    {
        // Quit after 42 seconds, to discourage eBay sellers bundling us on CD/DVD, likely with unauthorised SAM software
        SetTimer(nullptr, 0, 42 * 1000, CloseTimerProc);
    }

    if (fFirstInit_)
    {
#ifdef _DEBUG
        _CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_LEAK_CHECK_DF);
#endif
        // Enable portable mode if the options file is local
        fPortable = fs::exists(fs::path(szModule).remove_filename() / OPTIONS_FILE);
        if (fPortable)
            Options::Load(__argc, __argv);

        // Initialise COM and Windows common controls
        CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
        InitCommonControls();

        // We'll do our own error handling, so suppress any windows error dialogs
        SetErrorMode(SEM_FAILCRITICALERRORS);
    }

    return true;
}

void OSD::Exit(bool fReInit_/*=false*/)
{
}


// Return a time-stamp in milliseconds
uint32_t OSD::GetTime()
{
    static LARGE_INTEGER llFreq;
    LARGE_INTEGER llNow;

    // Read high frequency counter, falling back on the multimedia timer
    if (!llFreq.QuadPart && !QueryPerformanceFrequency(&llFreq))
        return timeGetTime();

    // Read the current 64-bit time value, falling back on the multimedia timer
    QueryPerformanceCounter(&llNow);
    return static_cast<uint32_t>((llNow.QuadPart * 1000i64) / llFreq.QuadPart);
}


static bool GetSpecialFolderPath(int csidl_, char* pszPath_, int cbPath_)
{
    char szPath[MAX_PATH] = "";
    LPITEMIDLIST pidl = nullptr;
    bool fRet = false;

    if (cbPath_ > 0 && SUCCEEDED(SHGetSpecialFolderLocation(nullptr, csidl_, &pidl)))
    {
        if (SHGetPathFromIDList(pidl, szPath))
        {
            // Copy to the supplied buffer, leave room for backslash
            lstrcpyn(pszPath_, szPath, cbPath_ - 1);

            // Ensure any non-empty path ends in a backslash
            if (*pszPath_ && pszPath_[lstrlen(pszPath_) - 1] != '\\')
                lstrcat(pszPath_, "\\");

            fRet = true;
        }

        CoTaskMemFree(pidl);
    }

    return fRet;
}

const char* OSD::MakeFilePath(int nDir_, const char* pcszFile_/*=""*/)
{
    static char szPath[MAX_PATH * 2];
    szPath[0] = '\0';

    // In portable mode, force everything to be kept with the EXE, like we used to
    if (fPortable)
        nDir_ = MFP_RESOURCE;

    switch (nDir_)
    {
        // Settings are stored in the user's AppData\Roaming (under SimCoupe\)
    case MFP_SETTINGS:
        GetSpecialFolderPath(CSIDL_APPDATA, szPath, MAX_PATH);
        CreateDirectory(lstrcat(szPath, "SimCoupe\\"), nullptr);
        break;

        // Input file prompts default to the user's Documents directory
    case MFP_INPUT:
        if (GetOption(inpath)[0])
            lstrcpyn(szPath, GetOption(inpath), MAX_PATH);
        else
            GetSpecialFolderPath(CSIDL_MYDOCUMENTS, szPath, MAX_PATH);
        break;

        // Output files go in the user's Documents (under SimCoupe\)
    case MFP_OUTPUT:
        if (GetOption(outpath)[0])
            lstrcpyn(szPath, GetOption(outpath), MAX_PATH);
        else
        {
            GetSpecialFolderPath(CSIDL_MYDOCUMENTS, szPath, MAX_PATH);
            CreateDirectory(lstrcat(szPath, "SimCoupe\\"), nullptr);
        }
        break;

        // Resources are bundled with the EXE, which may be a read-only location
    case MFP_RESOURCE:
#ifdef RESOURCE_DIR
        strncpy(szPath, RESOURCE_DIR, _countof(szPath));
        szPath[_countof(szPath) - 1] = '\0';
        strncat(szPath, "/", _countof(szPath));
        szPath[_countof(szPath) - 1] = '\0';
#else
        szPath[0] = '\0';
#endif
        break;
    }

    // Append any supplied filename (backslash separator already added)
    lstrcat(szPath, pcszFile_);

    // If the resource isn't where we expected, use the EXE directory.
    if (nDir_ == MFP_RESOURCE && !fs::exists(szPath))
    {
        GetModuleFileName(GetModuleHandle(NULL), szPath, MAX_PATH);
        auto path = fs::path(szPath).remove_filename() / pcszFile_;
        strncpy(szPath, path.string().c_str(), MAX_PATH - 1);
    }

    // Return a pointer to the new path
    szPath[_countof(szPath) - 1] = '\0';
    return szPath;
}


// Check whether the specified path is accessible
bool OSD::CheckPathAccess(const char* pcszPath_)
{
    return !access(pcszPath_, X_OK);
}


// Return whether a file/directory is normally hidden from a directory listing
bool OSD::IsHidden(const char* pcszFile_)
{
    // Hide entries with the hidden or system attribute bits set
    uint32_t dwAttrs = GetFileAttributes(pcszFile_);
    return (dwAttrs != INVALID_FILE_ATTRIBUTES) && (dwAttrs & (FILE_ATTRIBUTE_HIDDEN | FILE_ATTRIBUTE_SYSTEM));
}

// Return the path to use for a given drive with direct floppy access
const char* OSD::GetFloppyDevice(int nDrive_)
{
    static char szDevice[] = "_:";

    szDevice[0] = 'A' + nDrive_ - 1;
    return szDevice;
}


void OSD::DebugTrace(const char* pcsz_)
{
    OutputDebugString(pcsz_);
}

VOID CALLBACK CloseTimerProc(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime)
{
    PostMessage(g_hwnd, WM_CLOSE, 0, 0L);
}

////////////////////////////////////////////////////////////////////////////////

CPrinterDevice::CPrinterDevice()
    : m_hPrinter(INVALID_HANDLE_VALUE)
{
}

CPrinterDevice::~CPrinterDevice()
{
    Close();
}


bool CPrinterDevice::Open()
{
    PRINTER_DEFAULTS pd = { "RAW", nullptr, PRINTER_ACCESS_USE };

    if (m_hPrinter != INVALID_HANDLE_VALUE)
        return true;

    if (OpenPrinter(const_cast<char*>(GetOption(printerdev)), &m_hPrinter, &pd))
    {
        DOC_INFO_1 docinfo;
        docinfo.pDocName = "SimCoupe print";
        docinfo.pOutputFile = nullptr;
        docinfo.pDatatype = "RAW";

        // Start the job
        if (StartDocPrinter(m_hPrinter, 1, reinterpret_cast<uint8_t*>(&docinfo)) && StartPagePrinter(m_hPrinter))
            return true;

        ClosePrinter(m_hPrinter);
    }

    Frame::SetStatus("Failed to open %s", GetOption(printerdev));
    return false;
}

void CPrinterDevice::Close()
{
    if (m_hPrinter != INVALID_HANDLE_VALUE)
    {
        EndPagePrinter(m_hPrinter);
        EndDocPrinter(m_hPrinter);

        ClosePrinter(m_hPrinter);
        m_hPrinter = INVALID_HANDLE_VALUE;

        Frame::SetStatus("Printed to %s", GetOption(printerdev));
    }
}

void CPrinterDevice::Write(uint8_t* pb_, size_t uLen_)
{
    if (m_hPrinter != INVALID_HANDLE_VALUE)
    {
        DWORD dwWritten;

        if (!WritePrinter(m_hPrinter, pb_, static_cast<DWORD>(uLen_), &dwWritten))
        {
            Close();
            Frame::SetStatus("Printer error!");
        }
    }
}


// Win32 lacks a few of the required POSIX functions, so we'll implement them ourselves...

WIN32_FIND_DATA s_fd;
struct dirent s_dir;

DIR* opendir(const char* pcszDir_)
{
    static char szPath[MAX_PATH];

    memset(&s_dir, 0, sizeof(s_dir));

    // Append a wildcard to match all files
    lstrcpy(szPath, pcszDir_);
    if (szPath[lstrlen(szPath) - 1] != '\\')
        lstrcat(szPath, "\\");
    lstrcat(szPath, "*");

    // Find the first file, saving the details for later
    HANDLE h = FindFirstFile(szPath, &s_fd);

    // Return the handle if successful, otherwise nullptr
    return (h == INVALID_HANDLE_VALUE) ? nullptr : reinterpret_cast<DIR*>(h);
}

struct dirent* readdir(DIR* hDir_)
{
    // All done?
    if (!s_fd.cFileName[0])
        return nullptr;

    // Copy the filename and set the length
    s_dir.d_reclen = lstrlen(lstrcpyn(s_dir.d_name, s_fd.cFileName, sizeof(s_dir.d_name)));

    // If we'd already reached the end
    if (!FindNextFile(reinterpret_cast<HANDLE>(hDir_), &s_fd))
        s_fd.cFileName[0] = '\0';

    // Return the current entry
    return &s_dir;
}

int closedir(DIR* hDir_)
{
    return FindClose(reinterpret_cast<HANDLE>(hDir_)) ? 0 : -1;
}
