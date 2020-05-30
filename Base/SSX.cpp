// Part of SimCoupe - A SAM Coupe emulator
//
// SSX.cpp: SAM main screen data saving in raw formats
//
// These files hold the display memory data for the main screen area,
// followed by the CLUT indices into the 128 SAM palette colours.
//
//  MODE 1 = 6144 data + 768 attrs + 16 CLUT = 6928 bytes.
//  MODE 2 = 6144 data + 6144 attrs + 16 CLUT = 12304 bytes.
//  MODE 3 = 24576 data + 4 CLUT = 24580 bytes.
//  MODE 4 = 24576 data + 16 CLUT = 24592 bytes.
//
// Mid-display changes to VMPR or CLUT will give the wrong result for
// the dumps above. If detected the file is written in a different format:
//
//  512x192 pixels, each holding palette index (0-127) = 98304 bytes.
//
// The extra horizontal resolution is required for MODE 3. In other modes
// each native pixel is represented by a pair of thin pixels.

#include "SimCoupe.h"
#include "SAMIO.h"
#include "Memory.h"

namespace SSX
{

bool Save(CScreen* screen, int main_x, int main_y)
{
    char szPath[MAX_PATH]{};
    Util::GetUniqueFile("ssx", szPath, sizeof(szPath));

    auto file = fopen(szPath, "wb");
    if (!file)
    {
        Frame::SetStatus("Failed to open %s for writing!", szPath);
        return false;
    }

    if (display_changed)
    {
        for (auto y = 0; y < GFX_SCREEN_LINES; ++y)
            fwrite(screen->GetLine(main_y + y) + main_x, 1, GFX_SCREEN_PIXELS, file);
    }
    else
    {
        auto vmpr_page = VMPR_PAGE;
        auto screen_mode = 1 + (VMPR_MODE >> VMPR_MODE_SHIFT);

        const void* ptr0 = PageReadPtr(vmpr_page);
        const void* ptr1 = nullptr;
        size_t size0 = 0;
        size_t size1 = 0;

        switch (screen_mode)
        {
        case 1:
            ptr0 = PageReadPtr(vmpr_page);
            size0 = 32*192 + 32*24;
            break;
        case 2:
            ptr0 = PageReadPtr(vmpr_page);
            ptr1 = PageReadPtr(vmpr_page) + 0x2000;
            size0 = size1 = 32*192;
            break;
        case 3:
        case 4:
        {
            vmpr_page &= ~1;
            ptr0 = PageReadPtr(vmpr_page);
            ptr1 = PageReadPtr((vmpr_page + 1) & (MEM_PAGE_SIZE - 1));
            size0 = 128*128;
            size1 = 128*64;
            break;
        }
        }

        if (ptr0 && size0)
            fwrite(ptr0, 1, size0, file);
        if (ptr1 && size1)
            fwrite(ptr1, 1, size1, file);

        if (screen_mode == 3)
        {
            for (int i = 0; i < 4; ++i)
                fputc(mode3clut[i], file);
        }
        else
        {
            for (int i = 0; i < N_CLUT_REGS; ++i)
                fputc(clut[i], file);
        }
    }

    fclose(file);
    Frame::SetStatus("Saved %s", szPath);

    return true;
}

}
