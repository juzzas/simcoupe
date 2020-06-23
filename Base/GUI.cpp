// Part of SimCoupe - A SAM Coupe emulator
//
// GUI.cpp: GUI and controls for on-screen interface
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

//  ToDo:
//   - FileView derived class needed to supply file icons
//   - button repeat on scrollbar
//   - add extra message box buttons (yes/no/cancel, etc.)
//   - regular list box?
//   - use icon for button arrows?
//   - edit box cursor positioning

#include "SimCoupe.h"
#include "GUI.h"

#include <ctype.h>

#include "Expr.h"
#include "Font.h"
#include "Frame.h"
#include "Input.h"
#include "Keyboard.h"
#include "Sound.h"
#include "UI.h"
#include "Video.h"

Window* GUI::s_pGUI;
int GUI::s_nX, GUI::s_nY;

static uint32_t dwLastClick = 0;   // Time of last double-click

std::queue<Window*> GUI::s_garbageQueue;
std::stack<Window*> GUI::s_dialogStack;

bool GUI::SendMessage(int nMessage_, int nParam1_/*=0*/, int nParam2_/*=0*/)
{
    // We're not interested in messages when we're inactive
    if (!s_pGUI)
        return false;

    // Keep track of the mouse
    if (nMessage_ == GM_MOUSEMOVE)
    {
        s_nX = nParam1_;
        s_nY = nParam2_;
    }

    // Check for double-clicks
    else if (nMessage_ == GM_BUTTONDOWN)
    {
        static int nLastX, nLastY;
        static bool fDouble = false;

        // Work out how long it's been since the last click, and how much the mouse has moved
        auto dwNow = OSD::GetTime();
        int nMovedSquared = (nLastX - nParam1_) * (nLastX - nParam1_) + (nLastY - nParam2_) * (nLastY - nParam2_);

        // If the click is close enough to the last click (in space and time), convert it to a double-click
        if (!fDouble && nMovedSquared < (DOUBLE_CLICK_THRESHOLD * DOUBLE_CLICK_THRESHOLD) &&
            dwNow != dwLastClick && dwNow - dwLastClick < DOUBLE_CLICK_TIME)
            nMessage_ = GM_BUTTONDBLCLK;

        // Remember the last time and position of the click
        dwLastClick = dwNow;
        nLastX = nParam1_;
        nLastY = nParam2_;

        // Remember whether we've processed a double-click, so a third click isn't another one
        fDouble = (nMessage_ == GM_BUTTONDBLCLK);
    }

    // Pass the message to the active GUI component
    s_pGUI->RouteMessage(nMessage_, nParam1_, nParam2_);

    // Send a move after a button up, to give a hit test after an effective mouse capture
    if (s_pGUI && nMessage_ == GM_BUTTONUP)
        s_pGUI->RouteMessage(GM_MOUSEMOVE, s_nX, s_nY);

    // Clear out the garbage
    while (!s_garbageQueue.empty())
    {
        delete s_garbageQueue.front();
        s_garbageQueue.pop();
    }

    // If the GUI still exists, update the activation state
    if (s_pGUI)
        s_pGUI->RouteMessage(GM_MOUSEMOVE, s_nX, s_nY);
    // Otherwise we're done
    else
        Stop();

    return true;
}


bool GUI::Start(Window* pGUI_)
{
    // Reject the new GUI if it's already running, or if the emulator is paused
    if (s_pGUI || g_fPaused)
    {
        // Delete the supplied object tree and return failure
        delete pGUI_;
        return false;
    }

    // Set the top level window and clear any last click time
    s_pGUI = pGUI_;
    dwLastClick = 0;

    // Position the cursor off-screen, to ensure the first drawn position matches the native OS position
    s_nX = s_nY = -ICON_SIZE;

    // Silence sound playback
    Sound::Silence();

    return true;
}

void GUI::Stop()
{
    // Delete any existing GUI object
    if (s_pGUI)
    {
        delete s_pGUI;
        s_pGUI = nullptr;
    }

    Input::Purge();
}

void GUI::Delete(Window* pWindow_)
{
    s_garbageQueue.push(pWindow_);

    // If the top-most window is being removed, invalidate the main GUI pointer
    if (pWindow_ == s_pGUI)
        s_pGUI = nullptr;
}

void GUI::Draw(FrameBuffer& fb)
{
    if (s_pGUI)
    {
        fb.SetFont(s_pGUI->GetFont());
        s_pGUI->Draw(fb);

        // Use hardware cursor on Win32, software cursor on everything else (for now).
#ifndef WIN32
        fb.DrawImage(s_nX, s_nY, ICON_SIZE, ICON_SIZE,
            reinterpret_cast<const uint8_t*>(sMouseCursor.abData), sMouseCursor.abPalette);
#endif
    }
}


bool GUI::IsModal()
{
    return !s_dialogStack.empty();
}

////////////////////////////////////////////////////////////////////////////////

Window::Window(Window* pParent_/*=nullptr*/, int nX_/*=0*/, int nY_/*=0*/, int nWidth_/*=0*/, int nHeight_/*=0*/, int nType_/*=ctUnknown*/)
    : m_nX(nX_), m_nY(nY_), m_nWidth(nWidth_), m_nHeight(nHeight_), m_nType(nType_), m_pFont(sGUIFont)
{
    if (pParent_)
    {
        // Set the parent window
        SetParent(pParent_);

        // Adjust our position to be relative to the parent
        m_nX += pParent_->m_nX;
        m_nY += pParent_->m_nY;
    }
}

Window::~Window()
{
    // Delete any child controls
    while (m_pChildren)
    {
        Window* pChild = m_pChildren;
        m_pChildren = m_pChildren->m_pNext;
        delete pChild;
    }

    delete[] m_pszText;
}


// Test whether the given point is inside the current control
bool Window::HitTest(int nX_, int nY_)
{
    return (nX_ >= m_nX) && (nX_ < (m_nX + m_nWidth)) && (nY_ >= m_nY) && (nY_ < (m_nY + m_nHeight));
}

// Draw the child controls on the current window
void Window::Draw(FrameBuffer& fb)
{
    for (Window* p = m_pChildren; p; p = p->m_pNext)
    {
        if (p != m_pActive)
        {
            fb.SetFont(p->m_pFont);
            p->Draw(fb);
        }
    }

    // Draw the active control last to ensure it's shown above any other controls
    if (m_pActive)
    {
        fb.SetFont(m_pActive->m_pFont);
        m_pActive->Draw(fb);
    }
}

// Notify the parent something has changed
void Window::NotifyParent(int nParam_/*=0*/)
{
    if (m_pParent)
        m_pParent->OnNotify(this, nParam_);
}

bool Window::RouteMessage(int nMessage_, int nParam1_/*=0*/, int nParam2_/*=0*/)
{
    bool fProcessed = false;
    bool fMouseMessage = (nMessage_ & GM_TYPE_MASK) == GM_MOUSE_MESSAGE;

    // The active child gets first go at the message
    if (m_pActive)
    {
        // If it's a mouse message, update the hit status for the active child
        if (fMouseMessage)
            m_pActive->m_fHover = m_pActive->HitTest(nParam1_, nParam2_);

        fProcessed = m_pActive->RouteMessage(nMessage_, nParam1_, nParam2_);
    }

    // Give the remaining child controls a chance to process the message
    for (Window* p = m_pChildren; !fProcessed && p; )
    {
        Window* pChild = p;
        p = p->m_pNext;

        // If it's a mouse message, update the child control hit status
        if (fMouseMessage)
            pChild->m_fHover = pChild->HitTest(nParam1_, nParam2_);

        // Skip the active window and disabled windows
        if (pChild != m_pActive && pChild->IsEnabled())
        {
            // If we're clicking on a control, activate it
            if ((nMessage_ == GM_BUTTONDOWN) && pChild->IsTabStop() && pChild->m_fHover)
                pChild->Activate();

            fProcessed = pChild->RouteMessage(nMessage_, nParam1_, nParam2_);
        }
    }

    // After the children have had a look, allow the current window a chance to process
    if (!fProcessed)
        fProcessed = OnMessage(nMessage_, nParam1_, nParam2_);

    // If it's a mouse message, update the hit status for this window
    if (fMouseMessage)
        m_fHover = HitTest(nParam1_, nParam2_);

    // Return whether the message was processed
    return fProcessed;
}

bool Window::OnMessage(int /*nMessage_*/, int /*nParam1_=0*/, int /*nParam2_=0*/)
{
    return false;
}


void Window::SetParent(Window* pParent_/*=nullptr*/)
{
    // Unlink from any existing parent
    if (m_pParent)
    {
        Window* pPrev = GetPrev();

        if (!pPrev)
            m_pParent->m_pChildren = GetNext();
        else
            pPrev->m_pNext = GetNext();

        if (m_pParent->m_pActive == this)
            m_pParent->m_pActive = nullptr;

        m_pParent = m_pNext = nullptr;
    }

    // Set the new parent, if any
    if (pParent_ && pParent_ != this)
    {
        m_pParent = pParent_;

        if (!pParent_->m_pChildren)
            pParent_->m_pChildren = this;
        else
        {
            Window* p;
            for (p = pParent_->m_pChildren; p->GetNext(); p = p->GetNext());
            p->m_pNext = this;
        }
    }
}


void Window::Destroy()
{
    // Destroy any child windows first
    while (m_pChildren)
        m_pChildren->Destroy();

    if (m_pParent)
    {
        // Unlink us from the parent, but remember what it was
        Window* pParent = m_pParent;
        SetParent(nullptr);

        // Re-activate the parent now we're gone
        pParent->Activate();
    }

    // Schedule the object to be deleted when safe
    GUI::Delete(this);
}


void Window::Activate()
{
    if (m_pParent)
        m_pParent->m_pActive = this;
}


void Window::SetText(const char* pcszText_)
{
    // Delete any old string and make a copy of the new one (take care in case the new string is the old one)
    char* pcszOld = m_pszText;
    strcpy(m_pszText = new char[strlen(pcszText_) + 1], pcszText_);
    delete[] pcszOld;
}

unsigned int Window::GetValue() const
{
    char* pEnd = nullptr;
    unsigned long ulValue = strtoul(m_pszText, &pEnd, 0);
    return *pEnd ? 0 : static_cast<unsigned int>(ulValue);
}

void Window::SetValue(unsigned int u_)
{
    char sz[16];
    snprintf(sz, sizeof(sz) - 1, "%u", u_);
    SetText(sz);
}

int Window::GetTextWidth(size_t offset, size_t max_length) const
{
    return m_pFont->StringWidth(GetText() + offset, static_cast<int>(max_length));
}

int Window::GetTextWidth(const char* pcsz_) const
{
    return m_pFont->StringWidth(pcsz_);
}


Window* Window::GetNext(bool fWrap_/*=false*/)
{
    return m_pNext ? m_pNext : (fWrap_ ? GetSiblings() : nullptr);
}

Window* Window::GetPrev(bool fWrap_/*=false*/)
{
    Window* pLast = nullptr;

    for (Window* p = GetSiblings(); p; pLast = p, p = p->m_pNext)
        if (p->m_pNext == this)
            return p;

    return fWrap_ ? pLast : nullptr;
}

// Return the start of the control group containing the current control
Window* Window::GetGroup()
{
    // Search our sibling controls
    for (Window* p = GetSiblings(); p; p = p->GetNext())
    {
        // Continue looking if it's not a radio button
        if (p->GetType() != GetType())
            continue;

        // Search the rest of the radio group
        for (Window* p2 = p; p2 && p2->GetType() == GetType(); p2 = p2->GetNext())
        {
            // If we've found ourselves, return the start of the group
            if (p2 == this)
                return p;
        }
    }

    return nullptr;
}


void Window::MoveRecurse(Window* pWindow_, int ndX_, int ndY_)
{
    // Move our window by the specified offset
    pWindow_->m_nX += ndX_;
    pWindow_->m_nY += ndY_;

    // Move and child windows
    for (Window* p = pWindow_->m_pChildren; p; p = p->m_pNext)
        MoveRecurse(p, ndX_, ndY_);
}

void Window::Move(int nX_, int nY_)
{
    // Perform a recursive relative move of the window and all children
    MoveRecurse(this, nX_ - m_nX, nY_ - m_nY);
}

void Window::Offset(int ndX_, int ndY_)
{
    // Perform a recursive relative move of the window and all children
    MoveRecurse(this, ndX_, ndY_);
}


void Window::SetSize(int nWidth_, int nHeight_)
{
    if (nWidth_) m_nWidth = nWidth_;
    if (nHeight_) m_nHeight = nHeight_;
}

void Window::Inflate(int ndW_, int ndH_)
{
    m_nWidth += ndW_;
    m_nHeight += ndH_;
}

////////////////////////////////////////////////////////////////////////////////

TextControl::TextControl(Window* pParent_/*=nullptr*/, int nX_/*=0*/, int nY_/*=0*/, const char* pcszText_/*=""*/,
    uint8_t bColour_/*=WHITE*/, uint8_t bBackColour_/*=0*/)
    : Window(pParent_, nX_, nY_, 0, 0, ctText), m_bColour(bColour_), m_bBackColour(bBackColour_)
{
    SetTextAndColour(pcszText_, bColour_);
    m_nWidth = GetTextWidth();
}

void TextControl::Draw(FrameBuffer& fb)
{
    if (m_bBackColour)
        fb.FillRect(m_nX - 1, m_nY - 1, GetTextWidth() + 2, 14, m_bBackColour);

    fb.DrawString(m_nX, m_nY, GetText(), IsEnabled() ? m_bColour : GREY_5);
}

void TextControl::SetTextAndColour(const char* pcszText_, uint8_t bColour_)
{
    m_bColour = bColour_;
    Window::SetText(pcszText_);
}

////////////////////////////////////////////////////////////////////////////////

const int BUTTON_BORDER = 3;
const int BUTTON_HEIGHT = BUTTON_BORDER + Font::CHAR_HEIGHT + BUTTON_BORDER;

Button::Button(Window* pParent_, int nX_, int nY_, int nWidth_, int nHeight_)
    : Window(pParent_, nX_, nY_, nWidth_, nHeight_ ? nHeight_ : BUTTON_HEIGHT, ctButton)
{
}

void Button::Draw(FrameBuffer& fb)
{
    bool fPressed = m_fPressed && IsOver();

    // Fill the main button background
    fb.FillRect(m_nX + 1, m_nY + 1, m_nWidth - 2, m_nHeight - 2, IsActive() ? YELLOW_8 : GREY_7);

    // Draw the edge highlight for the top and left
    fb.DrawLine(m_nX, m_nY, m_nWidth, 0, fPressed ? GREY_5 : WHITE);
    fb.DrawLine(m_nX, m_nY, 0, m_nHeight, fPressed ? GREY_5 : WHITE);

    // Draw the edge highlight for the bottom and right
    fb.DrawLine(m_nX + 1, m_nY + m_nHeight - 1, m_nWidth - 2, 0, fPressed ? WHITE : GREY_5);
    fb.DrawLine(m_nX + m_nWidth - 1, m_nY + 1, 0, m_nHeight - 1, fPressed ? WHITE : GREY_5);
}

bool Button::OnMessage(int nMessage_, int nParam1_, int /*nParam2_*/)
{
    switch (nMessage_)
    {
    case GM_CHAR:
        if (!IsActive())
            break;

        switch (nParam1_)
        {
        case HK_SPACE:
        case HK_RETURN:
            NotifyParent(nParam1_ == HK_RETURN);
            return true;
        }
        break;

    case GM_BUTTONDOWN:
    case GM_BUTTONDBLCLK:
        // If the click was over us, flag the button as pressed
        if (IsOver())
            return m_fPressed = true;
        break;

    case GM_BUTTONUP:
        // If we were depressed and the mouse has been released over us, register a button press
        if (IsOver() && m_fPressed)
            NotifyParent();

        // Ignore button ups that aren't for us
        else if (!m_fPressed)
            return false;

        // Unpress the button and return that we processed the message
        m_fPressed = false;
        return true;

    case GM_MOUSEMOVE:
        return m_fPressed;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

TextButton::TextButton(Window* pParent_, int nX_, int nY_, const char* pcszText_/*=""*/, int nMinWidth_/*=0*/)
    : Button(pParent_, nX_, nY_, 0, BUTTON_HEIGHT), m_nMinWidth(nMinWidth_)
{
    SetText(pcszText_);
}

void TextButton::SetText(const char* pcszText_)
{
    Window::SetText(pcszText_);

    // Set the control width to be just enough to contain the text and a border
    m_nWidth = BUTTON_BORDER + GetTextWidth() + BUTTON_BORDER;

    // If we're below the minimum width, set to the minimum
    if (m_nWidth < m_nMinWidth)
        m_nWidth = m_nMinWidth;
}

void TextButton::Draw(FrameBuffer& fb)
{
    Button::Draw(fb);

    bool fPressed = m_fPressed && IsOver();

    // Centralise the text in the button, and offset down and right if it's pressed
    int nX = m_nX + fPressed + (m_nWidth - GetTextWidth()) / 2;
    int nY = m_nY + fPressed + (m_nHeight - Font::CHAR_HEIGHT) / 2 + 1;
    fb.DrawString(nX, nY, GetText(), IsEnabled() ? (IsActive() ? BLACK : BLACK) : GREY_5);
}

////////////////////////////////////////////////////////////////////////////////

ImageButton::ImageButton(Window* pParent_, int nX_, int nY_, int nWidth_, int nHeight_,
    const GUI_ICON* pIcon_, int nDX_/*=0*/, int nDY_/*=0*/)
    : Button(pParent_, nX_, nY_, nWidth_, nHeight_), m_pIcon(pIcon_), m_nDX(nDX_), m_nDY(nDY_)
{
    m_nType = ctImageButton;
}

void ImageButton::Draw(FrameBuffer& fb)
{
    Button::Draw(fb);

    bool fPressed = m_fPressed && IsOver();
    int nX = m_nX + m_nDX + fPressed, nY = m_nY + m_nDY + fPressed;

    fb.DrawImage(nX, nY, ICON_SIZE, ICON_SIZE, reinterpret_cast<const uint8_t*>(m_pIcon->abData),
        IsEnabled() ? m_pIcon->abPalette : m_pIcon->abPalette);
}

////////////////////////////////////////////////////////////////////////////////

UpButton::UpButton(Window* pParent_, int nX_, int nY_, int nWidth_, int nHeight_)
    : Button(pParent_, nX_, nY_, nWidth_, nHeight_)
{
}

void UpButton::Draw(FrameBuffer& fb)
{
    Button::Draw(fb);

    bool fPressed = m_fPressed && IsOver();

    int nX = m_nX + 2 + fPressed, nY = m_nY + 3 + fPressed;
    uint8_t bColour = GetParent()->IsEnabled() ? BLACK : GREY_5;
    fb.DrawLine(nX + 5, nY, 1, 0, bColour);
    fb.DrawLine(nX + 4, nY + 1, 3, 0, bColour);
    fb.DrawLine(nX + 3, nY + 2, 2, 0, bColour);  fb.DrawLine(nX + 6, nY + 2, 2, 0, bColour);
    fb.DrawLine(nX + 2, nY + 3, 2, 0, bColour);  fb.DrawLine(nX + 7, nY + 3, 2, 0, bColour);
    fb.DrawLine(nX + 1, nY + 4, 2, 0, bColour);  fb.DrawLine(nX + 8, nY + 4, 2, 0, bColour);
}

////////////////////////////////////////////////////////////////////////////////

DownButton::DownButton(Window* pParent_, int nX_, int nY_, int nWidth_, int nHeight_)
    : Button(pParent_, nX_, nY_, nWidth_, nHeight_)
{
}

void DownButton::Draw(FrameBuffer& fb)
{
    Button::Draw(fb);

    bool fPressed = m_fPressed && IsOver();

    int nX = m_nX + 2 + fPressed, nY = m_nY + 5 + fPressed;
    uint8_t bColour = GetParent()->IsEnabled() ? BLACK : GREY_5;
    fb.DrawLine(nX + 5, nY + 5, 1, 0, bColour);
    fb.DrawLine(nX + 4, nY + 4, 3, 0, bColour);
    fb.DrawLine(nX + 3, nY + 3, 2, 0, bColour);  fb.DrawLine(nX + 6, nY + 3, 2, 0, bColour);
    fb.DrawLine(nX + 2, nY + 2, 2, 0, bColour);  fb.DrawLine(nX + 7, nY + 2, 2, 0, bColour);
    fb.DrawLine(nX + 1, nY + 1, 2, 0, bColour);  fb.DrawLine(nX + 8, nY + 1, 2, 0, bColour);
}

////////////////////////////////////////////////////////////////////////////////

const int PRETEXT_GAP = 5;
const int BOX_SIZE = 11;

CheckBox::CheckBox(Window* pParent_/*=nullptr*/, int nX_/*=0*/, int nY_/*=0*/, const char* pcszText_/*=""*/,
    uint8_t bColour_/*=WHITE*/, uint8_t bBackColour_/*=0*/)
    : Window(pParent_, nX_, nY_, 0, BOX_SIZE, ctCheckBox), m_bColour(bColour_), m_bBackColour(bBackColour_)
{
    SetText(pcszText_);
}

void CheckBox::SetText(const char* pcszText_)
{
    Window::SetText(pcszText_);

    // Set the control width to be just enough to contain the text
    m_nWidth = 1 + BOX_SIZE + PRETEXT_GAP + GetTextWidth();
}

void CheckBox::Draw(FrameBuffer& fb)
{
    // Draw the text to the right of the box, grey if the control is disabled
    int nX = m_nX + BOX_SIZE + PRETEXT_GAP;
    int nY = m_nY + (BOX_SIZE - Font::CHAR_HEIGHT) / 2 + 1;

    // Fill the background if required
    if (m_bBackColour)
        fb.FillRect(m_nX - 1, m_nY - 1, BOX_SIZE + PRETEXT_GAP + GetTextWidth() + 2, BOX_SIZE + 2, m_bBackColour);

    // Draw the label text
    fb.DrawString(nX, nY, GetText(), IsEnabled() ? (IsActive() ? YELLOW_8 : m_bColour) : GREY_5);

    // Draw the empty check box
    fb.FrameRect(m_nX, m_nY, BOX_SIZE, BOX_SIZE, !IsEnabled() ? GREY_5 : IsActive() ? YELLOW_8 : GREY_7);

    uint8_t abEnabled[] = { 0, GREY_7 }, abDisabled[] = { 0, GREY_5 };

    static const std::vector<uint8_t> check_mark
    {
        0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,1,0,0,
        0,0,0,0,0,0,0,1,1,0,0,
        0,0,0,0,0,0,1,1,1,0,0,
        0,0,1,0,0,1,1,1,0,0,0,
        0,0,1,1,1,1,1,0,0,0,0,
        0,0,1,1,1,1,0,0,0,0,0,
        0,0,0,1,1,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,
    };

    // Box checked?
    if (m_fChecked)
        fb.DrawImage(m_nX, m_nY, 11, 11, check_mark.data(), IsEnabled() ? abEnabled : abDisabled);
}

bool CheckBox::OnMessage(int nMessage_, int nParam1_, int /*nParam2_*/)
{
    static bool fPressed = false;

    switch (nMessage_)
    {
    case GM_CHAR:
    {
        if (!IsActive())
            break;

        switch (nParam1_)
        {
        case HK_SPACE:
        case HK_RETURN:
            SetChecked(!IsChecked());
            NotifyParent();
            return true;
        }
        break;
    }

    case GM_BUTTONDOWN:
    case GM_BUTTONDBLCLK:
        // Was the click over us?
        if (IsOver())
        {
            SetChecked(!IsChecked());
            NotifyParent();
            return fPressed = true;
        }
        break;

    case GM_BUTTONUP:
        if (fPressed)
        {
            fPressed = false;
            return true;
        }
        break;

    case GM_MOUSEMOVE:
        return fPressed;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

const int EDIT_BORDER = 3;
const int EDIT_HEIGHT = EDIT_BORDER + Font::CHAR_HEIGHT + EDIT_BORDER;

const size_t MAX_EDIT_LENGTH = 250;

EditControl::EditControl(Window* pParent_, int nX_, int nY_, int nWidth_, const char* pcszText_/*=""*/)
    : Window(pParent_, nX_, nY_, nWidth_, EDIT_HEIGHT, ctEdit)
{
    SetText(pcszText_);
}

EditControl::EditControl(Window* pParent_, int nX_, int nY_, int nWidth_, unsigned int u_)
    : Window(pParent_, nX_, nY_, nWidth_, EDIT_HEIGHT, ctEdit)
{
    SetValue(u_);
}

void EditControl::Activate()
{
    Window::Activate();

    m_nCaretStart = 0;
    m_nCaretEnd = strlen(GetText());
}

void EditControl::SetSelectedText(const char* pcszText_, bool fSelected_)
{
    Window::SetText(pcszText_);

    // Select the text or position the caret at the end, as requested
    m_nCaretEnd = strlen(pcszText_);
    m_nCaretStart = fSelected_ ? 0 : m_nCaretEnd;
}

void EditControl::Draw(FrameBuffer& fb)
{
    int nWidth = m_nWidth - 2 * EDIT_BORDER;

    // Fill overall control background, and draw a frame round it
    fb.FillRect(m_nX + 1, m_nY + 1, m_nWidth - 2, m_nHeight - 2, IsEnabled() ? (IsActive() ? YELLOW_8 : WHITE) : GREY_7);
    fb.FrameRect(m_nX, m_nY, m_nWidth, m_nHeight, GREY_7);

    // Draw a light edge highlight for the bottom and right
    fb.DrawLine(m_nX + 1, m_nY + m_nHeight - 1, m_nWidth - 1, 0, GREY_7);
    fb.DrawLine(m_nX + m_nWidth - 1, m_nY + 1, 0, m_nHeight - 1, GREY_7);

    // If the caret is before the current view start we need to shift the view back
    if (m_nCaretEnd < m_nViewOffset)
    {
        for (m_nViewOffset = m_nCaretEnd; m_nViewOffset > 0; m_nViewOffset--)
        {
            // Stop if we've exposed 1/4 of the edit control length
            if (GetTextWidth(m_nViewOffset, m_nCaretEnd - m_nViewOffset) >= (nWidth / 4))
            {
                m_nViewOffset--;
                break;
            }
        }
    }
    // If the caret is beyond the end of the control we need to shift the view forwards
    else if (GetTextWidth(m_nViewOffset, m_nCaretEnd - m_nViewOffset) > nWidth)
    {
        // Loop while the view string is still too long for the control
        for (; GetTextWidth(m_nViewOffset) > nWidth; m_nViewOffset++)
        {
            // Stop if caret is within 3/4 of the edit control length
            if (GetTextWidth(m_nViewOffset, m_nCaretEnd - m_nViewOffset) < (nWidth * 3 / 4))
                break;
        }
    }

    // Top-left of text region within edit control
    int nX = m_nX + EDIT_BORDER;
    int nY = m_nY + EDIT_BORDER;

    // Determine the visible length of the string within the control
    size_t nViewLength = 0;
    for (; GetText()[m_nViewOffset + nViewLength]; nViewLength++)
    {
        // Too long for control?
        if (GetTextWidth(m_nViewOffset, nViewLength) > nWidth)
        {
            // Remove a character to bring within width, and finish
            nViewLength--;
            break;
        }
    }

    // Draw the visible text
    fb.Printf(nX, nY + 1, "\a%c%.*s", IsEnabled() ? 'k' : 'K', nViewLength, GetText() + m_nViewOffset);

    // Is the control focussed with an active selection?
    if (IsActive() && m_nCaretStart != m_nCaretEnd)
    {
        size_t nStart = m_nCaretStart, nEnd = m_nCaretEnd;
        if (nStart > nEnd)
            std::swap(nStart, nEnd);

        // Clip start to visible selection
        if (nStart < m_nViewOffset)
            nStart = m_nViewOffset;

        // Clip end to visible selection
        if (nEnd - nStart > nViewLength)
            nEnd = nStart + nViewLength;

        // Determine offset and pixel width of highlighted selection
        int dx = GetTextWidth(m_nViewOffset, nStart - m_nViewOffset);
        int wx = GetTextWidth(nStart, nEnd - nStart);

        // Draw the black selection highlight and white text over it
        fb.FillRect(nX + dx + !!dx - 1, nY - 1, 1 + wx + 1, 1 + Font::CHAR_HEIGHT + 1, IsEnabled() ? (IsActive() ? BLACK : GREY_4) : GREY_6);
        fb.Printf(nX + dx + !!dx, nY + 1, "%.*s", nEnd - nStart, GetText() + nStart);
    }

    // If the control is enabled and focussed we'll show a flashing caret after the text
    if (IsEnabled() && IsActive())
    {
        bool fCaretOn = ((OSD::GetTime() - m_dwCaretTime) % 800) < 400;
        int dx = GetTextWidth(m_nViewOffset, m_nCaretEnd - m_nViewOffset);

        // Draw a character-height vertical bar after the text
        fb.DrawLine(nX + dx - !dx, nY - 1, 0, 1 + Font::CHAR_HEIGHT + 1, fCaretOn ? BLACK : YELLOW_8);
    }
}

bool EditControl::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    switch (nMessage_)
    {
    case GM_BUTTONDOWN:
    case GM_BUTTONDBLCLK:
        // Was the click over us?
        if (IsOver())
            return true;
        break;

    case GM_CHAR:
        // Reject key presses if we're not active and the cursor isn't over us
        if (!IsActive())
            break;

        // Reset caret blink time so it's visible
        m_dwCaretTime = OSD::GetTime();

        bool fCtrl = !!(nParam2_ & HM_CTRL);
        bool fShift = !!(nParam2_ & HM_SHIFT);

        switch (nParam1_)
        {
        case HK_HOME:
            // Start of line, with optional select
            m_nCaretEnd = fShift ? 0 : m_nCaretStart = 0;
            return true;

        case HK_END:
            // End of line, with optional select
            m_nCaretEnd = fShift ? strlen(GetText()) : m_nCaretStart = strlen(GetText());
            return true;

        case HK_LEFT:
            // Previous word
            if (fCtrl)
            {
                // Step back over whitespace
                while (m_nCaretEnd > 0 && GetText()[m_nCaretEnd - 1] == ' ')
                    m_nCaretEnd--;

                // Step back until whitespace
                while (m_nCaretEnd > 0 && GetText()[m_nCaretEnd - 1] != ' ')
                    m_nCaretEnd--;
            }
            // Previous character
            else if (m_nCaretEnd > 0)
                m_nCaretEnd--;

            // Not extending selection?
            if (!fShift)
                m_nCaretStart = m_nCaretEnd;

            return true;

        case HK_RIGHT:
            // Next word
            if (fCtrl)
            {
                // Step back until whitespace
                while (GetText()[m_nCaretEnd] && GetText()[m_nCaretEnd] != ' ')
                    m_nCaretEnd++;

                // Step back over whitespace
                while (GetText()[m_nCaretEnd] && GetText()[m_nCaretEnd] == ' ')
                    m_nCaretEnd++;
            }
            // Next character
            else if (GetText()[m_nCaretEnd])
                m_nCaretEnd++;

            // Not extending selection?
            if (!fShift)
                m_nCaretStart = m_nCaretEnd;

            return true;

            // Return possibly submits the dialog contents
        case HK_RETURN:
        case HK_KPENTER:
            NotifyParent(1);
            return true;

        default:
            // Ctrl combination?
            if (nParam2_ & HM_CTRL)
            {
                // Ctrl-A is select all
                if (nParam1_ == 'A')
                {
                    m_nCaretStart = 0;
                    m_nCaretEnd = strlen(GetText());
                    return true;
                }

                break;
            }

            // No selection?
            if (m_nCaretStart == m_nCaretEnd)
            {
                // For backspace and delete, create a single character selection, if not at the ends
                if (nParam1_ == HK_BACKSPACE && m_nCaretStart > 0)
                    m_nCaretStart--;
                else if (nParam1_ == HK_DELETE && m_nCaretEnd < strlen(GetText()))
                    m_nCaretEnd++;
            }

            // Ignore anything that isn't a printable character, backspace or delete
            if (nParam1_ != HK_BACKSPACE && nParam1_ != HK_DELETE && (nParam1_ < ' ' || nParam1_ > 0x7f))
                break;

            // Active selection?
            if (m_nCaretStart != m_nCaretEnd)
            {
                char sz[MAX_EDIT_LENGTH + 1];

                // Ensure start < end
                if (m_nCaretStart > m_nCaretEnd)
                    std::swap(m_nCaretStart, m_nCaretEnd);

                // Remove the selection, and set the text back
                strcpy(sz, GetText());
                memmove(sz + m_nCaretStart, sz + m_nCaretEnd, strlen(sz + m_nCaretEnd) + 1);
                Window::SetText(sz);

                // Move the end selection to the previous selection start position
                m_nCaretEnd = m_nCaretStart;
            }

            // Only accept printable characters
            if (nParam1_ >= ' ' && nParam1_ <= 0x7f)
            {
                // Only add a character if we're not at the maximum length yet
                if (strlen(GetText()) < MAX_EDIT_LENGTH)
                {
                    char sz[MAX_EDIT_LENGTH + 1];

                    // Insert the new character at the caret position
                    strcpy(sz, GetText());
                    sz[m_nCaretEnd] = nParam1_;
                    memmove(sz + m_nCaretEnd + 1, GetText() + m_nCaretEnd, strlen(GetText() + m_nCaretEnd) + 1);
                    m_nCaretStart = ++m_nCaretEnd;

                    Window::SetText(sz);
                }
            }

            // Content changed
            NotifyParent();
            return true;
        }
        break;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

bool NumberEditControl::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    if (nMessage_ == GM_CHAR)
    {
        switch (nParam1_)
        {
        case HK_KPDECIMAL:  nParam1_ = '.'; break;
        case HK_KPPLUS:     nParam1_ = '+'; break;
        case HK_KPMINUS:    nParam1_ = '-'; break;
        case HK_KPMULT:     nParam1_ = '*'; break;
        case HK_KPDIVIDE:   nParam1_ = '/'; break;
        default:
            if (nParam1_ >= HK_KP0 && nParam1_ <= HK_KP9)
                nParam1_ = (nParam1_ - HK_KP0) + '0';
            break;
        }
    }

    return EditControl::OnMessage(nMessage_, nParam1_, nParam2_);
}

////////////////////////////////////////////////////////////////////////////////

const int RADIO_PRETEXT_GAP = 16;

RadioButton::RadioButton(Window* pParent_/*=nullptr*/, int nX_/*=0*/, int nY_/*=0*/, const char* pcszText_/*=""*/, int nWidth_/*=0*/)
    : Window(pParent_, nX_, nY_, nWidth_, Font::CHAR_HEIGHT + 2, ctRadio)
{
    SetText(pcszText_);
}

void RadioButton::SetText(const char* pcszText_)
{
    Window::SetText(pcszText_);

    // Set the control width to be just enough to contain the text
    m_nWidth = 1 + RADIO_PRETEXT_GAP + GetTextWidth();
}

void RadioButton::Draw(FrameBuffer& fb)
{
    int nX = m_nX + 1, nY = m_nY;

    uint8_t abActive[] = { 0, GREY_5, GREY_7, YELLOW_8 };
    uint8_t abEnabled[] = { 0, GREY_5, GREY_7, GREY_7 };
    uint8_t abDisabled[] = { 0, GREY_3, GREY_5, GREY_5 };

    static const std::vector<uint8_t> selected
    {
        0,0,0,3,3,3,3,0,0,0,
        0,0,3,0,0,0,0,3,0,0,
        0,3,0,1,2,2,1,0,3,0,
        3,0,1,2,2,2,2,1,0,3,
        3,0,2,2,2,2,2,2,0,3,
        3,0,2,2,2,2,2,2,0,3,
        3,0,1,2,2,2,2,1,0,3,
        0,3,0,1,2,2,1,0,3,0,
        0,0,3,0,0,0,0,3,0,0,
        0,0,0,3,3,3,3,0,0,0,
    };

    static const std::vector<uint8_t> unselected
    {
        0,0,0,3,3,3,3,0,0,0,
        0,0,3,0,0,0,0,3,0,0,
        0,3,0,0,0,0,0,0,3,0,
        3,0,0,0,0,0,0,0,0,3,
        3,0,0,0,0,0,0,0,0,3,
        3,0,0,0,0,0,0,0,0,3,
        3,0,0,0,0,0,0,0,0,3,
        0,3,0,0,0,0,0,0,3,0,
        0,0,3,0,0,0,0,3,0,0,
        0,0,0,3,3,3,3,0,0,0,
    };

    // Draw the radio button image in the current state
    fb.DrawImage(nX, nY, 10, 10, (m_fSelected ? selected : unselected).data(),
        !IsEnabled() ? abDisabled : IsActive() ? abActive : abEnabled);

    // Draw the text to the right of the button, grey if the control is disabled
    fb.DrawString(nX + RADIO_PRETEXT_GAP, nY + 1, GetText(), IsEnabled() ? (IsActive() ? YELLOW_8 : GREY_7) : GREY_5);
}

bool RadioButton::OnMessage(int nMessage_, int nParam1_, int /*nParam2_*/)
{
    static bool fPressed = false;

    switch (nMessage_)
    {
    case GM_CHAR:
    {
        if (!IsActive())
            break;

        switch (nParam1_)
        {
        case HK_LEFT:
        case HK_UP:
        {
            Window* pPrev = GetPrev();
            if (pPrev && pPrev->GetType() == GetType())
            {
                pPrev->Activate();
                reinterpret_cast<RadioButton*>(pPrev)->Select();
                NotifyParent();
            }
            return true;
        }

        case HK_RIGHT:
        case HK_DOWN:
        {
            Window* pNext = GetNext();
            if (pNext && pNext->GetType() == GetType())
            {
                pNext->Activate();
                reinterpret_cast<RadioButton*>(pNext)->Select();
                NotifyParent();
            }
            return true;
        }

        case HK_SPACE:
            NotifyParent(1);
            return true;
        }
        break;
    }

    case GM_BUTTONDOWN:
    case GM_BUTTONDBLCLK:
        // Was the click over us?
        if (IsOver())
        {
            Select();
            NotifyParent();
            return fPressed = true;
        }
        break;

    case GM_BUTTONUP:
        if (fPressed)
        {
            fPressed = false;
            return true;
        }
        break;
    }

    return fPressed;
}

void RadioButton::Select(bool fSelected_/*=true*/)
{
    // Remember the new status
    m_fSelected = fSelected_;

    // Of it's a selection we have more work to do...
    if (m_fSelected)
    {
        // Search the control group
        for (Window* p = GetGroup(); p && p->GetType() == ctRadio; p = p->GetNext())
        {
            // Deselect the button if it's not us
            if (p != this)
                reinterpret_cast<RadioButton*>(p)->Select(false);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

const char* const MENU_DELIMITERS = "|";
const int MENU_TEXT_GAP = 5;
const int MENU_ITEM_HEIGHT = 2 + Font::CHAR_HEIGHT + 2;

Menu::Menu(Window* pParent_/*=nullptr*/, int nX_/*=0*/, int nY_/*=0*/, const char* pcszText_/*=""*/)
    : Window(pParent_, nX_, nY_, 0, 0, ctMenu)
{
    SetText(pcszText_);
    Activate();
}

void Menu::Select(int nItem_)
{
    m_nSelected = (nItem_ < 0) ? 0 : (nItem_ >= m_nItems) ? m_nItems - 1 : nItem_;
}

void Menu::SetText(const char* pcszText_)
{
    Window::SetText(pcszText_);

    int nMaxLen = 0;
    char sz[256];
    strncpy(sz, GetText(), sizeof(sz) - 1);
    sz[sizeof(sz) - 1] = '\0';

    char* psz = strtok(sz, MENU_DELIMITERS);

    for (m_nItems = 0; psz; psz = strtok(nullptr, MENU_DELIMITERS), m_nItems++)
    {
        int nLen = GetTextWidth(psz);
        if (nLen > nMaxLen)
            nMaxLen = nLen;
    }

    // Set the control width to be just enough to contain the text
    m_nWidth = MENU_TEXT_GAP + nMaxLen + MENU_TEXT_GAP;
    m_nHeight = MENU_ITEM_HEIGHT * m_nItems;
}

void Menu::Draw(FrameBuffer& fb)
{
    // Fill overall control background and frame it
    fb.FillRect(m_nX, m_nY, m_nWidth, m_nHeight, YELLOW_8);
    fb.FrameRect(m_nX - 1, m_nY - 1, m_nWidth + 2, m_nHeight + 2, GREY_7);

    // Make a copy of the menu item list as strtok() munges as it iterates
    char sz[256];
    strncpy(sz, GetText(), sizeof(sz) - 1);
    sz[sizeof(sz) - 1] = '\0';

    char* psz = strtok(sz, MENU_DELIMITERS);

    // Loop through the items on the menu
    for (int i = 0; psz; psz = strtok(nullptr, MENU_DELIMITERS), i++)
    {
        int nX = m_nX, nY = m_nY + (MENU_ITEM_HEIGHT * i);

        // Draw the string over the default background if not selected
        if (i != m_nSelected)
            fb.DrawString(nX + MENU_TEXT_GAP, nY + 2, psz, BLACK);

        // Otherwise draw the selected item as white on black
        else
        {
            fb.FillRect(nX, nY, m_nWidth, MENU_ITEM_HEIGHT, BLACK);
            fb.DrawString(nX + MENU_TEXT_GAP, nY + 2, psz, WHITE);
        }
    }
}

bool Menu::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    switch (nMessage_)
    {
    case GM_CHAR:
        switch (nParam1_)
        {
            // Return uses the current selection
        case HK_RETURN:
            NotifyParent();
            Destroy();
            return true;

            // Esc cancels
        case HK_ESC:
            m_nSelected = -1;
            NotifyParent();
            Destroy();
            return true;

            // Move to the previous selection, wrapping to the bottom if necessary
        case HK_UP:
            Select((m_nSelected - 1 + m_nItems) % m_nItems);
            break;

            // Move to the next selection, wrapping to the top if necessary
        case HK_DOWN:
            Select((m_nSelected + 1) % m_nItems);
            break;
        }
        return true;

    case GM_BUTTONDBLCLK:
    case GM_BUTTONDOWN:
        // Button clicked (held?) on the menu, so remember it's happened
        if (IsOver())
            return m_fPressed = true;

        // Button clicked away from the menu, which cancels it
        m_nSelected = -1;
        NotifyParent();
        Destroy();
        return true;

    case GM_BUTTONUP:
        // Cursor not over the control?
        if (!IsOver())
        {
            // If it wasn't a drag from the menu, ignore it
            if (!m_fPressed)
                break;

            // Was a drag, so treat this as a cancel
            m_nSelected = -1;
        }

        // Return the current selection
        NotifyParent();
        Destroy();
        return true;

    case GM_MOUSEMOVE:
        // Determine the menu item we're above, if any
        m_nSelected = IsOver() ? ((nParam2_ - m_nY) / MENU_ITEM_HEIGHT) : -1;

        // Treat the movement as an effective drag, for the case of a combo-box drag
        return m_fPressed = true;

    case GM_MOUSEWHEEL:
        if (IsActive())
        {
            Select((m_nSelected + nParam1_ + m_nItems) % m_nItems);
            return true;
        }
        break;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

DropList::DropList(Window* pParent_/*=nullptr*/, int nX_/*=0*/, int nY_/*=0*/, const char* pcszText_/*=""*/, int nMinWidth_/*=0*/)
    : Menu(pParent_, nX_, nY_, pcszText_), m_nMinWidth(nMinWidth_)
{
    m_nSelected = 0;
    SetText(pcszText_);
}

void DropList::SetText(const char* pcszText_)
{
    Menu::SetText(pcszText_);

    if (m_nWidth < m_nMinWidth)
        m_nWidth = m_nMinWidth;
}

bool DropList::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    // Eat movement messages that are not over the control
    if (nMessage_ == GM_MOUSEMOVE && !IsOver())
        return true;

    return Menu::OnMessage(nMessage_, nParam1_, nParam2_);
}

////////////////////////////////////////////////////////////////////////////////

const int COMBO_BORDER = 3;
const int COMBO_HEIGHT = COMBO_BORDER + Font::CHAR_HEIGHT + COMBO_BORDER;

ComboBox::ComboBox(Window* pParent_/*=nullptr*/, int nX_/*=0*/, int nY_/*=0*/, const char* pcszText_/*=""*/, int nWidth_/*=0*/)
    : Window(pParent_, nX_, nY_, nWidth_, COMBO_HEIGHT, ctComboBox),
    m_nItems(0), m_nSelected(0), m_fPressed(false), m_pDropList(nullptr)
{
    SetText(pcszText_);
}


void ComboBox::Select(int nSelected_)
{
    int nOldSelection = m_nSelected;
    m_nSelected = (nSelected_ < 0 || !m_nItems) ? 0 : (nSelected_ >= m_nItems) ? m_nItems - 1 : nSelected_;

    // Nofify the parent if the selection has changed
    if (m_nSelected != nOldSelection)
        NotifyParent();
}

void ComboBox::Select(const char* pcszItem_)
{
    char sz[256];
    strncpy(sz, GetText(), sizeof(sz) - 1);
    sz[sizeof(sz) - 1] = '\0';

    // Find the text for the current selection
    char* psz = strtok(sz, "|");
    for (int i = 0; psz && i < m_nSelected; psz = strtok(nullptr, "|"), i++)
    {
        // If we've found the item, select it
        if (!strcasecmp(psz, pcszItem_))
        {
            Select(i);
            break;
        }
    }
}


const char* ComboBox::GetSelectedText()
{
    static char sz[256];
    strncpy(sz, GetText(), sizeof(sz) - 1);
    sz[sizeof(sz) - 1] = '\0';

    // Find the text for the current selection
    char* psz = strtok(sz, "|");
    for (int i = 0; psz && i < m_nSelected; psz = strtok(nullptr, "|"), i++);

    // Return the item string if found
    return psz ? psz : "";
}

void ComboBox::SetText(const char* pcszText_)
{
    Window::SetText(pcszText_);

    // Count the number of items in the list, then select the first one
    for (m_nItems = !!*pcszText_; (pcszText_ = strchr(pcszText_, '|')); pcszText_++, m_nItems++);
    Select(0);
}

void ComboBox::Draw(FrameBuffer& fb)
{
    bool fPressed = m_fPressed;

    // Fill the main control background
    fb.FrameRect(m_nX, m_nY, m_nWidth, m_nHeight, GREY_7);
    fb.FillRect(m_nX + 1, m_nY + 1, m_nWidth - COMBO_HEIGHT - 1, m_nHeight - 2,
        !IsEnabled() ? GREY_7 : (IsActive() && !fPressed) ? YELLOW_8 : WHITE);

    // Fill the main button background
    int nX = m_nX + m_nWidth - COMBO_HEIGHT, nY = m_nY + 1;
    fb.FillRect(nX + 1, nY + 1, COMBO_HEIGHT - 1, m_nHeight - 3, GREY_7);

    // Draw the edge highlight for the top, left, right and bottom
    fb.DrawLine(nX, nY, COMBO_HEIGHT, 0, fPressed ? GREY_5 : WHITE);
    fb.DrawLine(nX, nY, 0, m_nHeight - 2, fPressed ? GREY_5 : WHITE);
    fb.DrawLine(nX + 1, nY + m_nHeight - 2, COMBO_HEIGHT - 2, 0, fPressed ? WHITE : GREY_5);
    fb.DrawLine(nX + COMBO_HEIGHT - 1, nY + 1, 0, m_nHeight - 2, fPressed ? WHITE : GREY_5);

    // Show the arrow button, down a pixel if it's pressed
    nY += fPressed;
    uint8_t bColour = IsEnabled() ? BLACK : GREY_5;
    fb.DrawLine(nX + 8, nY + 9, 1, 0, bColour);
    fb.DrawLine(nX + 7, nY + 8, 3, 0, bColour);
    fb.DrawLine(nX + 6, nY + 7, 2, 0, bColour);  fb.DrawLine(nX + 9, nY + 7, 2, 0, bColour);
    fb.DrawLine(nX + 5, nY + 6, 2, 0, bColour);  fb.DrawLine(nX + 10, nY + 6, 2, 0, bColour);
    fb.DrawLine(nX + 4, nY + 5, 2, 0, bColour);  fb.DrawLine(nX + 11, nY + 5, 2, 0, bColour);


    char sz[256];
    strncpy(sz, GetText(), sizeof(sz) - 1);
    sz[sizeof(sz) - 1] = '\0';

    // Find the text for the current selection
    char* psz = strtok(sz, "|");
    for (int i = 0; psz && i < m_nSelected; psz = strtok(nullptr, "|"), i++);

    // Draw the current selection
    nX = m_nX + 5;
    nY = m_nY + (m_nHeight - Font::CHAR_HEIGHT) / 2 + 1;
    fb.DrawString(nX, nY, psz ? psz : "", IsEnabled() ? (IsActive() ? BLACK : BLACK) : GREY_5);

    // Call the base to paint any child controls
    Window::Draw(fb);
}

bool ComboBox::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    // Give child controls first go at the message
    if (Window::OnMessage(nMessage_, nParam1_, nParam2_))
        return true;

    switch (nMessage_)
    {
    case GM_CHAR:
        if (!IsActive())
            break;

        switch (nParam1_)
        {
        case HK_SPACE:
        case HK_RETURN:
            m_fPressed = !m_fPressed;
            if (m_fPressed)
                (m_pDropList = new DropList(this, 1, COMBO_HEIGHT, GetText(), m_nWidth - 2))->Select(m_nSelected);
            return true;

        case HK_UP:
            Select(m_nSelected - 1);
            return true;

        case HK_DOWN:
            Select(m_nSelected + 1);
            return true;

        case HK_HOME:
            Select(0);
            return true;

        case HK_END:
            Select(m_nItems - 1);
            return true;
        }
        break;

    case GM_BUTTONDOWN:
    case GM_BUTTONDBLCLK:
        if (!IsOver())
            break;

        // If the click was over us, and the button is pressed, create the drop list
        if (IsOver() && (m_fPressed = !m_fPressed))
            (m_pDropList = new DropList(this, 1, COMBO_HEIGHT, GetText(), m_nWidth - 2))->Select(m_nSelected);

        return true;

    case GM_MOUSEWHEEL:
        if (IsActive())
        {
            Select(m_nSelected + nParam1_);
            return true;
        }
        break;
    }

    return false;
}

void ComboBox::OnNotify(Window* pWindow_, int /*nParam_=0*/)
{
    if (pWindow_ == m_pDropList)
    {
        int nSelected = m_pDropList->GetSelected();
        if (nSelected != -1)
            Select(nSelected);

        m_fPressed = false;
        m_pDropList = nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////

const int SCROLLBAR_WIDTH = 15;
const int SB_BUTTON_HEIGHT = 15;

ScrollBar::ScrollBar(Window* pParent_, int nX_, int nY_, int nHeight_, int nMaxPos_, int nStep_/*=1*/)
    : Window(pParent_, nX_, nY_, SCROLLBAR_WIDTH, nHeight_), m_nStep(nStep_)
{
    m_pUp = new UpButton(this, 0, 0, m_nWidth, SB_BUTTON_HEIGHT);
    m_pDown = new DownButton(this, 0, m_nHeight - SB_BUTTON_HEIGHT, m_nWidth, SB_BUTTON_HEIGHT);

    m_nScrollHeight = nHeight_ - SB_BUTTON_HEIGHT * 2;
    SetMaxPos(nMaxPos_);
}

void ScrollBar::SetPos(int nPosition_)
{
    m_nPos = (nPosition_ < 0) ? 0 : (nPosition_ > m_nMaxPos) ? m_nMaxPos : nPosition_;
}

void ScrollBar::SetMaxPos(int nMaxPos_)
{
    m_nPos = 0;

    // Determine how much of the height is not covered by the current view
    m_nMaxPos = nMaxPos_ - m_nHeight;

    // If we have a scrollable portion, set the thumb size to indicate how much
    if (nMaxPos_ && m_nMaxPos > 0)
        m_nThumbSize = std::max(m_nHeight * m_nScrollHeight / nMaxPos_, 10);
}

void ScrollBar::Draw(FrameBuffer& fb)
{
    // Don't draw anything if we don't have a scroll range
    if (m_nMaxPos <= 0)
        return;

    // Fill the main button background
    fb.FillRect(m_nX + 1, m_nY + 1, m_nWidth - 2, m_nHeight - 2, GREY_7);

    // Draw the edge highlight for the top, left, bottom and right
    fb.DrawLine(m_nX, m_nY, m_nWidth, 0, WHITE);
    fb.DrawLine(m_nX, m_nY, 0, m_nHeight, WHITE);
    fb.DrawLine(m_nX + 1, m_nY + m_nHeight - 1, m_nWidth - 2, 0, WHITE);
    fb.DrawLine(m_nX + m_nWidth - 1, m_nY + 1, 0, m_nHeight - 1, WHITE);

    int nHeight = m_nScrollHeight - m_nThumbSize, nPos = nHeight * m_nPos / m_nMaxPos;

    int nX = m_nX, nY = m_nY + SB_BUTTON_HEIGHT + nPos;

    // Fill the main button background
    fb.FillRect(nX, nY, m_nWidth, m_nThumbSize, !IsEnabled() ? GREY_7 : GREY_7);

    fb.DrawLine(nX, nY, m_nWidth, 0, WHITE);
    fb.DrawLine(nX, nY, 0, m_nThumbSize, WHITE);
    fb.DrawLine(nX + 1, nY + m_nThumbSize - 1, m_nWidth - 1, 0, GREY_4);
    fb.DrawLine(nX + m_nWidth - 1, nY + 1, 0, m_nThumbSize - 1, GREY_4);

    Window::Draw(fb);
}

bool ScrollBar::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    static int nDragOffset;
    static bool fDragging;

    // We're inert (and invisible) if there's no scroll range
    if (m_nMaxPos <= 0)
        return false;

    bool fRet = Window::OnMessage(nMessage_, nParam1_, nParam2_);

    // Stop the buttons remaining active
    m_pActive = nullptr;

    if (fRet)
        return true;

    switch (nMessage_)
    {
    case GM_CHAR:
        if (!IsActive())
            break;

        switch (nParam1_)
        {
        case HK_UP:     SetPos(m_nPos - m_nStep);   break;
        case HK_DOWN:   SetPos(m_nPos + m_nStep);   break;

        default:        return false;
        }

        return true;

    case GM_BUTTONDOWN:
    case GM_BUTTONDBLCLK:
        // Was the click over us when we have an range to scroll
        if (IsOver() && m_nMaxPos > 0)
        {
            int nPos = (m_nScrollHeight - m_nThumbSize) * m_nPos / m_nMaxPos, nY = nParam2_ - (m_nY + SB_BUTTON_HEIGHT);

            // Page up if the click was above the thumb
            if (nY < nPos)
                SetPos(m_nPos - m_nHeight);

            // Page down if below the thumb
            else if (nY >= (nPos + m_nThumbSize))
                SetPos(m_nPos + m_nHeight);

            // Click is on the thumb
            else
            {
                // Remember the vertical offset onto the thumb for drag calculation
                nDragOffset = nParam2_ - (m_nY + SB_BUTTON_HEIGHT) - nPos;
                fDragging = true;
            }

            return true;
        }
        break;

    case GM_BUTTONUP:
        if (fDragging)
        {
            fDragging = false;
            return true;
        }
        break;

    case GM_MOUSEMOVE:
        if (fDragging)
        {
            // Calculate the new position represented by the thumb, and limit it to the valid range
            SetPos((nParam2_ - (m_nY + SB_BUTTON_HEIGHT) - nDragOffset) * m_nMaxPos / (m_nScrollHeight - m_nThumbSize));

            return true;
        }
        break;

    case GM_MOUSEWHEEL:
        SetPos(m_nPos + m_nStep * nParam1_);
        return true;
    }
    return false;
}

void ScrollBar::OnNotify(Window* pWindow_, int /*nParam_=0*/)
{
    if (pWindow_ == m_pUp)
        SetPos(m_nPos - m_nStep);
    else if (pWindow_ == m_pDown)
        SetPos(m_nPos + m_nStep);
}

////////////////////////////////////////////////////////////////////////////////

const int ITEM_SIZE = 72;

// Helper to locate matches when a filename is typed
static bool IsPrefix(const char* pcszPrefix_, const char* pcszName_)
{
    // Skip any common characters at the start of the name
    while (*pcszPrefix_ && *pcszName_ && tolower(*pcszPrefix_) == tolower(*pcszName_))
    {
        pcszPrefix_++;
        pcszName_++;
    }

    // Return true if the full prefix was matched
    return !*pcszPrefix_;
}


ListView::ListView(Window* pParent_, int nX_, int nY_, int nWidth_, int nHeight_, int nItemOffset_/*=0*/)
    : Window(pParent_, nX_, nY_, nWidth_, nHeight_, ctListView), m_nItemOffset(nItemOffset_)
{
    // Create a scrollbar to cover the overall height, scrolling if necessary
    m_pScrollBar = new ScrollBar(this, m_nWidth - SCROLLBAR_WIDTH, 0, m_nHeight, 0, ITEM_SIZE);
}

ListView::~ListView()
{
    // Free any existing items
    SetItems(nullptr);
}


void ListView::Select(int nItem_)
{
    int nOldSelection = m_nSelected;
    m_nSelected = (nItem_ < 0) ? 0 : (nItem_ >= m_nItems) ? m_nItems - 1 : nItem_;

    // Calculate the row containing the new item, and the vertical offset in the list overall
    int nRow = m_nSelected / m_nAcross, nOffset = nRow * ITEM_SIZE - m_pScrollBar->GetPos();

    // If the new item is not completely visible, scroll the list so it _just_ is
    if (nOffset < 0 || nOffset >= (m_nHeight - ITEM_SIZE))
        m_pScrollBar->SetPos(nRow * ITEM_SIZE - ((nOffset < 0) ? 0 : (m_nHeight - ITEM_SIZE)));

    // Inform the owner if the selection has changed
    if (m_nSelected != nOldSelection)
        NotifyParent();
}


// Return the entry for the specified item, or the current item if none was specified
const ListViewItem* ListView::GetItem(int nItem_/*=-1*/) const
{
    int i;

    // If no item is specified, return the default
    if (nItem_ == -1)
        nItem_ = GetSelected();

    // Linear search for the item - not so good for large directories!
    const ListViewItem* pItem = m_pItems;
    for (i = 0; pItem && i < nItem_; i++)
        pItem = pItem->m_pNext;

    // Return the item if found
    return (i == nItem_) ? pItem : nullptr;
}

// Find the item with the specified label (not case-sensitive)
int ListView::FindItem(const char* pcszLabel_, int nStart_/*=0*/)
{
    // Linear search for the item - not so good for large directories!
    int nItem = 0;
    for (const ListViewItem* pItem = GetItem(nStart_); pItem; pItem = pItem->m_pNext, nItem++)
    {
        if (!strcasecmp(pItem->m_pszLabel, pcszLabel_))
            return nItem;
    }

    // Not found
    return -1;
}


void ListView::SetItems(ListViewItem* pItems_)
{
    // Delete any existing list
    for (ListViewItem* pNext; m_pItems; m_pItems = pNext)
    {
        pNext = m_pItems->m_pNext;
        delete m_pItems;
    }

    if (pItems_)
    {
        // Count the number of items in the new list
        for (m_nItems = 0, m_pItems = pItems_; pItems_; pItems_ = pItems_->m_pNext, m_nItems++);

        // Calculate how many items on a row, and how many rows, and set the required scrollbar size
        m_nAcross = m_nWidth / ITEM_SIZE;
        m_nDown = (m_nItems + m_nAcross - 1) / m_nAcross;
        m_pScrollBar->SetMaxPos(m_nDown * ITEM_SIZE);

        Select(0);
    }
}

void ListView::DrawItem(FrameBuffer& fb, int nItem_, int nX_, int nY_, const ListViewItem* pItem_)
{
    // If this is the selected item, draw a box round it (darkened if the control isn't active)
    if (nItem_ == m_nSelected)
    {
        if (IsActive())
            fb.FillRect(nX_ + 1, nY_ + 1, ITEM_SIZE - 2, ITEM_SIZE - 2, BLUE_2);

        fb.FrameRect(nX_, nY_, ITEM_SIZE, ITEM_SIZE, IsActive() ? GREY_7 : GREY_5, true);
    }

    if (pItem_->m_pIcon)
    {
        fb.DrawImage(nX_ + (ITEM_SIZE - ICON_SIZE) / 2, nY_ + m_nItemOffset + 5, ICON_SIZE, ICON_SIZE,
            reinterpret_cast<const uint8_t*>(pItem_->m_pIcon->abData), pItem_->m_pIcon->abPalette);
    }

    const char* pcsz = pItem_->m_pszLabel;
    if (pcsz)
    {
        fb.SetFont(sPropFont);

        int nLine = 0;
        const char* pszStart = pcsz, * pszBreak = nullptr;
        char szLines[2][64], sz[64];
        *szLines[0] = *szLines[1] = '\0';

        // Spread the item text over up to 2 lines
        while (nLine < 2)
        {
            size_t uLen = pcsz - pszStart;
            strncpy(sz, pszStart, uLen)[uLen] = '\0';

            if (GetTextWidth(sz) >= (ITEM_SIZE - 9))
            {
                sz[uLen - 1] = '\0';
                pcsz--;

                if (nLine == 1 || !pszBreak)
                {
                    if (nLine == 1)
                        strcpy(sz + (pcsz - pszStart - 2), "...");
                    strcpy(szLines[nLine++], sz);
                    pszStart = pcsz;
                }
                else
                {
                    if (nLine == 1)
                        strcpy(sz + (pszBreak - pszStart - 2), "...");
                    else
                        sz[pszBreak - pszStart] = '\0';

                    strcpy(szLines[nLine++], sz);
                    pszStart = pszBreak + (*pszBreak == ' ');
                    pszBreak = nullptr;
                }
            }

            // Check for a break point position
            if ((*pcsz == '.' || *pcsz == ' ') && pcsz != pszStart)
                pszBreak = pcsz;

            if (!*pcsz++)
            {
                if (nLine < 2)
                    strcpy(szLines[nLine++], pszStart);
                break;
            }
        }

        // Output the two text lines using the small font, each centralised below the icon
        nY_ += m_nItemOffset + 42;

        fb.DrawString(nX_ + (ITEM_SIZE - fb.StringWidth(szLines[0])) / 2, nY_, szLines[0], WHITE);
        fb.DrawString(nX_ + (ITEM_SIZE - fb.StringWidth(szLines[1])) / 2, nY_ + 12, szLines[1], WHITE);

        fb.SetFont(sGUIFont);
    }
}

// Erase the control background
void ListView::EraseBackground(FrameBuffer& fb)
{
    fb.FillRect(m_nX, m_nY, m_nWidth, m_nHeight, BLUE_1);
}

void ListView::Draw(FrameBuffer& fb)
{
    // Fill the main background of the control
    EraseBackground(fb);

    // Fetch the current scrollbar position
    int nScrollPos = m_pScrollBar->GetPos();

    // Calculate the range of icons that are visible and need drawing
    int nStart = nScrollPos / ITEM_SIZE * m_nAcross, nOffset = nScrollPos % ITEM_SIZE;
    int nDepth = (m_nHeight + nOffset + ITEM_SIZE - 1) / ITEM_SIZE;
    int nEnd = std::min(m_nItems, nStart + m_nAcross * nDepth);

    // Clip to the main control, to keep partly drawn icons within our client area
    fb.ClipTo(m_nX, m_nY, m_nWidth, m_nHeight);

    const ListViewItem* pItem = GetItem(nStart);
    for (int i = nStart; pItem && i < nEnd; pItem = pItem->m_pNext, i++)
    {
        int x = m_nX + ((i % m_nAcross) * ITEM_SIZE), y = m_nY + (((i - nStart) / m_nAcross) * ITEM_SIZE) - nOffset;
        DrawItem(fb, i, x, y, pItem);
    }

    // Restore the default clip area
    fb.ClipNone();
    Window::Draw(fb);
}


bool ListView::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    static char szPrefix[16] = "";

    // Give the scrollbar first look at the message, but prevent it remaining active
    bool fRet = Window::OnMessage(nMessage_, nParam1_, nParam2_);
    m_pActive = nullptr;
    if (fRet)
        return fRet;

    switch (nMessage_)
    {
    case GM_CHAR:
        if (!IsActive())
            break;

        switch (nParam1_)
        {
        case HK_LEFT:   Select(m_nSelected - 1);  break;
        case HK_RIGHT:  Select(m_nSelected + 1);  break;

        case HK_UP:
            // Only move up if we're not already on the top line
            if (m_nSelected >= m_nAcross)
                Select(m_nSelected - m_nAcross);
            break;

        case HK_DOWN:
        {
            // Calculate the row the new item would be on
            int nNewRow = std::min(m_nSelected + m_nAcross, m_nItems - 1) / m_nAcross;

            // Only move down if we're not already on the bottom row
            if (nNewRow != m_nSelected / m_nAcross)
                Select(m_nSelected + m_nAcross);
            break;
        }

        // Move up one screen full, staying on the same column
        case HK_PGUP:
        {
            int nUp = std::min(m_nHeight / ITEM_SIZE, m_nSelected / m_nAcross) * m_nAcross;
            Select(m_nSelected - nUp);
            break;
        }

        // Move down one screen full, staying on the same column
        case HK_PGDN:
        {
            int nDown = std::min(m_nHeight / ITEM_SIZE, (m_nItems - m_nSelected - 1) / m_nAcross) * m_nAcross;
            Select(m_nSelected + nDown);
            break;
        }

        // Move to first item
        case HK_HOME:
            Select(0);
            break;

            // Move to last item
        case HK_END:
            Select(m_nItems - 1);
            break;

            // Return selects the current item - like a double-click
        case HK_RETURN:
            szPrefix[0] = '\0';
            NotifyParent(1);
            break;

        default:
        {
            static uint32_t dwLastChar = 0;
            auto dwNow = OSD::GetTime();

            // Clear the buffer on any non-printing characters or if too long since the last one
            if (nParam1_ < ' ' || nParam1_ > 0x7f || (dwNow - dwLastChar > 1000))
                szPrefix[0] = '\0';

            // Ignore non-printable characters, or if the buffer is full
            if (nParam1_ < ' ' || nParam1_ > 0x7f || strlen(szPrefix) >= 15)
                return false;

            // Ignore duplicates of the same first character, to skip to the next match
            if (!(strlen(szPrefix) == 1 && szPrefix[0] == nParam1_))
            {
                // Add the new character to the buffer
                char* psz = szPrefix + strlen(szPrefix);
                *psz++ = nParam1_;
                *psz = '\0';

                dwLastChar = dwNow;
            }

            // Look for a match, starting *after* the current selection if this is the first character
            int nItem = GetSelected() + (strlen(szPrefix) == 1);
            const ListViewItem* p = GetItem(nItem);

            bool fFound = false;
            for (; p && !(fFound = IsPrefix(szPrefix, p->m_pszLabel)); p = p->m_pNext, nItem++);

            // Nothing found from the item to the end of the list
            if (!fFound)
            {
                // Wrap to search from the start
                p = GetItem(nItem = 0);
                for (; p && !(fFound = IsPrefix(szPrefix, p->m_pszLabel)); p = p->m_pNext, nItem++);

                // No match, so give up
                if (!fFound)
                    break;
            }

            // Select the matching item
            Select(nItem);
            break;
        }
        }

        m_nHoverItem = -1;
        return true;

    case GM_MOUSEMOVE:
    {
        if (!IsOver())
        {
            m_nHoverItem = -1;
            return false;
        }

        int nAcross = (nParam1_ - m_nX) / ITEM_SIZE, nDown = (nParam2_ - m_nY + m_pScrollBar->GetPos()) / ITEM_SIZE;

        // Calculate the item we're above, if any
        int nHoverItem = nAcross + (nDown * m_nAcross);
        m_nHoverItem = (nAcross < m_nAcross && nHoverItem < m_nItems) ? nHoverItem : -1;

        break;
    }

    case GM_BUTTONDOWN:
        if (!IsOver())
            break;

        if (m_nHoverItem != -1)
            Select(m_nHoverItem);
        return true;

    case GM_BUTTONDBLCLK:
        if (!IsOver())
            break;

        NotifyParent(1);
        return true;

    case GM_MOUSEWHEEL:
        m_nHoverItem = -1;
        return false;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

// Compare two filenames, returning true if the 1st entry comes after the 2nd
static bool SortCompare(const char* pcsz1_, const char* pcsz2_)
{
    // Skip any common characters at the start of the name
    while (*pcsz1_ && *pcsz2_ && tolower(*pcsz1_) == tolower(*pcsz2_))
    {
        pcsz1_++;
        pcsz2_++;
    }

    // Compare the first character that differs
    return tolower(*pcsz1_) > tolower(*pcsz2_);
}

// Compare two file entries, returning true if the 1st entry comes after the 2nd
static bool SortCompare(ListViewItem* p1_, ListViewItem* p2_)
{
    // Drives come before directories, which come before files
    if ((p1_->m_pIcon == &sFolderIcon) ^ (p2_->m_pIcon == &sFolderIcon))
        return p2_->m_pIcon == &sFolderIcon;

    // Compare the filenames
    return SortCompare(p1_->m_pszLabel, p2_->m_pszLabel);
}


FileView::FileView(Window* pParent_, int nX_, int nY_, int nWidth_, int nHeight_)
    : ListView(pParent_, nX_, nY_, nWidth_, nHeight_)
{
}

FileView::~FileView()
{
    delete[] m_pszPath;
    delete[] m_pszFilter;
}


bool FileView::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    bool fRet = ListView::OnMessage(nMessage_, nParam1_, nParam2_);

    // Backspace moves up a directory
    if (!fRet && nMessage_ == GM_CHAR && nParam1_ == HK_BACKSPACE)
    {
        // We can only move up if there's a .. entry
        int nItem = FindItem("..");
        if (nItem != -1)
        {
            Select(nItem);
            NotifyParent(1);
            fRet = true;
        }
    }

    return fRet;
}

void FileView::NotifyParent(int nParam_)
{
    const ListViewItem* pItem = GetItem();

    if (pItem && nParam_)
    {
        // Double-clicking to open a directory?
        if (pItem->m_pIcon == &sFolderIcon)
        {
            // Make a copy of the current path to modify
            char szPath[MAX_PATH];
            strncpy(szPath, m_pszPath, MAX_PATH - 1);
            szPath[MAX_PATH - 1] = '\0';

            // Stepping up a level?
            if (!strcmp(pItem->m_pszLabel, ".."))
            {
                // Strip the trailing [back]slash
                szPath[strlen(szPath) - 1] = '\0';

                // Strip the current directory name, if we find another separator
                char* psz = strrchr(szPath, PATH_SEPARATOR);
                if (psz)
                    psz[1] = '\0';

                // Otherwise remove the drive letter to leave a blank path for a virtual drive list (DOS/Win32 only)
                else
                    szPath[0] = '\0';
            }

            // Change to a sub-directory
            else
            {
                // Add the sub-directory name and a trailing backslash
                char szSep[2] = { PATH_SEPARATOR, '\0' };
                strncat(szPath, pItem->m_pszLabel, MAX_PATH - strlen(szPath) - 1);
                szPath[MAX_PATH - 1] = '\0';
                strncat(szPath, szSep, MAX_PATH - strlen(szPath) - 1);
                szPath[MAX_PATH - 1] = '\0';
            }

            // Make sure we have access to the path before setting it
            if (!szPath[0] || OSD::CheckPathAccess(szPath))
                SetPath(szPath);
            else
            {
                char szError[256];
                sprintf(szError, "%s%c\n\nCan't access directory.", pItem->m_pszLabel, PATH_SEPARATOR);
                new MsgBox(this, szError, "Access Denied", mbError);
            }
        }
    }

    // Base handling, to notify parent
    Window::NotifyParent(nParam_);
}

// Determine an appropriate icon for the supplied file name/extension
const GUI_ICON* FileView::GetFileIcon(const char* pcszFile_)
{
    // Determine the main file extension
    char sz[MAX_PATH];
    strncpy(sz, pcszFile_, MAX_PATH - 1);
    sz[MAX_PATH - 1] = '\0';

    char* pszExt = strrchr(sz, '.');

    int nCompressType = 0;
    if (pszExt)
    {
        if (!strcasecmp(pszExt, ".gz")) nCompressType = 1;
        if (!strcasecmp(pszExt, ".zip")) nCompressType = 2;

        // Strip off the main extension and look for another
        if (nCompressType)
        {
            *pszExt = '\0';
            pszExt = strrchr(sz, '.');
        }
    }

    static const char* aExts[] = { ".dsk", ".sad", ".sbt", ".mgt", ".img", ".cpm" };
    bool fDiskImage = false;

    for (unsigned int u = 0; !fDiskImage && pszExt && u < _countof(aExts); u++)
        fDiskImage = !strcasecmp(pszExt, aExts[u]);

    return nCompressType ? &sCompressedIcon : fDiskImage ? &sDiskIcon : &sDocumentIcon;
}


// Get the full path of the current item
const char* FileView::GetFullPath() const
{
    static char szPath[MAX_PATH];
    const ListViewItem* pItem;

    if (!m_pszPath || !(pItem = GetItem()))
        return nullptr;

    strncpy(szPath, m_pszPath, MAX_PATH - 1);
    szPath[MAX_PATH - 1] = '\0';

    strncat(szPath, pItem->m_pszLabel, MAX_PATH - strlen(szPath) - 1);
    szPath[MAX_PATH - 1] = '\0';

    return szPath;
}

// Set a new path to browse
void FileView::SetPath(const char* pcszPath_)
{
    if (pcszPath_)
    {
        delete[] m_pszPath;
        strcpy(m_pszPath = new char[strlen(pcszPath_) + 1], pcszPath_);

        // If the path doesn't end in a separator, we've got a filename too
        const char* pcszFile = strrchr(pcszPath_, PATH_SEPARATOR);
        if (pcszFile && *++pcszFile)
            m_pszPath[pcszFile - pcszPath_] = '\0';

        // Fill the file list
        Refresh();

        // If we can find the file in the list, select it
        int nItem;
        if (pcszFile && (nItem = FindItem(pcszFile)) != -1)
            Select(nItem);
    }
}

// Set a new file filter
void FileView::SetFilter(const char* pcszFilter_)
{
    if (pcszFilter_)
    {
        delete[] m_pszFilter;
        strcpy(m_pszFilter = new char[strlen(pcszFilter_) + 1], pcszFilter_);
        Refresh();
    }
}

// Set whether hidden files should be shown
void FileView::ShowHidden(bool fShow_)
{
    // Update the option and refresh the file list
    m_fShowHidden = fShow_;
    Refresh();
}


// Populate the list view with items from the path matching the current file filter
void FileView::Refresh()
{
    // Return unless we've got both a path and a file filter
    if (!m_pszPath || !m_pszFilter)
        return;

    // Make a copy of the current selection label name
    const ListViewItem* pItem = GetItem();
    char* pszLabel = pItem ? strdup(pItem->m_pszLabel) : nullptr;

    // Free any existing list before we allocate a new one
    SetItems(nullptr);
    ListViewItem* pItems = nullptr;

    // An empty path gives a virtual drive list (only possible on DOS/Win32)
    if (!m_pszPath[0])
    {
        // Work through the letters backwards as we add to the head of the file chain
        for (char chDrive = 'Z'; chDrive >= 'A'; chDrive--)
        {
            char szRoot[] = { chDrive, ':', '\\', '\0' };

            // Can we access the root directory?
            if (OSD::CheckPathAccess(szRoot))
            {
                // Remove the backslash to leave just X:, and add to the list
                szRoot[2] = '\0';
                pItems = new ListViewItem(&sFolderIcon, szRoot, pItems);
            }
        }
    }
    else
    {
        DIR* dir = opendir(m_pszPath);
        if (dir)
        {
            // Count the number of filter items to apply
            int nFilters = *m_pszFilter ? 1 : 0;
            char szFilters[256];
            strncpy(szFilters, m_pszFilter, sizeof(szFilters) - 1);
            szFilters[sizeof(szFilters) - 1] = '\0';

            for (char* psz = strtok(szFilters, ";"); psz && (psz = strtok(nullptr, ";")); nFilters++);

            for (struct dirent* entry; (entry = readdir(dir)); )
            {
                char sz[MAX_PATH];
                struct stat st;

                // Ignore . and .. for now (we'll add .. back later if required)
                if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, ".."))
                    continue;

                // Should we remove hidden files from the listing?
                if (!m_fShowHidden)
                {
                    // Form the full path of the current file
                    char szPath[MAX_PATH];
                    strncpy(szPath, m_pszPath, MAX_PATH - 1);
                    szPath[MAX_PATH - 1] = '\0';
                    strncat(szPath, entry->d_name, MAX_PATH - strlen(szPath) - 1);
                    szPath[MAX_PATH - 1] = '\0';

                    // Skip the file if it's considered hidden
                    if (OSD::IsHidden(szPath))
                        continue;
                }

                // Examine the entry to see what it is
                strncpy(sz, m_pszPath, MAX_PATH - 1);
                sz[MAX_PATH - 1] = '\0';
                strncat(sz, entry->d_name, MAX_PATH - strlen(sz) - 1);

                sz[MAX_PATH - 1] = '\0';

                // Skip if there's no entry to examine
                if (stat(sz, &st) != 0)
                    continue;

                // Only regular files are affected by the file filter
                if (S_ISREG(st.st_mode))
                {
                    // If we have a filter list, apply it now
                    if (nFilters)
                    {
                        int i;

                        // Ignore files with no extension
                        char* pszExt = strrchr(entry->d_name, '.');
                        if (!pszExt)
                            continue;

                        // Compare the extension with each of the filters
                        char* pszFilter = szFilters;
                        for (i = 0; i < nFilters && strcasecmp(pszFilter, pszExt); i++, pszFilter += strlen(pszFilter) + 1);

                        // Ignore the entry if we didn't match it
                        if (i == nFilters)
                            continue;
                    }
                }

                // Ignore anything that isn't a directory or a block device (or a symbolic link to one)
                else if (!S_ISDIR(st.st_mode) && !S_ISBLK(st.st_mode))
                    continue;

                // Create a new list entry for the current item
                ListViewItem* pNew = new ListViewItem(S_ISDIR(st.st_mode) ? &sFolderIcon :
                    S_ISBLK(st.st_mode) ? &sMiscIcon :
                    GetFileIcon(entry->d_name), entry->d_name);

                // Insert the item into the correct sort position
                ListViewItem* p = pItems, * pPrev = nullptr;
                while (p && SortCompare(pNew, p))
                {
                    pPrev = p;
                    p = p->m_pNext;
                }

                // Adjust the links to any neighbouring entries
                if (pPrev) pPrev->m_pNext = pNew;
                pNew->m_pNext = p;
                if (pItems == p) pItems = pNew;
            }

            closedir(dir);
        }

        // If we're not a top-level directory, add a .. entry to the head of the list
        // This prevents non-DOS/Win32 machines stepping back up to the device list level
        if (strlen(m_pszPath) > 1)
            pItems = new ListViewItem(&sFolderIcon, "..", pItems);
    }

    // Give the item list to the list control
    SetItems(pItems);

    // Was there a previous selection?
    if (pszLabel)
    {
        // Look for it in the new list
        int nItem = FindItem(pszLabel);
        free(pszLabel);

        // If found, select it
        if (nItem != -1)
            Select(nItem);
    }
}

////////////////////////////////////////////////////////////////////////////////

IconControl::IconControl(Window* pParent_, int nX_, int nY_, const GUI_ICON* pIcon_)
    : Window(pParent_, nX_, nY_, 0, 0, ctImage), m_pIcon(pIcon_)
{
}

void IconControl::Draw(FrameBuffer& fb)
{
    uint8_t abGreyed[ICON_PALETTE_SIZE];

    // Is the control to be drawn disabled?
    if (!IsEnabled())
    {
        // Make a copy of the palette
        memcpy(abGreyed, m_pIcon->abPalette, sizeof(m_pIcon->abPalette));

        // Grey the icon by using shades of grey with approximate intensity
        for (int i = 0; i < ICON_PALETTE_SIZE; i++)
        {
            // Only change non-zero colours, to keep the background transparent
            if (abGreyed[i])
                abGreyed[i] = GREY_1 + (abGreyed[i] & 0x07);
        }
    }

    // Draw the icon, using the greyed palette if disabled
    fb.DrawImage(m_nX, m_nY, ICON_SIZE, ICON_SIZE, reinterpret_cast<const uint8_t*>(m_pIcon->abData),
        IsEnabled() ? m_pIcon->abPalette : abGreyed);
}

////////////////////////////////////////////////////////////////////////////////

FrameControl::FrameControl(Window* pParent_, int nX_, int nY_, int nWidth_, int nHeight_, uint8_t bColour_/*=WHITE*/, uint8_t bFill_/*=0*/)
    : Window(pParent_, nX_, nY_, nWidth_, nHeight_, ctFrame), m_bColour(bColour_), m_bFill(bFill_)
{
}

void FrameControl::Draw(FrameBuffer& fb)
{
    // If we've got a fill colour, use it now
    if (m_bFill)
        fb.FillRect(m_nX, m_nY, m_nWidth, m_nHeight, m_bFill);

    // Draw the frame around the area
    fb.FrameRect(m_nX, m_nY, m_nWidth, m_nHeight, m_bColour, true);
}

////////////////////////////////////////////////////////////////////////////////

const int TITLE_TEXT_COLOUR = WHITE;
const int TITLE_BACK_COLOUR = BLUE_3;
const int DIALOG_BACK_COLOUR = BLUE_2;
const int DIALOG_FRAME_COLOUR = GREY_7;

const int TITLE_HEIGHT = 4 + Font::CHAR_HEIGHT + 5;


Dialog::Dialog(Window* pParent_, int nWidth_, int nHeight_, const char* pcszCaption_)
    : Window(pParent_, 0, 0, nWidth_, nHeight_, ctDialog), m_nTitleColour(TITLE_BACK_COLOUR), m_nBodyColour(DIALOG_BACK_COLOUR)
{
    SetText(pcszCaption_);

    Centre();
    Window::Activate();

    GUI::s_dialogStack.push(this);
}

Dialog::~Dialog()
{
    GUI::s_dialogStack.pop();
}

bool Dialog::IsActiveDialog() const
{
    return !GUI::s_dialogStack.empty() && GUI::s_dialogStack.top() == this;
}

void Dialog::Centre()
{
    // Position the window slightly above centre
    Move((Frame::Width() - m_nWidth) >> 1, ((Frame::Height() - m_nHeight) * 9 / 10) >> 1);
}

// Activating the dialog will activate the first child control that accepts focus
void Dialog::Activate()
{
    // If there's no active window on the dialog, activate the tab-stop
    if (m_pChildren)
    {
        // Look for the first control with a tab-stop
        Window* p;
        for (p = m_pChildren; p && !p->IsTabStop(); p = p->GetNext());

        // If we found one, activate it
        if (p)
            p->Activate();
    }
}

bool Dialog::HitTest(int nX_, int nY_)
{
    // The caption is outside the original dimensions, so we need a special test
    return (nX_ >= m_nX - 1) && (nX_ < (m_nX + m_nWidth + 1)) && (nY_ >= m_nY - TITLE_HEIGHT) && (nY_ < (m_nY + 1));
}

// Fill the dialog background
void Dialog::EraseBackground(FrameBuffer& fb)
{
    fb.FillRect(m_nX, m_nY, m_nWidth, m_nHeight, IsActiveDialog() ? m_nBodyColour : (m_nBodyColour & ~0x7));
}

void Dialog::Draw(FrameBuffer& fb)
{
    // Make sure there's always an active control
    if (!m_pActive)
        Activate();

#if 0
    // Debug crosshairs to track mapped GUI cursor position
    fb.DrawLine(GUI::s_nX, 0, 0, fb.Height(), WHITE);
    fb.DrawLine(0, GUI::s_nY, fb.Width(), 0, WHITE);
#endif

    // Fill dialog background and draw a 3D frame
    EraseBackground(fb);
    fb.FrameRect(m_nX - 2, m_nY - TITLE_HEIGHT - 2, m_nWidth + 3, m_nHeight + TITLE_HEIGHT + 3, DIALOG_FRAME_COLOUR);
    fb.FrameRect(m_nX - 1, m_nY - TITLE_HEIGHT - 1, m_nWidth + 3, m_nHeight + TITLE_HEIGHT + 3, DIALOG_FRAME_COLOUR - 2);
    fb.Plot(m_nX + m_nWidth + 1, m_nY - TITLE_HEIGHT - 2, DIALOG_FRAME_COLOUR);
    fb.Plot(m_nX - 2, m_nY + m_nHeight + 1, DIALOG_FRAME_COLOUR - 2);

    // Fill caption background and draw the diving line at the bottom of it
    fb.FillRect(m_nX, m_nY - TITLE_HEIGHT, m_nWidth, TITLE_HEIGHT - 1, IsActiveDialog() ? m_nTitleColour : (m_nTitleColour & ~0x7));
    fb.DrawLine(m_nX, m_nY - 1, m_nWidth, 0, DIALOG_FRAME_COLOUR);

    // Draw caption text on the left side
    fb.SetFont(sSpacedGUIFont);
    fb.DrawString(m_nX + 5, m_nY - TITLE_HEIGHT + 5, GetText(), TITLE_TEXT_COLOUR);
    fb.DrawString(m_nX + 5 + 1, m_nY - TITLE_HEIGHT + 5, GetText(), TITLE_TEXT_COLOUR);
    fb.SetFont(sGUIFont);

    // Call the base to draw any child controls
    Window::Draw(fb);
}


bool Dialog::OnMessage(int nMessage_, int nParam1_, int nParam2_)
{
    // Pass to the active control, then the base implementation for the remaining child controls
    if (Window::OnMessage(nMessage_, nParam1_, nParam2_))
        return true;

    switch (nMessage_)
    {
    case GM_CHAR:
    {
        switch (nParam1_)
        {
            // Tab or Shift-Tab moves between any controls that are tab-stops
        case HK_TAB:
        {
            // Loop until we find an enabled control to stop on
            while (m_pActive)
            {
                m_pActive = nParam2_ ? m_pActive->GetPrev(true) : m_pActive->GetNext(true);

                // Stop once we find a suitable control
                if (m_pActive->IsTabStop() && m_pActive->IsEnabled())
                {
                    m_pActive->Activate();
                    break;
                }
            }

            return true;
        }

        // Cursor left or right moves between controls of the same type
        case HK_LEFT:
        case HK_RIGHT:
        case HK_UP:
        case HK_DOWN:
        {
            // Determine the next control
            bool fPrev = (nParam1_ == HK_LEFT) || (nParam1_ == HK_UP);

            // Look for the next enabled/tabstop control of the same type

            for (Window* p = m_pActive; p; )
            {
                p = fPrev ? p->GetPrev(true) : p->GetNext(true);

                // Stop if we're found a different control type
                if (p->GetType() != m_pActive->GetType())
                    break;

                // If we've found a suitable control, activate it
                else if (p->IsEnabled() && p->IsTabStop())
                {
                    p->Activate();
                    break;
                }
            }

            return true;
        }

        // Esc cancels the dialog
        case HK_ESC:
            Destroy();
            break;
        }
        break;
    }

    case GM_BUTTONDOWN:
    case GM_BUTTONDBLCLK:
        // Button down on the caption?
        if (IsOver() && nParam2_ < (m_nY + TITLE_HEIGHT))
        {
            // Remember the offset from the window position to the drag location
            m_nDragX = nParam1_ - m_nX;
            m_nDragY = nParam2_ - m_nY;

            // Flag we're dragging and return the message as processed
            return m_fDragging = true;
        }
        break;

    case GM_BUTTONUP:
        // If this is the button up after finishing a drag, clear the flag
        if (m_fDragging)
        {
            m_fDragging = false;
            return true;
        }
        break;

    case GM_MOUSEMOVE:
        // If we're dragging, move the window to it's new position
        if (m_fDragging)
        {
            Move(nParam1_ - m_nDragX, nParam2_ - m_nDragY);
            return true;
        }
        break;
    }

    // Absorb all other messages to prevent any parent processing
    return true;
}

////////////////////////////////////////////////////////////////////////////////

const int MSGBOX_NORMAL_COLOUR = BLUE_2;
const int MSGBOX_ERROR_COLOUR = RED_2;
const int MSGBOX_BUTTON_SIZE = 50;
const int MSGBOX_LINE_HEIGHT = 15;
const int MSGBOX_GAP = 13;

MsgBox::MsgBox(Window* pParent_, const char* pcszBody_, const char* pcszCaption_, int nFlags_)
    : Dialog(pParent_, 0, 0, pcszCaption_)
{
    // We need to be recognisably different from a regular dialog, despite being based on one
    m_nType = ctMessageBox;

    // Break the body text into lines
    for (char* psz = m_pszBody = strdup(pcszBody_), *pszEOL = psz; pszEOL && *psz; psz += strlen(psz) + 1, m_nLines++)
    {
        if ((pszEOL = strchr(psz, '\n')))
            *pszEOL = '\0';

        // Keep track of the maximum line width
        int nLen = GetTextWidth(psz);
        if (nLen > m_nWidth)
            m_nWidth = nLen;
    }

    // Calculate the text area height
    m_nHeight = (MSGBOX_LINE_HEIGHT * m_nLines);

    // Work out the icon to use, if any
    const GUI_ICON* apIcons[] = { nullptr, &sInformationIcon, &sWarningIcon, &sErrorIcon };
    const GUI_ICON* pIcon = apIcons[(nFlags_ & 0x30) >> 4];

    // Calculate the text area height, and increase to allow space for the icon if necessary
    if (pIcon)
    {
        // Update the body width and height to allow for the icon
        m_nWidth += ICON_SIZE + MSGBOX_GAP / 2;

        // Add the icon to the top-left of the dialog
        m_pIcon = new IconControl(this, MSGBOX_GAP / 2, MSGBOX_GAP / 2, pIcon);
    }

    // Work out the width of the button block, which depends on how many buttons are needed
    int nButtons = 1;
    int nButtonWidth = ((MSGBOX_BUTTON_SIZE + MSGBOX_GAP) * nButtons) - MSGBOX_GAP;

    // Allow for a surrounding border
    m_nWidth += MSGBOX_GAP << 1;
    m_nHeight += MSGBOX_GAP << 1;

    // Centre the button block at the bottom of the dialog [ToDo: add remaining buttons]
    int nButtonOffset = (m_nWidth - nButtonWidth) >> 1;
    (new TextButton(this, nButtonOffset, m_nHeight, "OK", MSGBOX_BUTTON_SIZE))->Activate();

    // Allow for the button height and a small gap underneath it
    m_nHeight += BUTTON_HEIGHT + MSGBOX_GAP / 2;

    // Centralise the dialog on the screen by default
    int nX = (Frame::Width() - m_nWidth) >> 1, nY = (Frame::Height() - m_nHeight) * 2 / 5;
    Move(nX, nY);

    // Error boxes are shown in red
    if (pIcon == &sInformationIcon)
        Dialog::SetColours(MSGBOX_NORMAL_COLOUR + 2, MSGBOX_NORMAL_COLOUR);
    else
        Dialog::SetColours(MSGBOX_ERROR_COLOUR + 1, MSGBOX_ERROR_COLOUR);
}

void MsgBox::Draw(FrameBuffer& fb)
{
    Dialog::Draw(fb);

    // Calculate the x-offset to the body text
    int nX = m_nX + MSGBOX_GAP + (m_pIcon ? ICON_SIZE + MSGBOX_GAP / 2 : 0);

    // Draw each line in the body text
    const char* psz = m_pszBody;
    for (int i = 0; i < m_nLines; psz += strlen(psz) + 1, i++)
        fb.DrawString(nX, m_nY + MSGBOX_GAP + (MSGBOX_LINE_HEIGHT * i), psz, WHITE);
}
