/*******************************************************************************
 * Copyright (C) Gallium Studio LLC. All rights reserved.
 *
 * This program is open source software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Alternatively, this program may be distributed and modified under the
 * terms of Gallium Studio LLC commercial licenses, which expressly supersede
 * the GNU General Public License and are specifically designed for licensees
 * interested in retaining the proprietary status of their code.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Contact information:
 * Website - https://www.galliumstudio.com
 * Source repository - https://github.com/galliumstudio
 * Email - admin@galliumstudio.com
 ******************************************************************************/

#ifndef CONSOLE_H
#define CONSOLE_H

#include "qpcpp.h"
#include "fw_active.h"
#include "fw_timer.h"
#include "fw_evt.h"
#include "fw_pipe.h"
#include "app_hsmn.h"
#include "CmdInput.h"
#include "CmdParser.h"
#include "ConsoleInterface.h"

using namespace QP;
using namespace FW;

namespace APP {

class Console : public Active {
public:
    static uint16_t GetInst(Hsmn hsmn);
    static Hsmn GetCmdInputHsmn(Hsmn hsmn) { return CMD_INPUT + GetInst(hsmn); }
    static Hsmn GetCmdParserHsmn(Hsmn hsmn) { return CMD_PARSER + GetInst(hsmn); }

    Console(Hsmn hsmn, char const *name, char const *cmdInputName, char const *cmdParserName);

    uint32_t PutChar(char c);
    uint32_t PutCharN(char c, uint32_t count);
    uint32_t PutStr(char const *str);
    void PutStrOver(char const *str, uint32_t oldLen);
    uint32_t Print(char const *format, ...);
    uint32_t PrintItem(uint32_t index, uint32_t minWidth, uint32_t itemPerLine, char const *format, ...);
    uint32_t PrintEvent(QP::QEvt const *e);
    uint32_t PrintBufLine(uint8_t const *lineBuf, uint32_t lineLen, uint8_t unit, uint32_t lineLabel);
    uint32_t PrintBuf(uint8_t const *dataBuf, uint32_t dataLen, uint8_t align = 1, uint32_t label = 0);
    CmdStatus HandleCmd(Evt const *e, CmdHandler const *cmd, uint32_t cmdCount, bool isRoot = false);
    CmdStatus ListCmd(Evt const *e, CmdHandler const *cmd, uint32_t cmdCount);
    uint32_t &Var(uint32_t index);

protected:
    static QState InitialPseudoState(Console * const me, QEvt const * const e);
    static QState Root(Console * const me, QEvt const * const e);
        static QState Stopped(Console * const me, QEvt const * const e);
        static QState Starting(Console * const me, QEvt const * const e);
        static QState Stopping(Console * const me, QEvt const * const e);
        static QState Started(Console * const me, QEvt const * const e);
            static QState Login(Console * const me, QEvt const * const e);
            static QState Interactive(Console * const me, QEvt const * const e);
            static QState Raw(Console * const me, QEvt const * const e);
                static QState RawNormal(Console * const me, QEvt const * const e);
                static QState RawFull(Console * const me, QEvt const * const e);

    bool IsUart() { return (m_ifHsmn >= UART_ACT) && (m_ifHsmn  <= UART_ACT_LAST); }
    // Add other interface type check here.
    void Banner();
    void Prompt();
    CmdStatus RunCmd(char const **argv, uint32_t argc, CmdHandler const *cmd, uint32_t cmdCount);
    void RootCmdFunc(Evt const *e);
    void LastCmdFunc(Evt const *e);
    void ClearVar() { memset(m_var, 0, sizeof(m_var)); }

    Hsmn m_ifHsmn;          // HSMN of the interface active object.
    Hsmn m_outIfHsmn;       // HSMN of the output interface region.
    bool m_isDefault;       // True if this is a default interface for log output when interface is not specified.

    CmdInput m_cmdInput;    // Region to get command line from interface input object.
    CmdParser m_cmdParser;  // Region to parse command line.

    enum {
        OUT_FIFO_ORDER = 12,
        IN_FIFO_ORDER = 10,
        MAX_ARGC = 8,
        MAX_VAR = 8,
    };
    uint8_t m_outFifoStor[1 << OUT_FIFO_ORDER];
    uint8_t m_inFifoStor[1 << IN_FIFO_ORDER];
    Fifo m_outFifo;
    Fifo m_inFifo;
    char m_cmdStr[CmdInput::MAX_LEN];
    char const *m_argv[MAX_ARGC];
    uint32_t m_argc;
    CmdFunc m_rootCmdFunc;
    CmdFunc m_lastCmdFunc;
    uint32_t m_var[MAX_VAR];

    Timer m_stateTimer;

    enum {
        STATE_TIMER = TIMER_EVT_START(CONSOLE),
    };

    enum {
        DONE = INTERNAL_EVT_START(CONSOLE),
        FAILED,
        CMD_RECV,
    };

    class Failed : public ErrorEvt {
    public:
        Failed(Hsmn hsmn, Error error, Hsmn origin, Reason reason) :
            ErrorEvt(FAILED, hsmn, hsmn, 0, error, origin, reason) {}
    };
};

} // namespace APP

#endif // CONSOLE_H