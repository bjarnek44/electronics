;;; Copyright 2020 Bjarne Knudsen
;;;
;;; Redistribution and use in source and binary forms, with or without modification, are permitted
;;; provided that the following conditions are met:
;;;
;;; 1. Redistributions of source code must retain the above copyright notice, this list of
;;; conditions and the following disclaimer.
;;;
;;; 2. Redistributions in binary form must reproduce the above copyright notice, this list of
;;; conditions and the following disclaimer in the documentation and/or other materials provided
;;; with the distribution.
;;;
;;; 3. Neither the name of the copyright holder nor the names of its contributors may be used to
;;; endorse or promote products derived from this software without specific prior written
;;; permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
;;; IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
;;; FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;; CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;;; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;;; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
;;; OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

#include <p16f1705.inc>

        CONFIG  FOSC=INTOSC
        CONFIG  WDTE=OFF, PWRTE=OFF, MCLRE=OFF, CP=OFF, CLKOUTEN=ON, LVP=OFF

        RADIX   dec

;;; Code
;;;
;;; To understand the code, start by reading the documentation for the
;;; multiplexer. The input channels are numbered 0 to 7 in the code
;;; instead of 1 to 8 as in the interface.
;;;
;;; There is an extra queue for characters being sent. This is to
;;; avoid the UART timing bug in the PIC (called "Duplicate
;;; Transmission" in the errata for the PIC). This works because the
;;; transfer to the built-in queue then only happens at a few well
;;; defined times.
;;;
;;; The "main" label defines the main loop of the program.
;;;
;;; The typical data flow is the following:
;;;
;;;   - Four port reads are made for each bit in each input channel.
;;;
;;;   - The bits are assembled to characters.
;;;
;;;   - When a new sentence starts, a storage bank is allocated for
;;;     it.
;;;
;;;   - When a sentence is done, a pointer to the storage bank is put
;;;     in a queue for transmission.
;;;
;;;   - Whenever data is available for transmission, it is sent to the
;;;     custom buffer and then on to the built-in output buffer.
;;;
;;;   - When a sentence is done being transmitted, its bank is marked
;;;     as free again.
;;;
;;; Apart from this, there are several possibilities for errors to
;;; arise. They are:
;;;
;;;   - A stop bit is not high as it should be: the sentence is
;;;     discarded and a period 3.3 ms of high input is awaited.
;;;
;;;   - A sentence has more than 80 characters: the sentence is
;;;     discarded.
;;;
;;;   - A sentence takes too long to receive: the sentence is
;;;     discarded.
;;;
;;;   - There is no free bank for a new sentence: the sentence is
;;;     discarded.
;;;
;;;
;;; Reading bits
;;;
;;; The time it takes for a bit to be received on a channel is split
;;; into four reads from the port. When an input line is idle, the PIC
;;; receives a high signal (the optocouplers invert the NMEA signal
;;; which is itself inverted relative to standard UART).
;;;
;;; When a start bit is received, a new character will be built. Since
;;; the communication is asynchronous, it is unknown when the start
;;; bit transition from high to low will occur. The code looks for a
;;; low value at times 0 and 2 out of the four reads. If the start bit
;;; is observed at time 0, it is known that the bit actually started
;;; some time between time 2 in the last round and time 0 in this
;;; round. Similarly, if the low bit is detected at time 2, it is
;;; known to that the bit started between time 0 and 2. Here are two
;;; examples of the latter:
;;;
;;;   Time        0    1    2    3    0    1    2    3
;;;
;;;   Read value  H         L
;;;                             (L) <- start         X   <- first data bit
;;;               _                     __________________
;;;   Input 1      |___________________X__________________   <- first data bit
;;;               _________                     __________
;;;   Input 2              |___________________X__________   <- first data bit
;;;
;;; Both inputs 1 and 2 read as high at time 0 and low at time 2. In
;;; this case, it is decided to read all the data bits from this
;;; character at time 3. This will be between 25 % and 75 % into
;;; signal for the bit. This means that if the clock difference
;;; between the PIC and the transmitter is less than 2.5 % the
;;; charcter will be successfully read (10 bits including start and
;;; stop).
;;;
;;; Similarly, if the start bit is detected at time 0, the rest of
;;; the character will be read at time 1. The start bit of each
;;; received character will be individually timed, so clock
;;; differences are accommodated.
;;;
;;; If an input channel is constantly low, it reads as a frame error
;;; and never recovers. This means that nothing is received on the
;;; channel and no storage banks are affected. So the channel acts
;;; like and idle channel (which is constantly high).
;;;
;;; All fast channels are on the same port, so they are read at the
;;; same time by reading the whole port. All slow channels are also on
;;; the same channel and are read the same way at different times.
;;;
;;;
;;; Timing
;;;
;;; One main loop is 2 bits for slow channels and 16 bits for fast
;;; channels. Since there are four reads for every bit, the fast
;;; channels are read every 52 cycles and slow channels are read every
;;; 416 cycles (or 417). The reads are macros and take four or six
;;; cycles.
;;;
;;; When a bit is read, it is parsed as described above. So for every
;;; four read operations on a channel, a parse operation is
;;; needed. Parse calls take 24 cycles.
;;;
;;; Parse operations lead to stored bits. Typically one bit per
;;; operation if the channel is active, but rarely two (if the clock
;;; on the transmitter is a bit faster than the clock on the
;;; PIC). Since a byte is ten bits with stop and start, checking for a
;;; new byte every eight parse operations is enough. This is done in
;;; the store calls. These calls put new bytes in the appropriate
;;; storage bank. The slow channels has a store call for every two
;;; parse calls, but that is ok, since nothing is done when no byte is
;;; ready.
;;;
;;; The store calls are split in two (a and b) with a read operation
;;; in the middle. The second part does two complex parts of the
;;; storage: the work setting up a bank for a newly started sentence
;;; and the work when a sentence overflows (is too long). Part a takes
;;; 48 cycles and part b takes 46 cycles.
;;;
;;; Stored bytes are sent over the serial connection. This is done in
;;; left over time in the second store calls (b) if no other work is
;;; needed (which is mostly the case). Every second one of these send
;;; calls move characters from the storage banks to the sending
;;; queue. The rest moves characters from the sending queue to the
;;; built-in UART queue. Since there are 12 storage calls per round of
;;; the main loop this means that at most six characters will be
;;; transmitted per round. For a rate of 115,200 baud, only 4.8
;;; characters needs to be sent per round, so the capacity is
;;; sufficient for that.
;;;
;;; All this leaves four time slots of around 48 cycles. These are
;;; used as follows:
;;;
;;;   chk_time: checks that the main loop always takes 3,333 cycles
;;;             (for debug output).
;;;
;;;   chk_input: checks for input and goes to interactive mode when
;;;              appropriate. Also adjusts suppression timers.
;;;
;;;   hdl_time: Adjust more suppression timers.
;;;
;;;   chk_new_msg: frees banks that are marked as being used but with
;;;                no new sentences being put in the bank.
;;;
;;; The last check handles the situation where a transmitter stops mid
;;; sentence. In this case, the sentence is dropped after about 15
;;; seconds and the bank is freed.
;;;
;;; nop instructions are inserted where needed to ensure 52 cycles
;;; between read operations. Four extra nop instructions along with
;;; the final goto gives five extra cycles for a total of 3,3333 for
;;; the main loop.
;;;
;;; An overview:
;;;
;;;  ___| 52 cycles            | 52 cycles            | 52 cycles          | 52 cycles          |___
;;;    * R_f0  P_f0/g0 P_f1/g1  R_f1  P_f2/g2 P_f3/g3  R_f2/g0  S_f0a       R_f3/s0  S_f0b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     S_f1a       R_f3     S_f1b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2/g1  S_f2a       R_f3/s1  S_f2b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     S_f3a       R_f3     S_f3b
;;;    * R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2/g2  S_s0a       R_f3/s2  S_s0b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     S_s1a       R_f3     S_s1b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2/g3  P_s0  P_s1  R_f3/s3  chk_time
;;;  __* R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     P_s2  P_s3  R_f3     chk_new_msg ___
;;;      R_f0  P_f0/g0 P_f1/g1  R_f1  P_f2/g2 P_f3/g3  R_f2/g0  S_f0a       R_f3/s0  S_f0b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     S_f1a       R_f3     S_f1b
;;;    * R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2/g1  S_f2a       R_f3/s1  S_f2b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     S_f3a       R_f3     S_f3b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2/g2  S_s2a       R_f3/s2  S_s2b
;;;    * R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     S_s3a       R_f3     S_s3b
;;;      R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2/g3  P_s0  P_s1  R_f3/s3  chk_input
;;;  ___ R_f0  P_f0    P_f1     R_f1  P_f2    P_f3     R_f2     P_s2  P_s3  R_f3     hdl_time    ___
;;;     | 52 cycles            | 52 cycles            | 52 cycles          | 52 cycles          |
;;;
;;;  * = extra cycle: nop / loop goto
;;;
;;; R = read, P = parse, S = store
;;; s = slow channel, f = fast channel, g = fast channel in slow mode
;;;
;;;
;;; Memory organization:
;;;
;;; The 11 storage banks are put in physical banks 1-11 while banks 0
;;; and 12 are used for general storage:
;;;
;;;   0x020-0x07F 96 General
;;;   0x0A0-0x0EF 80 Storage Bank 1
;;;   0x120-0x16F 80 Storage Bank 2
;;;   0x1A0-0x1EF 80 Storage Bank 3
;;;   0x220-0x26F 80 Storage Bank 4
;;;   0x2A0-0x2EF 80 Storage Bank 5
;;;   0x320-0x36F 80 Storage Bank 6
;;;   0x3A0-0x3EF 80 Storage Bank 7
;;;   0x420-0x46F 80 Storage Bank 8
;;;   0x4A0-0x4EF 80 Storage Bank 9
;;;   0x520-0x56F 80 Storage Bank 10
;;;   0x5A0-0x5EF 80 Storage Bank 11
;;;   0x620-0x64F 48 General
;;;
;;;
;;; Program memory organization:
;;;
;;; The goto instruction is limited to work in a 0x800 address
;;; section, so the code is split in three main sections with a small
;;; fourth one for settings:
;;;
;;;   near:     0x0000-0x07FF  start, main loop, chk functions, parse functions
;;;   far:      0x0000-0x07FF  store, send
;;;   veryfar:  0x1000-0x17FF  all interactive mode, init functions
;;;   settings: 0x1F60-0x1FFF
;;;
;;; Settings are stored in program memory to be persistent when the
;;; device is powered off.
;;;
;;;
;;; Test long call correctness:
;;;
;;; The assembler does not alert when a wrong long call is being
;;; used. This code cheks for such instances:
;;;
;;;   awk 'BEGIN {t="m"} $2 == "code" {t = substr($1,1,1)} substr($1, length($1)) == ":" {s[substr($1, 1, length($1)-1)]=t} END {for (i in s) {print i, s[i]}}' nmea.asm > tmp.txt ; awk 'BEGIN {while (getline < "tmp.txt") {s[$1]=$2}} substr($0,1,1) == " " && substr($1, 3) == "call" && s[$2] != substr($1,2,1) {print "Error:", $1, $2, s[$2]; c = 1} END {if (c == 0) {print "No errors"}}' nmea.asm

;;;   TODO: allow inverted input for configuration (for stand alone version)
;;;   TODO: auto detect inversion of input for configuration (for stand alone version)
;;;   TODO: count total number of messages

;;; /////////////////////////////////////////////////////////////////////////////

;;; Memory addresses for variables:

BUILD0          equ     0x20            ; For building chars for each channel
CHAR0           equ     0x28            ; Finished chars for each channel
BANK0           equ     0x30            ; Bank for each channel. 0xFF bank not found yet, DISCARD_BANK for discards
SUPPRESS0       equ     0x38            ; Suppress masks. One for each channel
TIMER0H         equ     0x40            ; Counters for busy channels, one for each channel
TIMER0L         equ     0x48            ; And low part for each channel
DISCARD_CHAR0   equ     0x50            ; Start char to discard for each channel, 0 means no discard

READF0          equ     0x58            ; Fast port data at time 0
READF1          equ     0x59            ; Fast port data at time 1
READF2          equ     0x5A            ; Fast port data at time 2
READF3          equ     0x5B            ; Fast port data at time 3
READS0          equ     0x5C            ; Slow port data at time 0
READS1          equ     0x5D            ; Slow port data at time 1
READS2          equ     0x5E            ; Slow port data at time 2
READS3          equ     0x5F            ; Slow port data at time 3
READFS0         equ     0x60            ; Fast port data at slow time 0
READFS1         equ     0x61            ; Fast port data at slow time 1
READFS2         equ     0x62            ; Fast port data at slow time 2
READFS3         equ     0x63            ; Fast port data at slow time 3

CH_RDY          equ     0x64            ; Flags for finished chars ready for each channel
WAITING         equ     0x65            ; Whether we are waiting for data (a bit for each channel)
PHASE           equ     0x66            ; Whether to read on round 1 (set) or 3 (not set) (a bit for each channel)
DONE            equ     0x67            ; Whether we are done with a byte and reading the stop bit (a bit for each channel)
SEND_BK         equ     0x68            ; Bank being sent from. 0x80: not sending, 0x4?: setting up, 0x0?: sending
SEND_END        equ     0x69            ; End address for sending
CH_BUSY         equ     0x6A            ; Flags for busy channels
Q_START         equ     0x6B            ; Index of transmit bank queue start
Q_END           equ     0x6C            ; Index of transmit bank queue end
FLAGS           equ     0x6D            ; Various flags

INVERT_F        equ     0x6E            ; Input on fast port is xor'ed with this
INVERT_S        equ     0x6F            ; Input on slow port is xor'ed with this

BK_FREEH        equ     0x70            ; Flags for banks being free, for high banks 8 - 15 (12 - 15 not used)
BK_FREEL        equ     0x71            ; Flags for banks being free, for low banks 0 - 7 (0 not used)

INTER_TMP       equ     0x72            ; Temporary storage used in interactive mode
INTER_CHANNEL   equ     0x73            ; Channel number used in interactive mode
INTER_VALUE     equ     0x74            ; Command value number used in interactive mode

NEW_MSGH        equ     0x75            ; low part. Used for discarding slow inputs
NEW_MSGL        equ     0x76            ; Banks which receive a new message, high part

TM0H            equ     INTER_CHANNEL   ; Last timer value, high part (note reuse of memory)
TM0L            equ     INTER_TMP       ; Last timer value, low part (note reuse of memory)
TM1H            equ     0x77            ; Minimum cycle count, high part
TM1L            equ     0x78            ; Minimum cycle count, low part
TM2H            equ     0x79            ; Maximum cycle count, high part
TM2L            equ     0x7A            ; Maximum cycle count, low part
TM3H            equ     0x7B            ; Temporary timer value, high part
TM3L            equ     INTER_VALUE     ; Temporary timer value, low part (note reuse of memory)
CNT_CONGEST     equ     0x7C            ; Counter for sentences dropped due to missing space
CNT_FRAME       equ     0x7D            ; Counter for sentences dropped due to frame errors
ERR_CHANNELS    equ     0x7E            ; Bits indicating channels with errors
SEND_CHAR       equ     0x7F            ; Single char buffer for trasnmission

;;; These pointers point to the last 8 bits of the full address in the relevant bank
PTR1            equ     0x620           ; Pointer for each bank.
PTR0            equ     PTR1 - 1        ; Just a reference, there is no bank 0

REF1            equ     0x62B           ; Channel number for bank
REF0            equ     REF1 - 1        ; Just a reference, there is no bank 0

QUEUE           equ     0x636           ; 16 bytes, circular buffer

NEW_MSGHB       equ     0x646           ; Like NEW_MSGH, but last observed value
NEW_MSGLB       equ     0x647           ; Like NEW_MSGL, but last observed value

STUCK_CNT1      equ     0x648           ; Counter for stuck channel checks, high part
STUCK_CNT2      equ     0x649           ; Counter for stuck channel checks, low part
STUCK_BANK      equ     0x64A           ; A bank observed to be stuck (otherwise DISCARD_BANK)

INTER_SPEED     equ     0x64B           ; The transmit baud rate (0-2) set in interactive mode

CNT_LONG        equ     0x64C           ; Counter for sentences dropped due to being too long
CNT_BINARY      equ     0x64D           ; Counter for sentences dropped due to being binary
CNT_SLOW        equ     0x64E           ; Counter for sentences dropped due to taking too long

STUCK_CHANNEL   equ     0x64F           ; For recording stuck channels as errors

;;; /////////////////////////////////////////////////////////////////////////////

;;; Constants:

MAJOR_VERSION   equ     '0'             ; Major version number
MINOR_VERSION   equ     '2'             ; Minor version number

CONFIG_PORT     equ     PORTA           ; Input port for configuration signal
CONFIG_PIN      equ     RA2             ; Input pin for configuration signal

FAST0_NUM       equ     0               ; Overall channel number for fast channel 0
FAST1_NUM       equ     1               ; Overall channel number for fast channel 1
FAST2_NUM       equ     2               ; Overall channel number for fast channel 2
FAST3_NUM       equ     3               ; Overall channel number for fast channel 3

SLOW0_NUM       equ     4               ; Overall channel number for slow channel 0
SLOW1_NUM       equ     5               ; Overall channel number for slow channel 1
SLOW2_NUM       equ     6               ; Overall channel number for slow channel 2
SLOW3_NUM       equ     7               ; Overall channel number for slow channel 3

FAST_PORT       equ     PORTA           ; Input port for fast channels
SLOW_PORT       equ     PORTC           ; Input port for slow channels

FAST0_PIN       equ     RA1             ; Input pin for fast channel 0
FAST1_PIN       equ     RA0             ; Input pin for fast channel 1
FAST2_PIN       equ     RA5             ; Input pin for fast channel 2
FAST3_PIN       equ     RA3             ; Input pin for fast channel 3

SLOW0_PIN       equ     RC5             ; Input pin for slow channel 0
SLOW1_PIN       equ     RC4             ; Input pin for slow channel 1
SLOW2_PIN       equ     RC3             ; Input pin for slow channel 2
SLOW3_PIN       equ     RC2             ; Input pin for slow channel 3

BANK_MASKH      equ     0x0F            ; Banks 8-11 in use and ...
BANK_MASKL      equ     0xFE            ; ... banks 1-7 in use

FRAME_REC_CNT_S equ     0x10            ; Stop bits to read before recovering from frame error on slow ch
FRAME_REC_CNT_F equ     0x80            ; Stop bits to read before recovering from frame error on fast ch

TIMER_HIGH      equ     0xE8            ; Timer for suppression, around 2.5 s

DISCARD_BANK    equ     12              ; Artifical unused bank number

NEWLINE_FLAGS   equ     FLAGS           ; Which flags byte to use for return-newline setting
NEWLINE_BIT     equ     5               ; The bit

TM_VALID_FLAGS  equ     FLAGS           ; Which flags byte to use for flag indicating last timing ...
                                        ; ... valid, used for timing the main loop
TM_VALID_BIT    equ     6               ; The bit

SD_CH_FLAGS     equ     FLAGS           ; Which flags byte to use for whether a char is ready to send
SD_CH_BIT       equ     4               ; The bit

CHN_OUT_FLAGS   equ     FLAGS           ; Which flags byte to use for channel output setting
CHN_OUT_BIT     equ     7               ; The bit

FAST_FLAGS      equ     FLAGS           ; Which flags byte to use for fast channel setting
FAST_BIT0       equ     0               ; The bit for fast channel 0
FAST_BIT1       equ     1               ; The bit for fast channel 1
FAST_BIT2       equ     2               ; The bit for fast channel 2
FAST_BIT3       equ     3               ; The bit for fast channel 3

;;; /////////////////////////////////////////////////////////////////////////////

;;; Settings are read from program memory. They are all given in this
;;; macro that is used for inittial user settings as well as factory
;;; settings.
settings        macro

        retlw   0x00            ; Suppression channel 1
        retlw   0x00            ; Suppression channel 2
        retlw   0x00            ; Suppression channel 3
        retlw   0x00            ; Suppression channel 4
        retlw   0x00            ; Suppression channel 5
        retlw   0x00            ; Suppression channel 6
        retlw   0x00            ; Suppression channel 7
        retlw   0x00            ; Suppression channel 8

        retlw   0x00            ; Discard channel 1
        retlw   0x00            ; Discard channel 2
        retlw   0x00            ; Discard channel 3
        retlw   0x00            ; Discard channel 4
        retlw   0x00            ; Discard channel 5
        retlw   0x00            ; Discard channel 6
        retlw   0x00            ; Discard channel 7
        retlw   0x00            ; Discard channel 8

        retlw   0x01            ; Channel output
        retlw   0x0F            ; Fast channels
        retlw   0x01            ; Return newline
        retlw   0x00            ; Inverted input
        retlw   0x00            ; Inverted output
        retlw   0x02            ; Speed
        retlw   0xFF            ; Schmitt triggers

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; Do nothing for a given number of cycles. If count >= 6, W will be affected.
nopm    macro   count

if (count >= 6)
        movlw   count / 3
        decfsz  WREG, f
        goto    $-1
endif

if (count >= 3 && count < 6)
        nop
        nop
        nop
endif

if (count % 3 == 2)
        nop
        nop
endif

if (count % 3 == 1)
        nop
endif
        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; For calling code in the far segment from the near segment.
nfcall  macro   label

        movlp   0x08
errorlevel      -306
        call    label
errorlevel      +306
        movlp   0x00

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; For calling code in the veryfar segment from the near segment.
nvcall  macro   label

        movlp   0x10
errorlevel      -306
        call    label
errorlevel      +306
        movlp   0x00

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; For calling code in the veryfar segment from the far segment.
fvcall  macro   label

        movlp   0x10
errorlevel      -306
        call    label
errorlevel      +306
        movlp   0x08

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 4 cycles. Read one port.
read1   macro   adr

        movfw   FAST_PORT
        xorwf   INVERT_F, W
        movwf   adr
        nop

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 4 cycles. Read one port as both fast and slow. The fast part is
;;; always put in READF2, the slow part is given by adr.
read1fs macro   adr

        movfw   FAST_PORT
        xorwf   INVERT_F, W
        movwf   READF2
        movwf   adr

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 6 cycles. Read a fast and a slow port. The fast one is always put
;;; in READF3. The slow one is given by adr.
read2   macro   adr

        movfw   FAST_PORT
        xorwf   INVERT_F, W
        movwf   READF3
        movfw   SLOW_PORT
        xorwf   INVERT_S, W
        movwf   adr

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 16 cycles. W is set to the lowest available bank number. Carry set
;;; if bank is found.
find_bk macro   bk1, bk2, channel
        local   check_4_7
        local   check_1_3
        local   no_free
        local   check_8_11
        local   done

        bsf     STATUS, C
        movfw   bk1
        btfsc   STATUS, Z
        goto    check_8_11

        andlw   0x0F
        btfss   STATUS, Z
        goto    check_1_3

check_4_7:                      ; 7 cycles to here
        movlw   7
        btfsc   bk1, 6
        movlw   6
        btfsc   bk1, 5
        movlw   5
        btfsc   bk1, 4
        movlw   4

        goto    done

check_1_3:                      ; 8 cycles to here
        movlw   3
        btfsc   bk1, 2
        movlw   2
        btfsc   bk1, 1
        movlw   1

        nop
        goto    done

no_free:                        ; 9 cycles to here
if (channel >= 0)
        incfsz  CNT_CONGEST, W
        movwf   CNT_CONGEST
        bsf     ERR_CHANNELS, channel
else
        nop
        nop
        nop
endif

        movlw   DISCARD_BANK

        bcf     STATUS, C       ; Indicate that a bank was not found
        goto    done

check_8_11:                     ; 5 cycles to here
        movfw   bk2
        btfsc   STATUS, Z
        goto    no_free

        movlw   11
        btfsc   bk2, 2
        movlw   10
        btfsc   bk2, 1
        movlw   9
        btfsc   bk2, 0
        movlw   8

        nop
done:                           ; 16 cycles to here

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 8 cycles. The timers count up and stop at zero. CH_BUSY is set
;;; while counter is still running.
tm_step macro   channel

        local   not_zero
        local   done

        movf    TIMER0H + channel, f
        btfss   STATUS, Z
        goto    not_zero

        bcf     CH_BUSY, channel

        nop
        nop

        goto    done

not_zero:                       ; 4 cycles to here
        incfsz  TIMER0L + channel, f
        decf    TIMER0H + channel, f
        incf    TIMER0H + channel, f

        bsf     CH_BUSY, channel

done:                           ; 8 cycles to here

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 46 = 43 + 3 cycles including call and return. Stores a character
;;; for a particular channel.
mv_char macro   channel

        local   discard_suppress
        local   store
        local   no_store
        local   discard_binary
        local   finish
        local   binary
        local   discard_binary2
        local   finish_discard
        local   discard
        local   overflow
        local   done

        btfss   CH_RDY, channel
        goto    done            ; no char is ready, so do nothing

        bcf     CH_RDY, channel

        movfw   CHAR0 + channel
        fvcall  convert_char    ; 8 cycles
        movwf   CHAR0 + channel ; 13 cycles to here

        incfsz  BANK0 + channel, W
        goto    store           ; The bank is set, so store char appropriately

        incfsz  CHAR0 + channel, W ; check for binary char
        movfw   CHAR0 + channel ; check for '\r' or '\n'
        btfss   STATUS, Z
        subwf   DISCARD_CHAR0 + channel, W ; check for discard start char
        btfsc   STATUS, Z
        goto    no_store

        movfw   SUPPRESS0 + channel
        andwf   CH_BUSY, W
        btfss   STATUS, Z
        goto    discard_suppress ; this channel is suppressed by a busy channel

        ;; 25 cycles to here

        find_bk BK_FREEL, BK_FREEH, channel ; 16 cycles, so 41 cycles to here

        movwf   BANK0 + channel ; BANK variable for channel set to chosen bank

;;; Carry flag was set by find_bk above and if set continuation will follow

        return                  ; 43 cycles to here

no_store:                       ; 22 cycles to here
        bcf     STATUS, C       ; No continuation later

        movfw   CHAR0 + channel ; check for '\r' or '\n'
        btfsc   STATUS, Z
        goto    freturn_in_18   ; 43 cycles to here, nothing more to be done

        incfsz  CHAR0 + channel, W ; check for binary char
        btfsc   STATUS, Z
        goto    discard_binary

        ;; Discard due to suppression

        movlw   DISCARD_BANK
        movwf   BANK0 + channel ; BANK variable for channel set to discard

        ;; Carry flag not set, so no continuation

        goto    freturn_in_12   ; 43 cycles to here

discard_binary:                 ; 30 cycles to here
        movlb   12
        incfsz  CNT_BINARY, W
        movwf   CNT_BINARY
        movlb   0

        bsf     ERR_CHANNELS, channel

        movlw   DISCARD_BANK
        movwf   BANK0 + channel ; BANK variable for channel set to discard

        ;; Carry flag not set, so no continuation

        goto    freturn_in_6    ; 43 cycles to here

discard_suppress:               ; 26 cycles to here
        movlw   DISCARD_BANK
        movwf   BANK0 + channel ; BANK variable for channel set to discard

        bcf     STATUS, C       ; No continuation later

        goto    freturn_in_14   ; 43 cycles to here

store:                          ; 16 cycles to here
        movfw   CHAR0 + channel
        btfss   STATUS, Z
        incf    CHAR0 + channel, W
        btfsc   STATUS, Z
        goto    finish          ; char '\r', '\n' or binary

        movfw   BANK0 + channel
        sublw   DISCARD_BANK
        btfsc   STATUS, Z
        goto    discard

        ;; 25 cycles to here

        movfw   BANK0 + channel

        addlw   LOW(PTR0)
        movwf   FSR0L           ; FSR0H:L points to pointer for bank

        lslf    INDF0, W
        addlw   0x20
        btfsc   STATUS, C
        goto    overflow        ; We have hit 0x70 or 0xF0 on low part of address, so overflow

        ;; 32 cycles to here

        movfw   INDF0
        incf    INDF0, f        ; Increment pointer

        movwf   FSR0L
        lsrf    BANK0 + channel, W
        movwf   FSR0H           ; FSR0H:L points to storage location

        movfw   CHAR0 + channel
        movwf   INDF0

        ;; 39 cycles to here

        movlw   6
        movwf   FSR0H           ; Reset FSR0H to point to bank 12

        bcf     STATUS, C       ; No continuation later

        return                  ; 43 cycles to here

finish:                         ; 22 cycles to here
        incf    CHAR0 + channel, W
        btfsc   STATUS, Z
        goto    binary

        movfw   BANK0 + channel
        sublw   DISCARD_BANK

        btfsc   STATUS, Z
        goto    finish_discard

        movfw   Q_END
        addlw   LOW(QUEUE)
        movwf   FSR0L           ; FSRH:L points to the element after the last in the queue

        ;; 32 cycles to here

        movfw   BANK0 + channel
        movwf   INDF0           ; Put bank number in the queue
        incf    Q_END, f
        bcf     Q_END, 4        ; start over at 16

        movlw   0xFF
        movwf   BANK0 + channel ; Channel set to waiting

        movlw   TIMER_HIGH      ; reset timer
        movwf   TIMER0H + channel
        clrf    TIMER0L + channel

        bcf     STATUS, C       ; No continuation later

        return                  ; 43 cycles to here

binary:                         ; 26 cycles to here
        movfw   BANK0 + channel
        sublw   DISCARD_BANK
        btfsc   STATUS, Z
        goto    discard_binary2

        bsf     BANK0 + channel, 7 ; Set bit 7 to indicate invalid data

        movlb   12
        incfsz  CNT_BINARY, W
        movwf   CNT_BINARY
        movlb   0

        bsf     ERR_CHANNELS, channel

        bsf     STATUS, C       ; set carry flag for continuation later

        goto    freturn_in_6    ; 43 cycles to here

discard_binary2:                ; 31 cycles to here
        bcf     STATUS, C       ; No continuation later

        goto    freturn_in_11   ; 43 cycles to here


finish_discard:                 ; 30 cycles to here
        movlw   0xFF
        movwf   BANK0 + channel ; Channel set to waiting

        bcf     STATUS, C       ; No continuation later

        goto    freturn_in_10   ; 43 cycles to here

discard:                        ; 26 cycles to here
        bcf     STATUS, C       ; No continuation later

        goto    freturn_in_16   ; 43 cycles to here

overflow:                       ; 33 cycles to here
        bsf     BANK0 + channel, 7 ; Set bit 7 to indicate invalid data
        bsf     STATUS, C       ; set carry flag for continuation later

        movlb   12
        incfsz  CNT_LONG, W
        movwf   CNT_LONG
        movlb   0

        bsf     ERR_CHANNELS, channel

        goto    freturn_in_3    ; 43 cycles to here

done:                           ; 3 cycles to here
        bcf     STATUS, C       ; No continuation later

        movfw   WAITING
        andwf   PHASE, W
        btfss   WREG, channel
        goto    freturn_in_36   ; 43 cycles to here

        movlw   0xFF
        movwf   BANK0 + channel ; Channel set to waiting

        goto    freturn_in_33   ; 43 cycles to here

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 46 = 41 + 5 cycles including nfcall and return. Finishes storage
;;; of a character or does some work sending it. The send2 parameter
;;; indicates what kind of sending work should be done.
mv_char2        macro   channel, send2
        local   finish_bank_setup
        local   invalid

        nopm    2
        btfsc   STATUS, C
        goto    finish_bank_setup ; Continuation from earlier

;;; No continuation, so help out with sending
if (send2)
        call    send_char       ; 31 cycles, so 35 cycles to here
else
        call    send_check      ; 31 cycles, so 35 cycles to here
endif

        goto    freturn_in_6    ; 41 cycles to here

finish_bank_setup:              ; 5 cycles to here
        btfsc   BANK0 + channel, 7
        goto    invalid

        movlw   0x01
        btfsc   BANK0 + channel, 1
        movlw   0x04
        btfsc   BANK0 + channel, 0
        lslf    WREG, f
        btfsc   BANK0 + channel, 2
        swapf   WREG, f

        btfss   BANK0 + channel, 3
        iorwf   NEW_MSGL, f
        btfsc   BANK0 + channel, 3
        iorwf   NEW_MSGH, f

        xorlw   0xFF

        btfss   BANK0 + channel, 3
        andwf   BK_FREEL, f
        btfsc   BANK0 + channel, 3
        andwf   BK_FREEH, f

        movfw   BANK0 + channel

        addlw   LOW(REF0)
        movwf   FSR0L           ; FSR0H:L points to reference for bank

        movlw   channel
        movwf   INDF0           ; Channel number set as reference
        movlw   0x21
        btfsc   BANK0 + channel, 0
        bsf     WREG, 7
        movwi   (PTR0-REF0)[FSR0] ; PTR for bank set to byte after start byte

        movwf   FSR0L
        lsrf    BANK0 + channel, W
        movwf   FSR0H           ; FSR0H:L points to first storage location

        movfw   CHAR0 + channel
        movwi   -1[FSR0]        ; Store first character

        movlw   6
        movwf   FSR0H           ; Reset FSR0H to point to bank 12

        nop

        return                  ; 41 cycles to here

invalid:                        ; 8 cycles to here
        movfw   BANK0 + channel
        call    free_bank       ; 16 cycles, so 25 cycles to here

        movlw   DISCARD_BANK
        movwf   BANK0 + channel ; Discard further data

        goto    freturn_in_14   ; 41 cycles to here

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; Macro for checking if a bit is set.
chk_bit macro   adr, bit

        btfss   adr, bit

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; Macro for checking if a bit is not set.
chk_nbt macro   adr, bit

        btfsc   adr, bit

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 18 cycles. Parses the input reads into bits.
;;;
;;; This is what can happen if the transmitter is fast:
;;;
;;;   0123 <- times
;;;
;;;   0001 waiting
;;;   1110 start bit detected on 0, read byte on 1
;;;   0001 0
;;;   1110 1
;;;   0001 0
;;;   1110 1
;;;   0011 0 <- clock difference occurred
;;;   1100 1
;;;   0011 0
;;;   1100 1 - done
;;;   0011 stop bit on 1, start bit detected on 2, read next byte on 3
;;;
rd_bit  macro   rd0, rd1, rd2, rd3, bit, channel, is_fast

        local   new_result
        local   no_new_result
        local   done
        local   skip_shift_check
        local   shift
        local   done_finish
        local   frame_error
        local   waiting
        local   found0
        local   found2
        local   fe_wait

        btfsc   DONE, channel
        goto    done
        btfsc   WAITING, channel
        goto    waiting
        movfw   rd1
        btfss   PHASE, channel
        movfw   rd3
        bsf     STATUS, C
        chk_bit WREG, bit
        bcf     STATUS, C       ; Carry flag is now the latest bit

        rrf     BUILD0 + channel, f
        btfss   STATUS, C
        goto    no_new_result

new_result:                     ; 13 cycles to get here
        bsf     CH_RDY, channel
        movfw   BUILD0 + channel
        movwf   CHAR0 + channel ; Move to finished char
        bsf     DONE, channel

        return                  ; 18 cycles to here

no_new_result:                  ; 14 cycles to here
        goto    return_in_4     ; 18 cycles to here

done:                           ; 3 cycles to here
        bcf     DONE, channel

        movfw   rd1
        btfss   PHASE, channel
        movfw   rd3
        chk_bit WREG, bit       ; Read stop bit
        goto    frame_error

        btfss   PHASE, channel
        goto    skip_shift_check
        chk_nbt rd2, bit        ; Check for new start bit due to fast
                                ; transmitter clock

skip_shift_check:
        goto    done_finish

;;; A "premature" start bit was detected, so start next character
        bcf     PHASE, channel
        movlw   0x80
        movwf   BUILD0 + channel

        nop

        return                  ; 18 cycles to here

done_finish:                    ; 14 cycles to here
        bsf     WAITING, channel
        bcf     PHASE, channel  ; Indicate that we are going to the normal waiting mode
        nop

        return                  ; 18 cycles to here

frame_error:                    ; 10 cycles to here
        bsf     WAITING, channel
        bsf     PHASE, channel  ; This indicates the frame error
if (is_fast)
        movlw   FRAME_REC_CNT_F
else
        movlw   FRAME_REC_CNT_S
endif
        movwf   BUILD0 + channel ; Counter for frame error recovery
        incfsz  CNT_FRAME, W
        movwf   CNT_FRAME

        bsf     ERR_CHANNELS, channel

        return                  ; 18 cycles to here

waiting:                        ; 5 cycles to here
        btfsc   PHASE, channel
        goto    fe_wait         ; We are waiting for recovery of a fream error

        chk_bit rd0, bit
        goto    found0
        chk_bit rd2, bit
        goto    found2

;;; Nothing found, still waiting
        goto    return_in_7     ; 18 cycles to here

found0:                         ; 10 cycles to here
        bsf     PHASE, channel
        nop

found2:                         ; 12 cycles to here
        movlw   0x80
        movwf   BUILD0 + channel
        bcf     WAITING, channel

        goto    return_in_3     ; 18 cycles to here

fe_wait:                        ; 8 cycles to here
if (is_fast)
        movlw   FRAME_REC_CNT_F
else
        movlw   FRAME_REC_CNT_S
endif
        chk_bit rd0, bit
        movwf   BUILD0 + channel ; Counter for recovery
        chk_bit rd2, bit
        movwf   BUILD0 + channel ; Counter for recovery

        decf    BUILD0 + channel, f
        btfsc   STATUS, Z
        bcf     PHASE, channel  ; We have recovered

        nop

        return                  ; 18 cycles to here

        endm

;;; /////////////////////////////////////////////////////////////////////////////

;;; 13 cycles. if v1h:l < v2h:l then save sh:l in dh:l.
cmp_save        macro   v1h, v1l, v2h, v2l, sh, sl, dh, dl

        local   cmp2
        local   save
        local   done

        movfw   v2h
        subwf   v1h, W
        btfss   STATUS, Z
        goto    cmp2

        movfw   v2l             ; Compare low parts since high parts are the same
        subwf   v1l, W

        btfss   STATUS, C
        goto    save
        nopm    3
        goto    done            ; Delay, then done

cmp2:                           ; 5 cycles to here
        nop
        btfss   STATUS, C
        goto    save
        nopm    3
        goto    done            ; Delay, then done

save:                           ; 9 cycles to here
        movfw   sh
        movwf   dh
        movfw   sl
        movwf   dl

done:                           ; 13 cycles to here

        endm

;;; /////////////////////////////////////////////////////////////////////////////
;;; Code starts here.
;;; /////////////////////////////////////////////////////////////////////////////

near    code    0x0000

        movlw   0x10            ; Take a break of about 100 ms (clock is 500 kHz)
        movwf   CHAR0
        clrf    CHAR0 + 1
start_lp:
        decfsz  CHAR0 + 1, f
        goto    start_lp
        decfsz  CHAR0, f
        goto    start_lp

        nvcall  init
        nvcall  load_user_settings
        nvcall  init2

        nvcall  wait_100ms

        nvcall  init3

main:
        read1   READF0
        call    parse_fs0
        call    parse_fs1
        read1   READF1
        call    parse_fs2
        call    parse_fs3
        read1fs READFS0
        nfcall  store_f0a
        read2   READS0
        nfcall  store_f0b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        nfcall  store_f1a
        read1   READF3
        nopm    2               ; before store_b to time those calls regularly
        nfcall  store_f1b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1fs READFS1
        nfcall  store_f2a
        read2   READS1
        nfcall  store_f2b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        nfcall  store_f3a
        read1   READF3
        nopm    2               ; before store_b to time those calls regularly
        nfcall  store_f3b
        nop                     ; extra cycle

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1fs READFS2
        nfcall  store_s0a
        read2   READS2
        nfcall  store_s0b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        nfcall  store_s1a
        read1   READF3
        nopm    2               ; before store_b to time those calls regularly
        nfcall  store_s1b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1fs READFS3
        call    parse_s0
        call    parse_s1
        read2   READS3
        call    chk_time        ; extra cycle

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        call    parse_s2
        call    parse_s3
        read1   READF3
        call    chk_new_msg

;;; Second round

        read1   READF0
        call    parse_fs0
        call    parse_fs1
        read1   READF1
        call    parse_fs2
        call    parse_fs3
        read1fs READFS0
        nfcall  store_f0a
        read2   READS0
        nfcall  store_f0b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        nfcall  store_f1a
        read1   READF3
        nopm    2               ; before store_b to time those calls regularly
        nfcall  store_f1b
        nop                     ; extra cycle

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1fs READFS1
        nfcall  store_f2a
        read2   READS1
        nfcall  store_f2b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        nfcall  store_f3a
        read1   READF3
        nopm    2               ; before store_b to time those calls regularly
        nfcall  store_f3b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1fs READFS2
        nfcall  store_s2a
        read2   READS2
        nfcall  store_s2b
        nop                     ; extra cycle

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        nfcall  store_s3a
        read1   READF3
        nopm    2               ; before store_b to time those calls regularly
        nfcall  store_s3b

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1fs READFS3
        call    parse_s0
        call    parse_s1
        read2   READS3
        call    chk_input

        read1   READF0
        call    parse_f0
        call    parse_f1
        read1   READF1
        call    parse_f2
        call    parse_f3
        read1   READF2
        call    parse_s2
        call    parse_s3
        read1   READF3
        call    hdl_time

        goto    main            ; extra cycle

;;; /////////////////////////////////////////////////////////////////////////////

;;; 46 cycles incuding call and return. Check for configuration signal
;;; and go to interactive mode if low. Also adjust supression timers
;;; for channels 0-3.

chk_input:
        tm_step 0
        tm_step 1
        tm_step 2
        tm_step 3               ; 32 cycles to here

        btfsc   CONFIG_PORT, CONFIG_PIN
        goto    return_in_10    ; 43 cycles to here

        nvcall  interactive_start

        return                  ; timing does not matter after interactive session

;;; /////////////////////////////////////////////////////////////////////////////

;;; 47 cycles incuding call and return. Adjust supression timers for
;;; channels 4-7.

hdl_time:
        tm_step 4
        tm_step 5
        tm_step 6
        tm_step 7               ; 32 cycles to here

        goto    return_in_12    ; 44 cycles to here

;;; /////////////////////////////////////////////////////////////////////////////

;;; 47 cycles including call and return. Update minimum and maximum
;;; time for main loop.
chk_time:
        movfw   TMR1H
        movwf   TM3H
        movfw   TMR1L
        movwf   TM3L

        lsrf    TM3L, W
        btfsc   STATUS, Z
        incf    TM3H, f         ; Adjust for two cycle difference in reading high/low timer parts

        btfss   TM_VALID_FLAGS, TM_VALID_BIT
        goto    chk_time_invalid

;;; TM3H:L now has the current time, TM0H:L is the last current time, 9 cycles to here

        movfw   TM3L
        subwf   TM0L, f

        movfw   TM3H
        subwfb  TM0H, f

;;; TM0H:L now has the negative of the elapsed time, 13 cycles to here

        cmp_save        TM1H, TM1L, TM0H, TM0L, TM0H, TM0L, TM1H, TM1L ; 13 cycles
        cmp_save        TM0H, TM0L, TM2H, TM2L, TM0H, TM0L, TM2H, TM2L ; 13 cycles

        movfw   TM3H            ; Set TM0H:L to the current time
        movwf   TM0H
        movfw   TM3L
        movwf   TM0L

        return                  ; 44 cycles to here

chk_time_invalid:               ; 10 cycles to here
        bsf     TM_VALID_FLAGS, TM_VALID_BIT

        movfw   TM3H            ; Set TM0H:L to the current time
        movwf   TM0H
        movfw   TM3L
        movwf   TM0L

        goto    return_in_29    ; 44 cycles to here

;;; /////////////////////////////////////////////////////////////////////////////

;;; 48 cycles including call and return. Once every 16384 times (7
;;; seconds), check for a stuck bank, which is one that is not free
;;; and was not used since last time or the round before. When such a
;;; bank is found, it is freed on the next call and recorded as an
;;; error 255 calls later.
chk_new_msg:
        movlb   12

        incfsz  STUCK_CNT1, f
        goto    chk_new_msg_stuck

        incf    STUCK_CNT2, f
        btfss   STUCK_CNT2, 6
        goto    chk_new_msg_record

        clrf    STUCK_CNT2

        movfw   NEW_MSGL
        iorwf   BK_FREEL, W
        iorwf   NEW_MSGLB, W
        xorlw   0xFF
        movwf   NEW_MSGLB       ; Set bit: not used last time or this time and not free

        movfw   NEW_MSGH
        iorwf   BK_FREEH, W
        iorwf   NEW_MSGHB, W
        xorlw   0xFF
        movwf   NEW_MSGHB       ; Set bit: not used last time or this time and not free

        find_bk NEW_MSGLB, NEW_MSGHB, -1 ; 16 cycles, so 33 cycles to here

        movwf   STUCK_BANK

        movfw   NEW_MSGL
        movwf   NEW_MSGLB
        movlw   ~BANK_MASKL
        movwf   NEW_MSGL

        movfw   NEW_MSGH
        movwf   NEW_MSGHB
        movlw   ~BANK_MASKH
        movwf   NEW_MSGH

        movlb   0

        nop
        return                  ; 45 cycles to here

chk_new_msg_stuck:              ; 4 cycles to here
        nop                     ; to balance cycles for chk_new_msg_done

        movfw   STUCK_BANK
        sublw   DISCARD_BANK

        btfsc   STATUS, Z
        goto    chk_new_msg_done

        movfw   STUCK_BANK

        nfcall  free_bank       ; 18 cycles, so 27 cycles to here

        movlw   LOW(REF0)
        addwf   STUCK_BANK, W
        movwf   FSR0L           ; FSR0H:L now points to REF

        movfw   INDF0
        movwf   STUCK_CHANNEL

        movlw   LOW(BANK0)
        addwf   INDF0, W
        movwf   FSR1L
        clrf    FSR1H           ; FSR1H:L now points to BANK
        movlw   0xFF
        movwf   INDF1           ; Mark bank as unused

        movlw   DISCARD_BANK
        movwf   STUCK_BANK

        movlb   0

        goto    return_in_3     ; 45 cycles to here

chk_new_msg_record:             ; 7 cycles to here
        btfsc   STUCK_CHANNEL, 7
        goto    chk_new_msg_done

        movlw   0x01
        btfsc   STUCK_CHANNEL, 1
        movlw   0x04
        btfsc   STUCK_CHANNEL, 0
        lslf    WREG, f
        btfsc   STUCK_CHANNEL, 2
        swapf   WREG, f         ; W now has the appropriate bit set

        movlb   0

        iorwf   ERR_CHANNELS, f

        movlb   12

        incfsz  CNT_SLOW, W
        movwf   CNT_SLOW

        movlw   0xFF
        movwf   STUCK_CHANNEL   ; mark as no error to be recorded

        movlb   0

        goto    return_in_21    ; 45 cycles to here

chk_new_msg_done:               ; 10 cycles to here
        movlb   0

        goto    return_in_34    ; 45 cycles to here

;;; /////////////////////////////////////////////////////////////////////////////

;;; 24 cycles including call and return. Parse slow channels.
parse_s0:
        nopm    3
        rd_bit  READS3, READS0, READS1, READS2, SLOW0_PIN, SLOW0_NUM, 0

;;; //////////

parse_s1:
        nopm    3
        rd_bit  READS3, READS0, READS1, READS2, SLOW1_PIN, SLOW1_NUM, 0

;;; //////////

parse_s2:
        nopm    3
        rd_bit  READS0, READS1, READS2, READS3, SLOW2_PIN, SLOW2_NUM, 0

;;; //////////

parse_s3:
        nopm    3
        rd_bit  READS0, READS1, READS2, READS3, SLOW3_PIN, SLOW3_NUM, 0

;;; /////////////////////////////////////////////////////////////////////////////

;;; 24 cycles including call and return. Parse fast channels as either
;;; fast or slow.
parse_fs0:
        btfss   FAST_FLAGS, FAST_BIT0
        goto    slow0_first
        nop
        rd_bit  READF1, READF2, READF3, READF0, FAST0_PIN, FAST0_NUM, 1
slow0_first:
        rd_bit  READFS0, READFS1, READFS2, READFS3, FAST0_PIN, FAST0_NUM, 0

;;; //////////

;;; 24 cycles including call and return. Parse fast channel but do
;;; nothing if it is set to slow.
parse_f0:
        btfss   FAST_FLAGS, FAST_BIT0
        goto    return_in_20    ; 21 cycles to here
        nop
        rd_bit  READF1, READF2, READF3, READF0, FAST0_PIN, FAST0_NUM, 1

;;; //////////

parse_fs1:
        btfss   FAST_FLAGS, FAST_BIT1
        goto    slow1_first
        nop
        rd_bit  READF1, READF2, READF3, READF0, FAST1_PIN, FAST1_NUM, 1
slow1_first:
        rd_bit  READFS0, READFS1, READFS2, READFS3, FAST1_PIN, FAST1_NUM, 0

;;; //////////

parse_f1:
        btfss   FAST_FLAGS, FAST_BIT1
        goto    return_in_20
        nop
        rd_bit  READF1, READF2, READF3, READF0, FAST1_PIN, FAST1_NUM, 1

;;; //////////

parse_fs2:
        btfss   FAST_FLAGS, FAST_BIT2
        goto    slow2_first
        nop
        rd_bit  READF2, READF3, READF0, READF1, FAST2_PIN, FAST2_NUM, 1
slow2_first:
        rd_bit  READFS0, READFS1, READFS2, READFS3, FAST2_PIN, FAST2_NUM, 0

;;; //////////

parse_f2:
        btfss   FAST_FLAGS, FAST_BIT2
        goto    return_in_20    ; 21 cycles to here
        nop
        rd_bit  READF2, READF3, READF0, READF1, FAST2_PIN, FAST2_NUM, 1

;;; //////////

parse_fs3:
        btfss   FAST_FLAGS, FAST_BIT3
        goto    slow3_first
        nop
        rd_bit  READF2, READF3, READF0, READF1, FAST3_PIN, FAST3_NUM, 1
slow3_first:
        rd_bit  READFS0, READFS1, READFS2, READFS3, FAST3_PIN, FAST3_NUM, 0

;;; //////////

parse_f3:
        btfss   FAST_FLAGS, FAST_BIT3
        goto    return_in_20    ; 21 cycles to here
        nop
        rd_bit  READF2, READF3, READF0, READF1, FAST3_PIN, FAST3_NUM, 1

;;; /////////////////////////////////////////////////////////////////////////////

;;; A goto to one of these is a delayed return. "goto return_in_n" is
;;; equivalent to n-1 nop instructions before a return.
return_in_37:
        nop
return_in_36:
        nop
return_in_35:
        nop
return_in_34:
        nop
return_in_33:
        nop
return_in_32:
        nop
return_in_31:
        nop
return_in_30:
        nop
return_in_29:
        nop
return_in_28:
        nop
return_in_27:
        nop
return_in_26:
        nop
return_in_25:
        nop
return_in_24:
        nop
return_in_23:
        nop
return_in_22:
        nop
return_in_21:
        nop
return_in_20:
        nop
return_in_19:
        nop
return_in_18:
        nop
return_in_17:
        nop
return_in_16:
        nop
return_in_15:
        nop
return_in_14:
        nop
return_in_13:
        nop
return_in_12:
        nop
return_in_11:
        nop
return_in_10:
        nop
return_in_9:
        nop
return_in_8:
        nop
return_in_7:
        nop
return_in_6:
        nop
return_in_5:
        nop
return_in_4:
        nop
return_in_3:
        return

;;; /////////////////////////////////////////////////////////////////////////////
;;; Far section starts here.
;;; /////////////////////////////////////////////////////////////////////////////

far     code    0x0800

;;; /////////////////////////////////////////////////////////////////////////////

;;; 48 cycles. Stores characters in banks.
store_s0a:
        mv_char SLOW0_NUM

;;; //////////

store_s1a:
        mv_char SLOW1_NUM

;;; //////////

store_s2a:
        mv_char SLOW2_NUM

;;; //////////

store_s3a:
        mv_char SLOW3_NUM

;;; //////////

store_f0a:
        mv_char FAST0_NUM

;;; //////////

store_f1a:
        mv_char FAST1_NUM

;;; //////////

store_f2a:
        mv_char FAST2_NUM

;;; //////////

store_f3a:
        mv_char FAST3_NUM

;;; /////////////////////////////////////////////////////////////////////////////

;;; 46 cycles including nfcall and return. Finishes storage or sends
;;; data.
store_s0b:
        mv_char2 SLOW0_NUM, 1

;;; //////////

store_s1b:
        mv_char2 SLOW1_NUM, 0

;;; //////////

store_s2b:
        mv_char2 SLOW2_NUM, 1

;;; //////////

store_s3b:
        mv_char2 SLOW3_NUM, 0

;;; //////////

store_f0b:
        mv_char2 FAST0_NUM, 1

;;; //////////

store_f1b:
        mv_char2 FAST1_NUM, 0

;;; //////////

store_f2b:
        mv_char2 FAST2_NUM, 1

;;; //////////

store_f3b:
        mv_char2 FAST3_NUM, 0

;;; /////////////////////////////////////////////////////////////////////////////

;;; 31 cycles including call and return. If needed, move a char from
;;; our own transmit queue to the built-in one.
send_char:
        nopm    8

        btfss   SD_CH_FLAGS, SD_CH_BIT
        goto    freturn_in_19   ; 28 cycles to here, no char to send

        btfss   PIR1, TXIF
        goto    freturn_in_17   ; 28 cycles to here, no room

        btfsc   T2CON, TMR2ON
        btfsc   PIR1, TMR2IF
        goto    send_char_ok    ; timer not running or has expired

        goto    freturn_in_13   ; 28 cycles to here, timer running and not expired

send_char_ok:                   ; 16 cycles to here
        bcf     PIR1, TMR2IF
        bcf     T2CON, TMR2ON

        movfw   SEND_CHAR
        sublw   '\n'
        btfsc   STATUS, Z
        bsf     T2CON, TMR2ON   ; start timer when newline

        movfw   SEND_CHAR
        bcf     SD_CH_FLAGS, SD_CH_BIT

        movlb   3
        movwf   TXREG
        movlb   0

        return                  ; 28 cycles to here

;;; /////////////////////////////////////////////////////////////////////////////

;;; 31 cycles including call and return. Check if we need to do
;;; something regarding sending.
send_check:
        btfss   SEND_BK, 7
        goto    send_check_done ; We are already sending

        movfw   Q_END
        subwf   Q_START, W

        btfsc   STATUS, Z
        goto    freturn_in_23   ; Nothing is awaiting being sent, 28 cycles to here

        movfw   Q_START
        addlw   LOW(QUEUE)
        movwf   FSR0L           ; FSRH:L points to the first element in the queue

        movfw   INDF0           ; W = bank to be sent
        movwf   SEND_BK

        btfsc   CHN_OUT_FLAGS, CHN_OUT_BIT
        bsf     SEND_BK, 4
        bsf     SEND_BK, 6

        incf    Q_START, f
        bcf     Q_START, 4      ; start over at 16

        goto    freturn_in_12   ; 28 cycles to here

send_check_done:                ; 3 cycles to here
        call    send            ; 24 cycles, so 27 cycles to here

        return                  ; 28 cycles to here

;;; /////////////////////////////////////////////////////////////////////////////

;;; SEND_BK   10000000: not sending
;;; SEND_BK   0100xxxx: getting ready to send from bank xxxx
;;; SEND_BK   0101xxxx: getting ready to send from bank xxxx
;;; SEND_BK   0010xxxx: finish of sending from bank xxxx
;;; SEND_BK   0011xxxx: more finish of sending from bank xxxx
;;; SEND_BK   0000xxxx: sending from bank xxxx

;;; 24 cycles including call and return. Sends data from banks.
send:
        btfsc   SEND_BK, 5
        goto    send_finish
        btfsc   SEND_BK, 6
        goto    send_setup
        btfsc   SEND_BK, 7
        goto    freturn_in_16   ; 21 cycles to here

do_send:                        ; 6 cycles to here
        btfsc   SD_CH_FLAGS, SD_CH_BIT
        goto    freturn_in_14   ; No room for next char, 21 cycles to here

        movfw   SEND_END
        subwf   FSR1L, W

        btfsc   STATUS, Z
        goto    do_send_last

;;; Not last position
        movfw   INDF1

        movwf   SEND_CHAR
        bsf     SD_CH_FLAGS, SD_CH_BIT

        incf    FSR1L, f        ; Move pointer

        goto    freturn_in_5    ; 21 cycles to here

do_send_last:                   ; 13 cycles to here
        bsf     SEND_BK, 5      ; Stop sending

        movlw   '\r'
        btfss   NEWLINE_FLAGS, NEWLINE_BIT
        movlw   '\n'

        movwf   SEND_CHAR
        bsf     SD_CH_FLAGS, SD_CH_BIT

        nop
        return                  ; 21 cycles to here

send_setup:                     ; 5 cycles to here
        btfsc   SEND_BK, 4
        goto    send_setup2
        movlw   LOW(PTR0)
        movwf   FSR0L

        bcf     SEND_BK, 6
        movfw   SEND_BK
        movwf   FSR1H
        addwf   FSR0L, f        ; FSR0H:L points to pointer for relevant bank

        movfw   INDF0
        movwf   SEND_END        ; SEND_END is the pointer to the byte after the last in the bank

        movlw   0x40
        movwf   FSR1L
        lsrf    FSR1H, f
        rrf     FSR1L, f        ; FSR1H:L points to first byte of data to be sent

        nop

        return                  ; 21 cycles to here

send_setup2:                    ; 8 cycles to here
        btfsc   SD_CH_FLAGS, SD_CH_BIT
        goto    freturn_in_12   ; No room for next char, 21 cycles to here

        bcf     SEND_BK, 4

        movfw   SEND_BK
        addlw   LOW(REF0) - 0x40 ; Account for bit 6 being set in SEND_BK
        movwf   FSR0L           ; FSR0H:L points to reference

        movfw   INDF0
        addlw   '1'

        movwf   SEND_CHAR
        bsf     SD_CH_FLAGS, SD_CH_BIT

        nopm    2

        return                  ; 21 cycles to here

send_finish:                    ; 3 cycles to here
        btfss   NEWLINE_FLAGS, NEWLINE_BIT
        goto    send_finish1
;;; Return-newline mode

        btfsc   SEND_BK, 4
        goto    send_finish2

        movlw   0x01
        btfsc   SEND_BK, 1
        movlw   0x04
        btfsc   SEND_BK, 0
        lslf    WREG, f
        btfsc   SEND_BK, 2
        swapf   WREG, f
        btfss   SEND_BK, 3
        iorwf   BK_FREEL, f
        btfsc   SEND_BK, 3
        iorwf   BK_FREEH, f

;;; The bank is now marked as free
        bsf     SEND_BK, 4      ; do send_finish2 next time

        nop
        return                  ; 21 cycles to here

;;; Newline only mode
send_finish1:                   ; 6 cycles to here
        movlw   0x01
        btfsc   SEND_BK, 1
        movlw   0x04
        btfsc   SEND_BK, 0
        lslf    WREG, f
        btfsc   SEND_BK, 2
        swapf   WREG, f
        btfss   SEND_BK, 3
        iorwf   BK_FREEL, f
        btfsc   SEND_BK, 3
        iorwf   BK_FREEH, f

;;; The bank is now marked as free
        movlw   0x80
        movwf   SEND_BK         ; We are done sending

        nop
        return                  ; 21 cycles to here

send_finish2:                   ; 8 cycles to here
        btfsc   SD_CH_FLAGS, SD_CH_BIT
        goto    freturn_in_12   ; No room for next char, 21 cycles to here

        movlw   '\n'

        movwf   SEND_CHAR
        bsf     SD_CH_FLAGS, SD_CH_BIT

        movlw   0x80
        movwf   SEND_BK         ; We are done sending

        goto    freturn_in_6    ; 21 cycles to here

;;; /////////////////////////////////////////////////////////////////////////////

;;; 16 cycles including call and return. Free the bank given by W.
free_bank:
        movwf   FSR0L           ; FSR0L is not used for anything else
        movlw   0x01
        btfsc   FSR0L, 1
        movlw   0x04
        btfsc   FSR0L, 0
        lslf    WREG, f
        btfsc   FSR0L, 2
        swapf   WREG, f
        btfss   FSR0L, 3
        iorwf   BK_FREEL, f
        btfsc   FSR0L, 3
        iorwf   BK_FREEH, f

        return                  ; 13 cycles to here

;;; /////////////////////////////////////////////////////////////////////////////

;;; A goto to one of these is a delayed return here in the far section.
freturn_in_36:
        nop
freturn_in_35:
        nop
freturn_in_34:
        nop
freturn_in_33:
        nop
freturn_in_32:
        nop
freturn_in_31:
        nop
freturn_in_30:
        nop
freturn_in_29:
        nop
freturn_in_28:
        nop
freturn_in_27:
        nop
freturn_in_26:
        nop
freturn_in_25:
        nop
freturn_in_24:
        nop
freturn_in_23:
        nop
freturn_in_22:
        nop
freturn_in_21:
        nop
freturn_in_20:
        nop
freturn_in_19:
        nop
freturn_in_18:
        nop
freturn_in_17:
        nop
freturn_in_16:
        nop
freturn_in_15:
        nop
freturn_in_14:
        nop
freturn_in_13:
        nop
freturn_in_12:
        nop
freturn_in_11:
        nop
freturn_in_10:
        nop
freturn_in_9:
        nop
freturn_in_8:
        nop
freturn_in_7:
        nop
freturn_in_6:
        nop
freturn_in_5:
        nop
freturn_in_4:
        nop
freturn_in_3:
        return

;;; /////////////////////////////////////////////////////////////////////////////
;;; Veryfar section starts here.
;;; /////////////////////////////////////////////////////////////////////////////

veryfar code    0x1000

if (FAST_BIT0 != 0 || FAST_BIT1 != 1 || FAST_BIT2 != 2 || FAST_BIT3 != 3)
        ERROR   "FAST_BITs assumed to be 0, 1, 2, and 3"
endif

;;; /////////////////////////////////////////////////////////////////////////////

;;; Load user settings from program memory.
load_user_settings:
        movlw   LOW(user_settings)
        movwf   FSR1L
        movlw   HIGH(user_settings)
        movwf   FSR1H

        goto    load_settings

;;; /////////////////////////////////////////////////////////////////////////////

;;; Load factory settings from program memory.
load_factory_settings:
        movlw   LOW(factory_settings)
        movwf   FSR1L
        movlw   HIGH(factory_settings)
        movwf   FSR1H

        goto    load_settings

;;; /////////////////////////////////////////////////////////////////////////////

;;; Load settings from FSR1æ
load_settings:
        moviw   FSR1++
        movwf   SUPPRESS0 + 0

        moviw   FSR1++
        movwf   SUPPRESS0 + 1

        moviw   FSR1++
        movwf   SUPPRESS0 + 2

        moviw   FSR1++
        movwf   SUPPRESS0 + 3

        moviw   FSR1++
        movwf   SUPPRESS0 + 4

        moviw   FSR1++
        movwf   SUPPRESS0 + 5

        moviw   FSR1++
        movwf   SUPPRESS0 + 6

        moviw   FSR1++
        movwf   SUPPRESS0 + 7

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 0

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 1

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 2

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 3

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 4

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 5

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 6

        moviw   FSR1++
        movwf   DISCARD_CHAR0 + 7

        moviw   FSR1++
        bsf     CHN_OUT_FLAGS, CHN_OUT_BIT
        btfss   WREG, 0
        bcf     CHN_OUT_FLAGS, CHN_OUT_BIT

        movfw   FAST_FLAGS
        andlw   0xF0
        movwf   FAST_FLAGS
        moviw   FSR1++
        iorwf   FAST_FLAGS, f

        moviw   FSR1++
        bsf     NEWLINE_FLAGS, NEWLINE_BIT
        btfss   WREG, 0
        bcf     NEWLINE_FLAGS, NEWLINE_BIT

        moviw   FSR1++
        call    inter_set_invert

        moviw   FSR1++
        call    inter_set_invert_out

        moviw   FSR1++
        movlb   12
        movwf   INTER_SPEED
        movlb   0

        moviw   FSR1++
        call    inter_set_schmitt

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Save current settings as user settings.
save_user_settings:
        call    write_start

        movfw   SUPPRESS0 + 0
        call    save_byte

        movfw   SUPPRESS0 + 1
        call    save_byte

        movfw   SUPPRESS0 + 2
        call    save_byte

        movfw   SUPPRESS0 + 3
        call    save_byte

        movfw   SUPPRESS0 + 4
        call    save_byte

        movfw   SUPPRESS0 + 5
        call    save_byte

        movfw   SUPPRESS0 + 6
        call    save_byte

        movfw   SUPPRESS0 + 7
        call    save_byte

        movfw   DISCARD_CHAR0 + 0
        call    save_byte

        movfw   DISCARD_CHAR0 + 1
        call    save_byte

        movfw   DISCARD_CHAR0 + 2
        call    save_byte

        movfw   DISCARD_CHAR0 + 3
        call    save_byte

        movfw   DISCARD_CHAR0 + 4
        call    save_byte

        movfw   DISCARD_CHAR0 + 5
        call    save_byte

        movfw   DISCARD_CHAR0 + 6
        call    save_byte

        movfw   DISCARD_CHAR0 + 7
        call    save_byte

        movlw   0x01
        btfss   CHN_OUT_FLAGS, CHN_OUT_BIT
        movlw   0x00
        call    save_byte

        movfw   FAST_FLAGS
        andlw   0x0F
        call    save_byte

        movlw   0x01
        btfss   NEWLINE_FLAGS, NEWLINE_BIT
        movlw   0x00
        call    save_byte

        call    inter_get_invert
        call    save_byte

        call    inter_get_invert_out
        call    save_byte

        movlb   12
        movfw   INTER_SPEED
        movlb   0
        call    save_byte

        call    inter_get_schmitt
        call    save_last_byte

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Erase user settings memory and get read to write.
write_start:
;;; We erase the user settings by erasing one row of 32 words
        movlb   3

        movlw   LOW(user_settings)
        movwf   PMADRL
        movlw   HIGH(user_settings)
        movwf   PMADRH

        bcf     PMCON1, CFGS    ; Normal program memory
        bsf     PMCON1, FREE    ; Free memory
        bsf     PMCON1, WREN    ; Write enable

        movlw   0x55            ; Unlock sequence start
        movwf   PMCON2
        movlw   0xAA
        movwf   PMCON2
        bsf     PMCON1, WR
        nop
        nop                     ; Unlock sequence end

;;; Here, we are getting ready to write:
        movlw   0x34            ; The program instruction written is
                                ; of type retlw
        movwf   PMDATH          ; Low byte will be supplied later

        bsf     PMCON1, LWLO    ; Only output to latches

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Save a byte to program memory latches.
save_byte:
        movlb   3

        movwf   PMDATL          ; High byte already set

        movlw   0x55            ; Unlock sequence start
        movwf   PMCON2
        movlw   0xAA
        movwf   PMCON2
        bsf     PMCON1, WR
        nop
        nop                     ; Unlock sequence end

        incf    PMADRL, f

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Save a byte along with previous latches to program memory.
save_last_byte:
        movlb   3
        bcf     PMCON1, LWLO    ; Store to everything memory
        movlb   0

        call    save_byte

        movlb   3
        bcf     PMCON1, WREN    ; Disable write
        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set W according to which channels are inverted.
inter_get_invert:
        clrw

        btfsc   INVERT_S, SLOW0_PIN
        bsf     WREG, SLOW0_NUM

        btfsc   INVERT_S, SLOW1_PIN
        bsf     WREG, SLOW1_NUM

        btfsc   INVERT_S, SLOW2_PIN
        bsf     WREG, SLOW2_NUM

        btfsc   INVERT_S, SLOW3_PIN
        bsf     WREG, SLOW3_NUM

        btfsc   INVERT_F, FAST0_PIN
        bsf     WREG, FAST0_NUM

        btfsc   INVERT_F, FAST1_PIN
        bsf     WREG, FAST1_NUM

        btfsc   INVERT_F, FAST2_PIN
        bsf     WREG, FAST2_NUM

        btfsc   INVERT_F, FAST3_PIN
        bsf     WREG, FAST3_NUM

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set which channels are inverted from W.
inter_set_invert:
        clrf    INVERT_S

        btfsc   WREG, SLOW0_NUM
        bsf     INVERT_S, SLOW0_PIN

        btfsc   WREG, SLOW1_NUM
        bsf     INVERT_S, SLOW1_PIN

        btfsc   WREG, SLOW2_NUM
        bsf     INVERT_S, SLOW2_PIN

        btfsc   WREG, SLOW3_NUM
        bsf     INVERT_S, SLOW3_PIN

        clrf    INVERT_F

        btfsc   WREG, FAST0_NUM
        bsf     INVERT_F, FAST0_PIN

        btfsc   WREG, FAST1_NUM
        bsf     INVERT_F, FAST1_PIN

        btfsc   WREG, FAST2_NUM
        bsf     INVERT_F, FAST2_PIN

        btfsc   WREG, FAST3_NUM
        bsf     INVERT_F, FAST3_PIN

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set W according to whether output is inverted.
inter_get_invert_out:
        movlb   3

        movlw   0x00
        btfsc   BAUD1CON, SCKP
        movlw   0x01

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set output is inversion according to W.
inter_set_invert_out:
        movlb   3

        bcf     BAUD1CON, SCKP
        btfsc   WREG, 0
        bsf     BAUD1CON, SCKP

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set W according to which channels are using Schmitt triggers.
inter_get_schmitt:
        clrw

        movlb   7

;;; In the following cade, SLOW_PORT and FAST_PORT refers to the
;;; appropriate INLVLA/C registers since we are in bank 7.

        btfsc   SLOW_PORT, SLOW0_PIN
        bsf     WREG, SLOW0_NUM

        btfsc   SLOW_PORT, SLOW1_PIN
        bsf     WREG, SLOW1_NUM

        btfsc   SLOW_PORT, SLOW2_PIN
        bsf     WREG, SLOW2_NUM

        btfsc   SLOW_PORT, SLOW3_PIN
        bsf     WREG, SLOW3_NUM

        btfsc   FAST_PORT, FAST0_PIN
        bsf     WREG, FAST0_NUM

        btfsc   FAST_PORT, FAST1_PIN
        bsf     WREG, FAST1_NUM

        btfsc   FAST_PORT, FAST2_PIN
        bsf     WREG, FAST2_NUM

        btfsc   FAST_PORT, FAST3_PIN
        bsf     WREG, FAST3_NUM

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set which channels are using Schmitt triggers from W.
inter_set_schmitt:
        movlb   7

;;; In the following cade, SLOW_PORT and FAST_PORT refers to the
;;; appropriate INLVLA/C registers since we are in bank 7.

        bcf     SLOW_PORT, SLOW0_PIN
        btfsc   WREG, SLOW0_NUM
        bsf     SLOW_PORT, SLOW0_PIN

        bcf     SLOW_PORT, SLOW1_PIN
        btfsc   WREG, SLOW1_NUM
        bsf     SLOW_PORT, SLOW1_PIN

        bcf     SLOW_PORT, SLOW2_PIN
        btfsc   WREG, SLOW2_NUM
        bsf     SLOW_PORT, SLOW2_PIN

        bcf     SLOW_PORT, SLOW3_PIN
        btfsc   WREG, SLOW3_NUM
        bsf     SLOW_PORT, SLOW3_PIN

        bcf     FAST_PORT, FAST0_PIN
        btfsc   WREG, FAST0_NUM
        bsf     FAST_PORT, FAST0_PIN

        bcf     FAST_PORT, FAST1_PIN
        btfsc   WREG, FAST1_NUM
        bsf     FAST_PORT, FAST1_PIN

        bcf     FAST_PORT, FAST2_PIN
        btfsc   WREG, FAST2_NUM
        bsf     FAST_PORT, FAST2_PIN

        bcf     FAST_PORT, FAST3_PIN
        btfsc   WREG, FAST3_NUM
        bsf     FAST_PORT, FAST3_PIN

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Start the interactive mode. Generally, the calls that read from
;;; the serial port set the zero flag if an error occurs. In this
;;; case, the last read char is put in W. The functions that read also
;;; return without doing anything if the zero flag is already
;;; set. That way a lot of read calls can be made in a row and if one
;;; of them gets unexpected input, the zero flag is set and W is the
;;; last wrong char. In this case, the code will go to
;;; inter_error_just_read that will go to the first newline (which may
;;; be the last read char in W) and print an error.
interactive_start:
        movlw   '\r'
        btfsc   NEWLINE_FLAGS, NEWLINE_BIT
        call    write_char
        movlw   '\n'
        call    write_char

        call    wait_100ms

        movlw   '\r'
        btfsc   NEWLINE_FLAGS, NEWLINE_BIT
        call    write_char
        movlw   '\n'
        call    write_char

        call    wait_100ms

        movlw   '\r'
        btfsc   NEWLINE_FLAGS, NEWLINE_BIT
        call    write_char
        movlw   '\n'
        call    write_char

        call    wait_100ms

        movlw   '\n'            ; No return-newline in debug mode
        call    write_char

        movlw   '\n'
        call    write_char

        call    clear_input

        call    speed_4800
        movlb   3
        bsf     RC1STA, CREN    ; Enable receive
        movlb   0

        goto    interactive_no_ok

interactive:                    ; This is where all the following goto's come back to
        movlw   'O'
        call    write_char

        movlw   'k'
        call    write_char

        movlw   '\n'
        call    write_char

interactive_no_ok:
        call    read_cmd

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movfw   INTER_VALUE
        addlw   -'I'
        btfsc   STATUS, Z
        goto    inter_cmd_invert

        addlw   'I' - 'J'
        btfsc   STATUS, Z
        goto    inter_cmd_invert_out

        addlw   'J' - 'C'
        btfsc   STATUS, Z
        goto    inter_cmd_channel_output

        addlw   'C' - 'N'
        btfsc   STATUS, Z
        goto    inter_cmd_return_newline

        addlw   'N' - 'D'
        btfsc   STATUS, Z
        goto    inter_cmd_discard

        addlw   'D' - 'F'
        btfsc   STATUS, Z
        goto    inter_cmd_fast

        addlw   'F' - 'U'
        btfsc   STATUS, Z
        goto    inter_cmd_suppress

        addlw   'U' - 'H'
        btfsc   STATUS, Z
        goto    inter_cmd_schmitt

        addlw   'H' - 'B'
        btfsc   STATUS, Z
        goto    inter_cmd_speed

        addlw   'B' - 'P'
        btfsc   STATUS, Z
        goto    inter_cmd_output

        addlw   'P' - 'G'
        btfsc   STATUS, Z
        goto    inter_cmd_debug

        addlw   'G' - 'L'
        btfsc   STATUS, Z
        goto    inter_cmd_load

        addlw   'L' - 'S'
        btfsc   STATUS, Z
        goto    inter_cmd_save

        addlw   'S' - 'R'
        btfsc   STATUS, Z
        goto    inter_cmd_factory

inter_error_lp:
        call    read_char

inter_error_just_read:          ; goto this place when a wrong char has just been read
        btfsc   CONFIG_PORT, CONFIG_PIN
        goto    inter_done

        sublw   '\n'
        btfss   STATUS, Z
        goto    inter_error_lp

inter_error:
        movlw   'E'
        call    write_char
        movlw   'r'
        call    write_char
        movlw   'r'
        call    write_char
        movlw   'o'
        call    write_char
        movlw   'r'
        call    write_char
        movlw   '\n'
        call    write_char

        goto    interactive_no_ok

;;; /////////////////////////////////////////////////////////////////////////////

;;; Write W as a char to the serial output-
write_char:
        btfss   PIR1, TXIF
        goto    $-1             ; Wait for space in buffer

        movlb   3
        movwf   TXREG           ; Send char
        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Convert the last four bits of W to a hex digit in W.
nibble_to_hex:
        andlw   0x0F
        sublw   0x09
        btfss   STATUS, C
        addlw   0xF9
        sublw   0x39
        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Write W as a hex number to the serial output.
write_hex:
        movwf   INTER_TMP

        swapf   INTER_TMP, W
        call    nibble_to_hex
        call    write_char

        movfw   INTER_TMP
        call    nibble_to_hex
        call    write_char

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Clear the serial port of input.
clear_input:
        movlb   3

        movfw   RC1REG          ; The buffer is small, so four reads is ok
        movfw   RC1REG
        movfw   RC1REG
        movfw   RC1REG

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Read a character from the serial port.
read_char:
        movlb   3

        btfsc   RC1STA, OERR
        bcf     RC1STA, CREN    ; Clear CREN on error
        bsf     RC1STA, CREN

        movlb   0

read_char_lp:
        btfsc   CONFIG_PORT, CONFIG_PIN
        return                  ; Return if CONFIG signal high (should not be in interactive mode)

        btfss   PIR1, RCIF
        goto    read_char_lp    ; Wait for data

        movlb   3
        movfw   RC1REG
        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Read an input command.
read_cmd:
        call    read_char

        bsf     STATUS, Z
        btfsc   CONFIG_PORT, CONFIG_PIN
        return                  ; Return error if CONFIG signal high (should not be in interactive mode)

        movwf   INTER_VALUE
        sublw   '\n'
        btfss   STATUS, Z
        return                  ; Success
        sublw   '\n'            ; Set W to the originally read char (\n)
        bsf     STATUS, Z
        return                  ; Error

;;; /////////////////////////////////////////////////////////////////////////////

;;; Read a number from 0 to W (included and <= 9), put it in W.
read_num:
        btfsc   STATUS, Z
        return

        movwf   INTER_TMP
        call    read_char
        addlw   -'0'
        subwf   INTER_TMP, W
        btfss   STATUS, C
        goto    read_num_error

read_num_success:
        subwf   INTER_TMP, W
        bcf     STATUS, Z
        return                  ; Success

read_num_error:
        subwf   INTER_TMP, W    ; Restore W to read char
        addlw   '0'
        bsf     STATUS, Z
        return                  ; Error

;;; /////////////////////////////////////////////////////////////////////////////

;;; Read an input character of 1-8 and return 0-7 (i.e. one less).
read_channel:
        movlw   8
        call    read_num
        btfsc   STATUS, Z
        return                  ; Error

        addlw   0xFF            ; Subtract one
        bcf     STATUS, Z
        btfsc   STATUS, C
        return                  ; Success

        bsf     STATUS, Z       ; We have read '0' which is not a channel
        movlw   '0'             ; Restore W to read char
        return                  ; Error

;;; /////////////////////////////////////////////////////////////////////////////

;;; Read a single hex number and put it in W.
read_hex_sgl:
        btfsc   STATUS, Z
        return

        call    read_char
        addlw   -'A'
        btfsc   STATUS, C
        goto    read_hex_A_F

read_hex_0_9:
        addlw   'A'-'0'
        sublw   9
        btfss   STATUS, C
        goto    read_hex_error_0_9
        sublw   9
        bcf     STATUS, Z
        return

read_hex_error_0_9:
        sublw   9               ; Restore W to read char
        addlw   '0'
        bsf     STATUS, Z
        return

read_hex_A_F:
        sublw   5
        btfss   STATUS, C
        goto    read_hex_error_A_F
        sublw   5
        addlw   0x0A
        bcf     STATUS, Z
        return

read_hex_error_A_F:
        sublw   5 
        addlw   'A'
        bsf     STATUS, Z
        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Read a double digit hex number and put it in W.
read_hex_dbl:
        btfsc   STATUS, Z
        return

        call    read_hex_sgl
        movwf   INTER_TMP

        call    read_hex_sgl

        btfsc   STATUS, Z
        return

        swapf   INTER_TMP, f
        iorwf   INTER_TMP, W

        bcf     STATUS, Z
        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Read a newline (or return followed by newline).
read_newline:
        btfsc   STATUS, Z
        return

        call    read_char
        sublw   '\n'
        btfss   STATUS, Z
        goto    read_newline_error

read_newline_success:
        bcf     STATUS, Z
        return

read_newline_error:
        sublw   '\n'
        sublw   '\r'
        btfsc   STATUS, Z
        goto    read_newline_return ; We read a return

        sublw   '\r'            ; Restore W to read char
        bsf     STATUS, Z       ; Return error
        return

read_newline_return:
        bcf     STATUS, Z       ; Clear error and try again
        goto    read_newline

;;; /////////////////////////////////////////////////////////////////////////////

;;; A number of functions for all the commands-
inter_cmd_channel_output:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        movlw   1
        call    read_num
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        bsf     CHN_OUT_FLAGS, CHN_OUT_BIT
        btfss   INTER_VALUE, 0
        bcf     CHN_OUT_FLAGS, CHN_OUT_BIT

        goto    interactive

;;; //////////

inter_cmd_fast:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_hex_sgl
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movfw   FAST_FLAGS
        andlw   0xF0
        iorwf   INTER_VALUE, W
        movwf   FAST_FLAGS

        goto    interactive

;;; //////////

inter_cmd_return_newline:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        movlw   1
        call    read_num
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        bsf     NEWLINE_FLAGS, NEWLINE_BIT
        btfss   INTER_VALUE, 0
        bcf     NEWLINE_FLAGS, NEWLINE_BIT

        goto    interactive

;;; //////////

inter_cmd_invert:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_hex_dbl
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movfw   INTER_VALUE
        call    inter_set_invert

        goto    interactive

;;; //////////

inter_cmd_invert_out:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        movlw   1
        call    read_num
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movfw   INTER_VALUE
        call    inter_set_invert_out

        call    wait_100ms

        movlw   '\n'
        call    write_char      ; Try to flush output after change

        call    wait_100ms

        movlw   '\n'
        call    write_char      ; Try to flush output after change

        call    wait_100ms

        movlw   '\n'
        call    write_char      ; Try to flush output after change

        call    wait_100ms

        movlw   '\n'
        call    write_char      ; Try to flush output after change

        goto    interactive

;;; //////////

inter_cmd_suppress:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_channel
        movwf   INTER_CHANNEL

        call    read_hex_dbl
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movlw   HIGH(SUPPRESS0)
        movwf   FSR1H
        movfw   INTER_CHANNEL
        addlw   LOW(SUPPRESS0)
        movwf   FSR1L

        movfw   INTER_VALUE
        movwf   INDF1

        goto    interactive

;;; //////////

inter_cmd_discard:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_channel
        movwf   INTER_CHANNEL

        call    read_hex_dbl
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movfw   INTER_VALUE
        btfsc   STATUS, Z
        goto    discard_ok      ; A value of zero is ok

        call    convert_char
        addlw   0               ; Update zero flag
        btfss   STATUS, Z
        incf    WREG, W
        movlw   '\n'            ; Set last char in case of error
        btfsc   STATUS, Z
        goto    inter_error_just_read ; Error binary char chosen

discard_ok:
        movlw   HIGH(DISCARD_CHAR0)
        movwf   FSR1H
        movfw   INTER_CHANNEL
        addlw   LOW(DISCARD_CHAR0)
        movwf   FSR1L

        movfw   INTER_VALUE
        movwf   INDF1

        goto    interactive

;;; //////////

inter_cmd_speed:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        movlw   2
        call    read_num
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movfw   INTER_VALUE
        movlb   12
        movwf   INTER_SPEED
        movlb   0

        goto    interactive

;;; //////////

inter_cmd_schmitt:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_hex_dbl
        movwf   INTER_VALUE

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        movfw   INTER_VALUE
        call    inter_set_schmitt

        goto    interactive

;;; //////////

inter_cmd_output:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        call    inter_output_channel_output

        call    inter_output_fast

        call    inter_output_return_newline

        call    inter_output_invert

        call    inter_output_invert_out

        movlw   0
        call    inter_output_suppress
        movlw   1
        call    inter_output_suppress
        movlw   2
        call    inter_output_suppress
        movlw   3
        call    inter_output_suppress
        movlw   4
        call    inter_output_suppress
        movlw   5
        call    inter_output_suppress
        movlw   6
        call    inter_output_suppress
        movlw   7
        call    inter_output_suppress

        movlw   0
        call    inter_output_discard
        movlw   1
        call    inter_output_discard
        movlw   2
        call    inter_output_discard
        movlw   3
        call    inter_output_discard
        movlw   4
        call    inter_output_discard
        movlw   5
        call    inter_output_discard
        movlw   6
        call    inter_output_discard
        movlw   7
        call    inter_output_discard

        call    inter_output_speed

        call    inter_output_schmitt

        goto    interactive_no_ok

;;; //////////

inter_cmd_save:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        call    save_user_settings

        goto    interactive

;;; //////////

inter_cmd_load:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        call    load_user_settings

        goto    interactive

;;; //////////

inter_cmd_factory:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        call    load_factory_settings
        call    save_user_settings

        goto    interactive

;;; //////////

inter_cmd_debug:
        bcf     STATUS, Z       ; Indicate that no error has occurred

        call    read_newline

        btfsc   STATUS, Z
        goto    inter_error_just_read

        call    output_debug

        goto    interactive_no_ok

;;; //////////

inter_done:
        movlb   3
        bcf     RC1STA, CREN    ; Disable receive
        movlb   0

        call    init2

        call    wait_100ms

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; A number of functions for printing settings.
inter_output_channel_output:
        movlw   'C'
        call    write_char

        movlw   '1'
        btfss   CHN_OUT_FLAGS, CHN_OUT_BIT
        movlw   '0'
        call    write_char

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_fast:
        movlw   'F'
        call    write_char

        movfw   FAST_FLAGS
        call    nibble_to_hex
        call    write_char

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_return_newline:
        movlw   'N'
        call    write_char

        movlw   '1'
        btfss   NEWLINE_FLAGS, NEWLINE_BIT
        movlw   '0'
        call    write_char

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_invert:
        movlw   'I'
        call    write_char

        call    inter_get_invert
        call    write_hex

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_suppress:
        movwf   INTER_CHANNEL

        movlw   HIGH(SUPPRESS0)
        movwf   FSR1H
        movfw   INTER_CHANNEL
        addlw   LOW(SUPPRESS0)
        movwf   FSR1L

        movlw   'U'
        call    write_char

        movfw   INTER_CHANNEL
        addlw   '1'
        call    write_char

        movfw   INDF1
        call    write_hex

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_discard:
        movwf   INTER_CHANNEL

        movlw   HIGH(DISCARD_CHAR0)
        movwf   FSR1H
        movfw   INTER_CHANNEL
        addlw   LOW(DISCARD_CHAR0)
        movwf   FSR1L

        movlw   'D'
        call    write_char

        movfw   INTER_CHANNEL
        addlw   '1'
        call    write_char

        movfw   INDF1
        call    write_hex

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_speed:
        movlw   'B'
        call    write_char

        movlb   12
        movfw   INTER_SPEED
        movlb   0
        addlw   '0'
        call    write_char

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_schmitt:
        movlw   'H'
        call    write_char

        call    inter_get_schmitt
        call    write_hex

        movlw   '\n'
        call    write_char

        return

;;; //////////

inter_output_invert_out:
        movlw   'J'
        call    write_char

        call    inter_get_invert_out
        addlw   '0'
        call    write_char

        movlw   '\n'
        call    write_char

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Print debug information.
output_debug:
        movlw   'V'
        call    write_char

        movlw   'E'
        call    write_char

        movlw   ' '
        call    write_char

        movlw   MAJOR_VERSION
        call    write_char

        movlw   '.'
        call    write_char

        movlw   MINOR_VERSION
        call    write_char

        movlw   '\n'
        call    write_char

        movlw   'R'
        call    write_char

        movlw   'I'
        call    write_char

        movlw   ' '
        call    write_char

        movlb   3               ; Get ready to read revision id

        movlw   0x05
        movwf   PMADRL
        clrf    PMADRH

        bsf     PMCON1, CFGS    ; Configuration memory
        bsf     PMCON1, RD      ; Start read
        nop                     ; Wait
        nop
        movfw   PMDATH          ; High part

        movlb   0

        call    write_hex

        movlb   3
        movfw   PMDATL          ; Low part
        movlb   0

        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'T'
        call    write_char

        movlw   '1'
        call    write_char

        movlw   ' '
        call    write_char

        clrf    TM3L            ; TM1H:L is the negative of the time
        clrf    TM3H
        movfw   TM1L
        subwf   TM3L, f
        movfw   TM1H
        subwfb  TM3H, f         ; TM3H:L = -TM1H:L

        movfw   TM3H
        call    write_hex

        movfw   TM3L
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'T'
        call    write_char

        movlw   '2'
        call    write_char

        movlw   ' '
        call    write_char

        clrf    TM3L            ; TM2H:L is the negative of the time
        clrf    TM3H
        movfw   TM2L
        subwf   TM3L, f
        movfw   TM2H
        subwfb  TM3H, f         ; TM3H:L = -TM2H:L

        movfw   TM3H
        call    write_hex

        movfw   TM3L
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'M'
        call    write_char

        movlw   'D'
        call    write_char

        movlw   ' '
        call    write_char

        call    is_raspberry

        movlw   'S'             ; Stand alone
        btfsc   STATUS, C
        movlw   'R'             ; Raspberry Pi

        call    write_char

        movlw   '\n'
        call    write_char

        movlw   'F'
        call    write_char

        movlw   'E'
        call    write_char

        movlw   ' '
        call    write_char

        movfw   CNT_FRAME
        call    write_hex

        movlw   '\n'
        call    write_char
        movlw   'C'
        call    write_char

        movlw   'G'
        call    write_char

        movlw   ' '
        call    write_char

        movfw   CNT_CONGEST
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'L'
        call    write_char

        movlw   'O'
        call    write_char

        movlw   ' '
        call    write_char

        movlb   12
        movfw   CNT_LONG
        movlb   0
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'S'
        call    write_char

        movlw   'L'
        call    write_char

        movlw   ' '
        call    write_char

        movlb   12
        movfw   CNT_SLOW
        movlb   0
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'B'
        call    write_char

        movlw   'I'
        call    write_char

        movlw   ' '
        call    write_char

        movlb   12
        movfw   CNT_BINARY
        movlb   0
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'E'
        call    write_char

        movlw   'C'
        call    write_char

        movlw   ' '
        call    write_char

        movfw   ERR_CHANNELS
        call    write_hex

        movlw   '\n'
        call    write_char

        return

;;; This is extra debug info that is not generally needed:
        movlw   'U'
        call    write_char

        movlw   'A'
        call    write_char

        movlw   ' '
        call    write_char

        movfw   NEW_MSGH
        call    write_hex

        movfw   NEW_MSGL
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'U'
        call    write_char

        movlw   'B'
        call    write_char

        movlw   ' '
        call    write_char

        movlb   12
        movfw   NEW_MSGHB
        movlb   0
        call    write_hex

        movlb   12
        movfw   NEW_MSGLB
        movlb   0
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   'F'
        call    write_char

        movlw   'B'
        call    write_char

        movlw   ' '
        call    write_char

        movfw   BK_FREEH
        call    write_hex

        movfw   BK_FREEL
        call    write_hex

        movlw   '\n'
        call    write_char

        movlw   LOW(REF0) + 11
        movwf   FSR1L
        movlw   HIGH(REF0)
        movwf   FSR1H

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        moviw   FSR1--
        call    write_hex

        movlw   '\n'
        call    write_char

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set carry flag according to whether the board is run from a
;;; Raspberry Pi (supply voltage <= 4.096 V, typically 3.3) or stand
;;; alone (> 4.096 V, typically 5.0).
is_raspberry:
        movlb   1
        rlf     ADRESH, W       ; Carry flag set to highest A/D bit
        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set up the PIC.
init:
        movlb   1

        bsf     OSCCON, IRCF3
        bcf     OSCCON, IRCF0   ; 32 MHz

        bcf     TRISC, TRISC0   ; RC0 as output

        bcf     OPTION_REG, NOT_WPUEN ; Enable weak pull-up

        movlb   3

        clrf    ANSELA          ; All PORTA pins as digital
        clrf    ANSELC          ; All PORTC pins as digital

        bsf     TX1STA, BRGH    ; High baud rate
        bsf     BAUD1CON, BRG16 ; 16 bit baud rate

        bsf     RC1STA, SPEN    ; Enable serial port

        bsf     TX1STA, TXEN    ; Enable transmission
        bcf     TX1STA, SYNC    ; Asynchronous

        movlb   29

        movlw   0x14
        movwf   RC0PPS          ; RC0 as UART transmit

        movlb   0

        bsf     T1CON, TMR1ON

        movlw   0x82
        movwf   PR2             ; Period for timer 2, used for breaks after sending newline

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set the UART receive to the standard pin
std_uart_receive:
        movlb   28

        movlw   0x11
        movwf   RXPPS           ; RC1 is UART receive

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set the UART receive to channel 8 (for standalone version of board)
ch8_uart_receive:
        movlb   28

        movlw   0x12
        movwf   RXPPS           ; RC2 is UART receive

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set up the variables along with the baud rate. This is called both
;;; at startup and after the interactive mode.
init2:
        movlw   TIMER_HIGH
        movwf   TIMER0H + FAST0_NUM
        movwf   TIMER0H + FAST1_NUM
        movwf   TIMER0H + FAST2_NUM
        movwf   TIMER0H + FAST3_NUM
        movwf   TIMER0H + SLOW0_NUM
        movwf   TIMER0H + SLOW1_NUM
        movwf   TIMER0H + SLOW2_NUM
        movwf   TIMER0H + SLOW3_NUM

        clrf    TIMER0L + FAST0_NUM
        clrf    TIMER0L + FAST1_NUM
        clrf    TIMER0L + FAST2_NUM
        clrf    TIMER0L + FAST3_NUM
        clrf    TIMER0L + SLOW0_NUM
        clrf    TIMER0L + SLOW1_NUM
        clrf    TIMER0L + SLOW2_NUM
        clrf    TIMER0L + SLOW3_NUM

        clrf    CH_RDY
        movlw   0xFF
        movwf   WAITING
        movwf   PHASE           ; Mark every channel as having a framing error,
                                ; this will make sure the first sentence is right
        clrf    DONE

        clrf    CH_BUSY

        clrf    READF0
        clrf    READF1
        clrf    READF2
        clrf    READF3
        clrf    READS0
        clrf    READS1
        clrf    READS2
        clrf    READS3

        movlw   0x80
        movwf   SEND_BK

        clrf    Q_START
        clrf    Q_END

        movlw   BANK_MASKL      ; Mark free banks as such
        movwf   BK_FREEL
        movlw   BANK_MASKH
        movwf   BK_FREEH

        movlw   6
        movwf   FSR0H           ; Reset FSR0H to point to bank 12

        movlb   12

        movlw   0xFF
        movwf   NEW_MSGL
        movwf   NEW_MSGH
        movwf   NEW_MSGLB
        movwf   NEW_MSGHB
        movlw   DISCARD_BANK
        movwf   STUCK_BANK
        clrf    STUCK_CNT1
        clrf    STUCK_CNT2

        clrf    CNT_LONG
        clrf    CNT_BINARY
        clrf    CNT_SLOW

        movlw   0xFF
        movwf   STUCK_CHANNEL   ; no channel stuck now

        movlb   0

        clrf    CNT_CONGEST
        clrf    CNT_FRAME
        clrf    ERR_CHANNELS

        clrf    TM1H
        clrf    TM1L

        movlw   0xFF
        movwf   TM2H
        movwf   TM2L

        bcf     TM_VALID_FLAGS, TM_VALID_BIT
        bcf     SD_CH_FLAGS, SD_CH_BIT ; No char to send

        movlw   0xFF
        movwf   BANK0 + 0
        movwf   BANK0 + 1
        movwf   BANK0 + 2
        movwf   BANK0 + 3
        movwf   BANK0 + 4
        movwf   BANK0 + 5
        movwf   BANK0 + 6
        movwf   BANK0 + 7

        movlb   12

        movwf   REF0 + 1
        movwf   REF0 + 2
        movwf   REF0 + 3
        movwf   REF0 + 4
        movwf   REF0 + 5
        movwf   REF0 + 6
        movwf   REF0 + 7
        movwf   REF0 + 8
        movwf   REF0 + 9
        movwf   REF0 + 10
        movwf   REF0 + 11

        movlb   3

        btfss   TX1STA, TRMT    ; Wait for trasmission to finish
        goto    $-1             ; before setting speed

        movlb   12

        movfw   INTER_SPEED

        movlb   0

        btfsc   STATUS, Z
        call    speed_4800

        decf    WREG, W
        btfsc   STATUS, Z
        call    speed_38400

        decf    WREG, W
        btfsc   STATUS, Z
        call    speed_115200

        movlb   2

        movlw   0x82
        movwf   FVRCON          ; Enable fixed voltage reference, set to 2.048 V for A/D conversion

        btfss   FVRCON, FVRRDY
        goto    $-1             ; Wait for voltage reference to be ready

        movlb   1

        movlw   0x7D
        movwf   ADCON0          ; A/D conversion on and set to fixed voltage reference

        movlw   0x60
        movwf   ADCON1          ; A/D conversion result left justified, clock Fosc/64, Vss and Vdd refs

        movlb   0

        return

;;; /////////////////////////////////////////////////////////////////////////////

init3:
        movlb   1

        bsf     ADCON0, GO_NOT_DONE
        btfsc   ADCON0, GO_NOT_DONE
        goto    $-1             ; Wait for A/D conversion to be done

        movlw   0x00
        movwf   ADCON0          ; Disable A/D conversion

        movlb   2

        movlw   0x00
        movwf   FVRCON          ; Disable fixed voltage reference

        movlb   0

        call    is_raspberry

        btfsc   STATUS, C
        call    std_uart_receive ; For Raspberry Pi
        btfss   STATUS, C
        call    ch8_uart_receive ; For stand alone

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Wait a little
wait_100ms:
        movlw   0x04            ; Take a break of about 100 ms (clock is 32 MHz)
        movwf   CHAR0
        clrf    CHAR0 + 1
        clrf    CHAR0 + 2
wait_100ms_lp:
        decfsz  CHAR0 + 2, f
        goto    wait_100ms_lp
        decfsz  CHAR0 + 1, f
        goto    wait_100ms_lp
        decfsz  CHAR0, f
        goto    wait_100ms_lp

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set serial speed to 4,800 baud.
speed_4800:
        movlb   3
        movlw   6
        movwf   SP1BRGH
        movlw   130
        movwf   SP1BRGL         ; 4,799 baud
        movlb   0

        movlw   0x2B
        movwf   T2CON           ; off, postscaler 6, prescaler 64 for 6240 us or 30 bits

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set serial speed to 38,400 baud.
speed_38400:
        movlb   3
        clrf    SP1BRGH
        movlw   207
        movwf   SP1BRGL         ; 38,462 baud
        movlb   0

        movlw   0x12
        movwf   T2CON           ; off, postscaler 3, prescaler 16 for 780 us or 30 bits

        return

;;; /////////////////////////////////////////////////////////////////////////////

;;; Set serial speed to 115,200 baud.
speed_115200:
        movlb   3
        clrf    SP1BRGH
        movlw   68
        movwf   SP1BRGL         ; 115,942 baud
        movlb   0

        movlw   0x02
        movwf   T2CON           ; off, postscaler 1, prescaler 16 for 260 us or 30 bits

        return


;;; /////////////////////////////////////////////////////////////////////////////

;;; Convert a char that was read according to this:
;;;
;;;   - '\r' and '\n' becomes 0x00
;;;
;;;   - printable characters ('\t' and 0x20 to 0x7E) are unchanged
;;;
;;;   - non-printable characters become 0xFF
convert_char:
        brw
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0x09             ; '\t', keep it is normal character
        retlw  0x00             ; '\n'
        retlw  0xFF
        retlw  0xFF
        retlw  0x00             ; '\r'
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0x20
        retlw  0x21
        retlw  0x22
        retlw  0x23
        retlw  0x24
        retlw  0x25
        retlw  0x26
        retlw  0x27
        retlw  0x28
        retlw  0x29
        retlw  0x2A
        retlw  0x2B
        retlw  0x2C
        retlw  0x2D
        retlw  0x2E
        retlw  0x2F
        retlw  0x30
        retlw  0x31
        retlw  0x32
        retlw  0x33
        retlw  0x34
        retlw  0x35
        retlw  0x36
        retlw  0x37
        retlw  0x38
        retlw  0x39
        retlw  0x3A
        retlw  0x3B
        retlw  0x3C
        retlw  0x3D
        retlw  0x3E
        retlw  0x3F
        retlw  0x40
        retlw  0x41
        retlw  0x42
        retlw  0x43
        retlw  0x44
        retlw  0x45
        retlw  0x46
        retlw  0x47
        retlw  0x48
        retlw  0x49
        retlw  0x4A
        retlw  0x4B
        retlw  0x4C
        retlw  0x4D
        retlw  0x4E
        retlw  0x4F
        retlw  0x50
        retlw  0x51
        retlw  0x52
        retlw  0x53
        retlw  0x54
        retlw  0x55
        retlw  0x56
        retlw  0x57
        retlw  0x58
        retlw  0x59
        retlw  0x5A
        retlw  0x5B
        retlw  0x5C
        retlw  0x5D
        retlw  0x5E
        retlw  0x5F
        retlw  0x60
        retlw  0x61
        retlw  0x62
        retlw  0x63
        retlw  0x64
        retlw  0x65
        retlw  0x66
        retlw  0x67
        retlw  0x68
        retlw  0x69
        retlw  0x6A
        retlw  0x6B
        retlw  0x6C
        retlw  0x6D
        retlw  0x6E
        retlw  0x6F
        retlw  0x70
        retlw  0x71
        retlw  0x72
        retlw  0x73
        retlw  0x74
        retlw  0x75
        retlw  0x76
        retlw  0x77
        retlw  0x78
        retlw  0x79
        retlw  0x7A
        retlw  0x7B
        retlw  0x7C
        retlw  0x7D
        retlw  0x7E
        retlw  0xFF             ; 0x7F and up is no good
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF
        retlw  0xFF

;;; /////////////////////////////////////////////////////////////////////////////
;;; Settings sections starts here.
;;; /////////////////////////////////////////////////////////////////////////////

facset  code    0x1F60

factory_settings:
        settings

;;; //////////

userset code    0x1F80          ; 0x1F80-0x1FFF is high-endurance
                                ; flash. Use it for user settings.
user_settings:
        settings

;;; /////////////////////////////////////////////////////////////////////////////

        end
