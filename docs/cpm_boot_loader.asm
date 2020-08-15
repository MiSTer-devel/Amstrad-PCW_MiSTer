;
; PCW for MiSTer CPM Boot Disassembly
;
; Copyright (c) 2020 Stephen Eddy
;
; All rights reserved
;
; Redistribution and use in source and synthezised forms, with or without
; modification, are permitted provided that the following conditions are met:
;
; * Redistributions of source code must retain the above copyright notice,
;   this list of conditions and the following disclaimer.
;
; * Redistributions in synthesized form must reproduce the above copyright
;   notice, this list of conditions and the following disclaimer in the
;   documentation and/or other materials provided with the distribution.
;
; * Neither the name of the author nor the names of other contributors may
;   be used to endorse or promote products derived from this software without
;   specific prior written agreement from the author.
;
; * License is granted for non-commercial use only.  A fee may not be charged
;   for redistributions as source code or in synthesized/hardware form without 
;   specific prior written agreement from the author.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
; THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
; PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGE.
;
; Keyboard and Joystick mapper for Amstrad PCW keyboard matrix

0000f000 00          nop
0000f001 00          nop
0000f002 28 09       jr z,0xf00d
0000f004 02          ld (bc),a
0000f005 01 03 02    ld bc,0x0203
0000f008 2a 52 00    ld hl,(0x0052)
0000f00b 00          nop
0000f00c 00          nop
0000f00d 00          nop
0000f00e 00          nop
0000f00f 04          inc b

; Main system entry point
0000f010 31 f0 ff    ld sp,0xfff0       ; Main Bootloader entry point
0000f013 3e ff       ld a,0xff
0000f015 32 d0 f8    ld (0xf8d0),a      ; Starting track number (0xff = undefined)
0000f018 cd b4 f0    call 0xf0b4        ; Clear screen to white
0000f01b 21 00 00    ld hl,0x0000       ; Sector offset, starting from Track 1
0000f01e e5          push hl
0000f01f 11 00 d0    ld de,0xd000       ; Address to load sectors into
0000f022 06 04       ld b,0x04          ; Number of sectors to load
0000f024 cd 8e f0    call 0xf08e        ; Load DIR table from Track 1, S 1-4 into 0xD000
0000f027 10 fb       djnz 0xf024        ; Loop for all 4 sectors
0000f029 cd 57 f0    call 0xf057        ; Find the address of the EMS CPM boot file
0000f02c c2 23 f1    jp nz,0xf123       ; Error out if not found on disk
0000f02f d1          pop de             ; DE = 0x0000 (was in HL above) - Load address
0000f030 06 10       ld b,0x10          ; Number of extent blocks in DIR entry to read
0000f032 7e          ld a,(hl)          ; Get CPM block number for EMS file
0000f033 b7          or a               ; Compare with self
0000f034 28 1d       jr z,0xf053        ; Block zero, indicates finished
0000f036 e5          push hl            ; Save Block address for later
0000f037 6f          ld l,a             ; Block number to read stored into L
0000f038 26 00       ld h,0x00          ; Extended to HL
0000f03a 29          add hl,hl          ; Convet block number to sector number (1024 byte blocks)
0000f03b cd 8e f0    call 0xf08e        ; Read first sector of 512 bytes of block
0000f03e cd 8e f0    call 0xf08e        ; Read second sector of 512 bytes of block
0000f041 cd d8 f0    call 0xf0d8        ; Draw screen lines to indicate loading progress
0000f044 e1          pop hl             ; Recover CPM block pointer above
0000f045 23          inc hl             ; Move to next byte
0000f046 10 ea       djnz 0xf032        ; Continue for all 16 blocks to read next (decreement B)
0000f048 21 b3 f0    ld hl,0xf0b3       ; Pointer to extent number in DB area (starts at 0)
0000f04b 34          inc (hl)           ; Increment to next extent
0000f04c d5          push de            ; Save DE address
0000f04d cd 57 f0    call 0xf057        ; Find next DIR entry for current extent number
0000f050 d1          pop de             ; Restore DE address for next sector
0000f051 28 dd       jr z,0xf030        ; More extents found, so continue loading file
0000f053 cd 42 f1    call 0xf142        ; Blank screen and motors off
0000f056 c7          rst 0x00           ; Reset system to continue booting

; Search for CPM EMS file extension through all DIR entries in memory at 0xD000
; On return the bytes from 0xf0a8-0xf0b4 will match the full filename of EMS file
; HL will point to the array of block pointers found
; B will be the number of DIR blocks still to search
0000f057 21 00 d0    ld hl,0xd000       ; Address in memory of DIR table
0000f05a 06 40       ld b,0x40          ; Number of directory entries to search through
0000f05c 7e          ld a,(hl)          ; Load byte from address 0xd000
0000f05d e6 f0       and 0xf0           ; Mask off top 4 bits
0000f05f 20 22       jr nz,0xf083       ; If set, ignore.  This isn't a FILE entry
0000f061 c5          push bc
0000f062 e5          push hl
0000f063 11 0d 00    ld de,0x000d       
0000f066 19          add hl,de          ; Add 0x0d to 0xd000
0000f067 11 b4 f0    ld de,0xf0b4       ; End of DB bytes below. End of extension EMS
0000f06a 01 ff 0c    ld bc,0x0cff       ; 12 characters, ff compare mask
0000f06d 1b          dec de             ; Ptr to '/0' in 'EMS/0' on first entry
0000f06e 1a          ld a,(de)          ; Get character pointed at above from DB area below
0000f06f 2b          dec hl             ; Dec HL to offset 0xc in DIR entry. Extend L (Xl)
0000f070 ae          xor (hl)           ; XOR with character
0000f071 a1          and c              ; Mask (7/ff)
0000f072 28 09       jr z,0xf07d        ; Zero indicates match found for char, so move to next
                                        ; Match found for char, so store it
0000f074 1a          ld a,(de)          ; Load last char again from DB table
0000f075 fe 3f       cp 0x3f            ; If it equals 0x3f, we need to store the name
0000f077 20 08       jr nz,0xf081       ; No match found if it is 0x3f, move to next entry
                                        ; We have found the EMS entry char
0000f079 7e          ld a,(hl)          ; Get last char in file name
0000f07a a1          and c              ; Apply ASCII mask
0000f07b 12          ld (de),a          ; Store in bytes before extension
0000f07c af          xor a              ; Clear A
                                        ; Continue to next charactersd
0000f07d 0e 7f       ld c,0x7f          ; Change mask to 0x7f for ascii characters
0000f07f 10 ec       djnz 0xf06d        ; and loop to next character above
                                        ; Else filename is stored
0000f081 e1          pop hl             ; Moved to next DIR entry
0000f082 c1          pop bc
0000f083 11 10 00    ld de,0x0010       ; Add 16 to DIR address pointer
0000f086 19          add hl,de
0000f087 c8          ret z              ; Return if A is zero.  Name found and stored
0000f088 19          add hl,de          ; Else, Add another 16 (entries 32 bytes)
0000f089 10 d1       djnz 0xf05c        ; Loop if not past end of DIR to next entry
0000f08b f6 ff       or 0xff            ; Finish all dir entries
0000f08d c9          ret

; Calculate sector and track that a given sector number resides on (in HL on entry)
; Then load it.  Parameters are:
; HL = Sector Number (From track 1, not 0)
; DE = Address in memory to load sector into
; DE and HL will be advanced to the next sector on return
0000f08e c5          push bc
0000f08f e5          push hl
0000f090 01 f7 ff    ld bc,0xfff7
0000f093 af          xor a
0000f094 3c          inc a          ; A now equals 1
0000f095 09          add hl,bc      ; Subract 9 from HL.  Check if block spans two tracks
0000f096 38 fc       jr c,0xf094    ; Calculate track number sector resides on (now in A), Sector in HL
0000f098 47          ld b,a         ; Store track number in B
0000f099 7d          ld a,l         ; Store sector number in A
0000f09a c6 0a       add a,0x0a     ; Add 10 to sector number to exclude first track (0)
0000f09c 4f          ld c,a         ; Adjusted track number stored into C (Now 01, not 00)
0000f09d eb          ex de,hl       ; Place load address DE into HL
0000f09e cd f3 f0    call 0xf0f3    ; Load the sector
0000f0a1 eb          ex de,hl       ; Restore saved load address from HL
0000f0a2 e1          pop hl
0000f0a3 c1          pop bc
0000f0a4 14          inc d          ; DE = Memory Pointer for sector loads
0000f0a5 14          inc d          ; Advance memory pointer by 0x200
0000f0a6 23          inc hl         ; Incremement sector number in HL
0000f0a7 c9          ret            

; Storage area for name of CPM EMS Boot file as used above
0000f0a8 3f          DB 0x3f        ; Filename storage
0000f0a9 3f          DB 0x3f
0000f0aa 3f          DB 0x3f
0000f0ab 3f          DB 0x3f
0000f0ac 3f          DB 0x3f
0000f0ad 3f          DB 0x3f
0000f0ae 3f          DB 0x3f
0000f0af 3f          DB 0x3f
0000f0b0 45          DB 0x45        ; E
0000f0b1 4d          DB 0x4d        ; M
0000f0b2 53          DB 0x53        ; S
0000f0b3 00          DB 0x00        ; Extent number

; Function to fill the screen with white.  Loads Roller Ram at f400-f5ff
; and one line of 90 characters at f600-f8d8
0000f0b4 01 ff 5a    ld bc,0x5aff
0000f0b7 21 00 f6    ld hl,0xf600
0000f0ba 11 07 00    ld de,0x0007
0000f0bd 71          ld (hl),c
0000f0be 23          inc hl
0000f0bf 72          ld (hl),d
0000f0c0 19          add hl,de
0000f0c1 10 fa       djnz 0xf0bd        ; Fill line buffer at f600
0000f0c3 21 00 f6    ld hl,0xf600
0000f0c6 2b          dec hl
0000f0c7 36 7b       ld (hl),0x7b
0000f0c9 2b          dec hl
0000f0ca 72          ld (hl),d
0000f0cb 10 f9       djnz 0xf0c6        ; Fill roller ram at f400
0000f0cd 7c          ld a,h
0000f0ce 0f          rrca
0000f0cf d3 f5       out (0xf5),a       ; Output roller ram offset 0x7a for 0xf400 pointer
0000f0d1 af          xor a
0000f0d2 d3 f6       out (0xf6),a       ; Roller offset 0x0000
0000f0d4 e5          push hl
0000f0d5 dd e1       pop ix             ; IX = 0xf400 = Start of Roller Ram, add 8 each time
0000f0d7 c9          ret

; Draw lines 3 and 4 black by change roller ram line from 0x7b00 to 0x7b01
0000f0d8 c5          push bc
0000f0d9 dd 34 04    inc (ix+4)         ; Line addres + 4, Line address + 6
0000f0dc dd 34 06    inc (ix+6)
0000f0df 01 08 00    ld bc,0x0008       ; Move down 8 lines
0000f0e2 dd 09       add ix,bc
0000f0e4 c1          pop bc
0000f0e5 c9          ret

; Wait for Vblank signal to set and then clear.
0000f0e6 db f8       in a,(0xf8)
0000f0e8 e6 40       and 0x40
0000f0ea 20 fa       jr nz,0xf0e6
0000f0ec db f8       in a,(0xf8)
0000f0ee e6 40       and 0x40
0000f0f0 28 fa       jr z,0xf0ec
0000f0f2 c9          ret

; Load the specified sector from disk.  Called from main load sector code at 0xf08e
; On Entry:
;   HL = Address in memory to load sector
;   B = Track number to load
;   C = Sector number to load
0000f0f3 1e 0a       ld e,0x0a          ; Will attempt this process 10x on failure
0000f0f5 c5          push bc            ; Save track / sector
0000f0f6 0e 40       ld c,0x40          ; Reverse screen code
0000f0f8 cd 8a f1    call 0xf18a        ; Locate track number on disk (reverse screen)
0000f0fb 38 15       jr c,0xf112
0000f0fd c1          pop bc             ; Restore track / sector
0000f0fe cd 51 f1    call 0xf151        ; Read the sector into memory
0000f101 c4 51 f1    call nz,0xf151     ; Retry once on failure
0000f104 c8          ret z              ; Return on success

; If we get here, the sector load has failed twice
0000f105 c5          push bc            ; Save Track / Sector
0000f106 3e 27       ld a,0x27          ; Last track on disk (39)
0000f108 b8          cp b               ; Check if we are already on it
0000f109 20 01       jr nz,0xf10c       ; If we aren't skip next instruction
0000f10b 0f          rrca               ; We are, so divide by 2 to get track 19
0000f10c 47          ld b,a             ; Save new track number
0000f10d 0e 00       ld c,0x00          ; Sector reset to 0
0000f10f cd 8a f1    call 0xf18a        ; Locate track number on disk (un-reverse screen)
0000f112 cd df f1    call 0xf1df        ; Send Recalibrate command to DC (back to track 0)
; Following bytes will be sent to the disk controller
; The called routine 0xf1df will return to 0xf118 below
0000f115 02          DB 0x02            ; Length of Recalibrate command (Go track 0)
0000f116 07          DB 0x07            ; Recalibrate command
0000f117 00          DB 0x00            ; Drive select 0
; Return from call at 0xf112 after Recalibrate command
0000f118 cd a7 f1    call 0xf1a7        ; Get response from DC
0000f11b af          xor a
0000f11c 32 d0 f8    ld (0xf8d0),a      ; Store track 0 after recalibrate
0000f11f c1          pop bc             ; Restore track / sector from before failure
0000f120 1d          dec e              ; Decrement retry time
0000f121 20 d2       jr nz,0xf0f5       ; Retry sector loading again
0000f123 cd 42 f1    call 0xf142        ; Alternate screen reverse and delay
0000f126 16 04       ld d,0x04          ; Sound counter
0000f128 01 0b 14    ld bc,0x140b
0000f12b cd 4c f1    call 0xf14c        ; Bleeper on then and delay
0000f12e 01 0c 50    ld bc,0x500c       
0000f131 cd 4c f1    call 0xf14c        ; Bleeper off then longer delay
0000f134 15          dec d      
0000f135 20 f1       jr nz,0xf128       ; Repeat sound 4 times
0000f137 21 f5 ff    ld hl,0xfff5       ; Should this be 0x3ff5 for keyboard row with space (bank)?
0000f13a 77          ld (hl),a          ; A should still be 0x00
0000f13b cb 7e       bit 7,(hl)         ; Test for spacebar pressed
0000f13d 28 fc       jr z,0xf13b        ; And loop forever until it is
0000f13f 3c          inc a              ; Increment A to 0x01
0000f140 d3 f8       out (0xf8),a       ; And cause a system reset.  Will not return

;Final call also comes here after EMS is loaded.  Will Reverse video, turn off motor then delay
0000f142 cd e6 f0    call 0xf0e6        ; Wait for vBlank
0000f145 3e 80       ld a,0x80
0000f147 d3 f7       out (0xf7),a       ; Reverse video output
0000f149 01 0a 01    ld bc,0x010a       
0000f14c 79          ld a,c
0000f14d d3 f8       out (0xf8),a       ; Turn disk motors off
0000f14f 18 64       jr 0xf1b5          ; Delay then return

; Main sector reading routine (called from 0xf0fe)
; B = Track number, C = Sector Number, HL = Location to save track in memory
0000f151 c5          push bc
0000f152 e5          push hl
0000f153 78          ld a,b
0000f154 32 65 f1    ld (0xf165),a      ; Track number
0000f157 79          ld a,c
0000f158 32 67 f1    ld (0xf167),a      ; Sector number
0000f15b 3e 06       ld a,0x06
0000f15d d3 f8       out (0xf8),a       ; Clear TC flag to Disk Controller
0000f15f cd df f1    call 0xf1df        ; Send command.  B = 0 on return
; Following bytes will be sent to the disk controller
; The called routine 0xf1df will return to 0xf16c below
0000f162 09          DB 0x09            ; Length of read command below
0000f163 66          DB 0x66            ; Command (Read track)
0000f164 00          DB 0x00            ; Drive select
0000f165 01          DB 0x01            ; Track number
0000f166 00          DB 0x00            ; Head number
0000f167 01          DB 0x01            ; Sector number
0000f168 02          DB 0x02            ; Sector size code (2 = 512 bytes)
0000f169 09          DB 0x09            ; EOT - Last sector on track
0000f16a 2a          DB 0x2a            ; GPL - Gap Length
0000f16b ff          DB 0xff            ; DTL - Data transfer length (SSC will override)
; return address from call at 0xf15f.  B will be set to 0x00 from above call
0000f16c 16 02       ld d,0x02          ; 2 x 256 for sector length to read. B = 0 from above
0000f16e db 00       in a,(0x00)
0000f170 87          add a,a
0000f171 30 fb       jr nc,0xf16e       ; Loop if status[7] not set (RQM) - Not ready
0000f173 87          add a,a
0000f174 f2 7e f1    jp p,0xf17e        ; If no longer in execute mode bit 5 (EXM), jump to end
0000f177 ed a2       ini                ; Receive byte (port in C=0x01), Write to (HL), INC HL, DEC B
0000f179 20 f3       jr nz,0xf16e       ; Loop for 256 bytes (in B)
0000f17b 15          dec d              ; One sector = 512 bytes, so go again.  B = 0 again
0000f17c 20 f0       jr nz,0xf16e       ; More bytes remaining (or sectors)
; All 512 bytes received, so set TC to end command and read response
0000f17e 3e 05       ld a,0x05
0000f180 d3 f8       out (0xf8),a       ; Set TC flag for end of transfer
0000f182 cd cb f1    call 0xf1cb        ; Read response bytes, return ST0
0000f185 e6 cb       and 0xcb           ; Mask IC code(7:6), NR, and Disk Select from ST0 and return it
0000f187 e1          pop hl             ; Recover saved memory pointer
0000f188 c1          pop bc             ; and saved track / sector number
0000f189 c9          ret

; Move to specified track if required
; On entry:
;   B = Track to move to
;   C = Screen reverse setting
0000f18a 3a d0 f8    ld a,(0xf8d0)      ; Get stored track number (pcn)
0000f18d b8          cp b               ; Compare to target track number (ncn)
0000f18e c8          ret z              ; Return if it matches
0000f18f 78          ld a,b
0000f190 32 d0 f8    ld (0xf8d0),a      ; Store ncn track number in pcn register
0000f193 cd e6 f0    call 0xf0e6        ; Wait for vBlank
0000f196 79          ld a,c
0000f197 d3 f7       out (0xf7),a       ; Set or unset screen reverse
0000f199 78          ld a,b
0000f19a 32 a6 f1    ld (0xf1a6),a      ; Save Track number for below
0000f19d cd df f1    call 0xf1df        ; Send the bytes below to the DC
; Following bytes will be sent the SPECIFY and SEEK commands to the disk controller
; The called routine 0xf1df will return to 0xf1a4 below
0000f1a0 06          DB 0x06            ; Length of command
0000f1a1 03          DB 0x03            ; Specify command (set head load, unload time and step rate)
0000f1a2 af          DB 0xaf            ; SRT = 10ms, HUT = 240ms
0000f1a3 03          DB 0x03            ; HLT = 2ms, Non-DMA mode = true

0000f1a4 0f          DB 0x0f            ; Seek command
0000f1a5 00          DB 0x00            ; Disk select
0000f1a6 01          DB 0x??            ; Track number to seek for
; Return address from call at 0xf19d above
0000f1a7 e5          push hl            ; Save load address for next sector
0000f1a8 cd c1 f1    call 0xf1c1        ; Check for an interrupt and if there is one send sense int command
0000f1a9 c1          pop bc             ; Return HL into BC
0000f1aa f1          pop af             ; Return flags.  Pushed where?
0000f1ab 30 fb       jr nc,0xf1a8       ; Wait for no interrupt and sense bytes read
0000f1ad 17          rla                
0000f1ae 38 f8       jr c,0xf1a8        ; Check no bytes waiting
0000f1b0 e1          pop hl
0000f1b1 17          rla
0000f1b2 d8          ret c              ; Optional delay below for head to settle

; Delay loop - 16.32ms
0000f1b3 06 03       ld b,0x03          ; +7 cycles
0000f1b5 3e b3       ld a,0xb3          ; +7 cycles
0000f1b7 e3          ex (sp),hl         ; 19 cycles
0000f1b8 e3          ex (sp),hl         ; 38 cycles
0000f1b9 e3          ex (sp),hl         ; 57 cycles
0000f1ba e3          ex (sp),hl         ; 76 cycles
0000f1bb 3d          dec a              ; +4 cycles
0000f1bc 20 f9       jr nz,0xf1b7       ; +12 cycles. Inner loop = 91 x 179 = 16,289 cycles
0000f1be 10 f5       djnz 0xf1b5        ; +13 cycles.  Outter Loop = 16,289+7+13 = 16,309 x 4 = 65,236
0000f1c0 c9          ret                ; +10 cycles.  Total = 65,236 + 7 + 10 = 65,253.  @4mhz = 16.32ms

; Check for disk controller interrupt at the end of execute mode and start of transfer
0000f1c1 db f8       in a,(0xf8)
0000f1c3 e6 20       and 0x20           ; Check for Disk Controller interrupt signal (end of transfer)
0000f1c5 c8          ret z              ; No flag, so return
0000f1c6 cd e6 f1    call 0xf1e6        ; Interrupt, so send Sense Interrupt command
0000f1c9 01          DB 0x01            ; Command length 1 
0000f1ca 08          DB 0x08            ; Sense interrupt command

; Read the Response Code bytes at the end of a transfer or above sense interrupt
0000f1cb 21 d1 f8    ld hl,0xf8d1
0000f1ce db 00       in a,(0x00)
0000f1d0 87          add a,a
0000f1d1 30 fb       jr nc,0xf1ce       ; Wait for status[7] RQM signal
0000f1d3 3a d1 f8    ld a,(0xf8d1)      ; Get ST0 returned from last transfer
0000f1d6 f0          ret p              ; If DIO clear, no more bytes ao return.  A set to last response
0000f1d7 ed a2       ini                ; Else receive byte (port in C=0x01), Write to (HL), INC HL, DEC B
0000f1d9 e3          ex (sp),hl         ; Delay
0000f1da e3          ex (sp),hl
0000f1db e3          ex (sp),hl
0000f1dc e3          ex (sp),hl
0000f1dd 18 ef       jr 0xf1ce          ; Repeat for next byte

; Send code pointed to by stack pointer to Disk Controller.
; Will first check for any pending interrupts
0000f1df e5          push hl            ; Save Memory pointer where results will be stored
0000f1e0 cd c1 f1    call 0xf1c1        ; Check for interrupt and send sense int command
0000f1e3 38 fb       jr c,0xf1e0        ; If interrupt pending, wait for it to clear
0000f1e5 e1          pop hl             ; Restore memory address pointer

; Continue sending command bytes now the interrupt has cleared.  Bytes to send
; Follow the caller address.
; Will progress stack pointer beyond the data byes.  Will also check for response
0000f1e6 e3          ex (sp),hl         ; Get stack pointer return address (command bytes)
0000f1e7 46          ld b,(hl)          ; Get length of disk command into B from stack
0000f1e8 23          inc hl             ; Move HL to next byte
0000f1e9 e3          ex (sp),hl         ; And save back to stack
0000f1ea 0e 01       ld c,0x01          ; Load C with 0x01.  FDC data register
0000f1ec e3          ex (sp),hl         ; HL points to next byte on stack again (command first)
0000f1ed db 00       in a,(0x00)        ; Get status from disk controller
0000f1ef 87          add a,a            ; Double to set C flag if bit7 set
0000f1f0 30 fb       jr nc,0xf1ed       ; Check for bit7 set, RQM.  NC = Controller not ready
0000f1f2 fa f8 f1    jp m,0xf1f8        ; Check for bit6 set (m if true). DIO. M indicates response bytes waiting
0000f1f5 7e          ld a,(hl)          ; Neither set, so we are good to send
0000f1f6 ed 79       out (c),a          ; Send command byte pointed to by HL
0000f1f8 23          inc hl             ; Progress to next byte. If DIO set byte is not sent to DC
0000f1f9 e3          ex (sp),hl         ; Store updated SP to stack.  Doesn't change SP
0000f1fa e3          ex (sp),hl         ; Additional delays
0000f1fb e3          ex (sp),hl         ;
0000f1fc 10 ee       djnz 0xf1ec        ; Decrememnt B and loop until all bytes sent
0000f1fe c9          ret                ; Return to caller.  A = last byte sent, B = 0

0000f1ff cf          rst 0x08           ; Reset to address 0x0008

.org 0xf8d0
0000f8d0 ??         DB 0x??         ; Current track number (PCN)

; Below records are the 7 bytes returned from a command - description below is for a Sector Read command
0000f8d1 ??         DB 0x??         ; Response 0 - ST0
0000f8d2 ??         DB 0x??         ; Response 1 - ST1
0000f8d3 ??         DB 0x??         ; Response 1 - ST2
0000f8d4 ??         DB 0x??         ; Response 1 - Cylinder (C)
0000f8d5 ??         DB 0x??         ; Response 1 - Head (H)
0000f8d6 ??         DB 0x??         ; Response 1 - Sector (R)
0000f8d7 ??         DB 0x??         ; Response 1 - Sector Size (N)

