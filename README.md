# Amstrad PCW MiSTer Core
<img src="./docs/images/gnome.jpg" alt="Gnome Ranger" width="400" height="300">

Gnome Ranger running on the Amstrad PCW core by [steddyman](https://twitter.com/steddyman)
## Overview
The Amstrad PCW was a line of computers released by Amstrad UK in the mid 1980's to provide a cheaper alternative to owning an IBM PC for word processing and simple office type applications.  It was supplied with a detachable keyboard, printer, a disk drive and a high resolution monitor.  This monitor allowed the PCW to display sharp monochrome graphics at a resolution of 720 x 256.

Despite the intended market being business applications and home office users, the computer sold over 8 million units and quite a number of games were also developed for the machine as well as  third party addons such as sound cards, joystick adapters and mice.  The Amstrad PCW was also sold as the Schneider Joyce in Europe.

The PCW came with 256k or 512k and was expandable upto 2MB of memory.  The high resolution display and large amount of memory for the time, makes the PCW the perfect CP/M machine and an ideal system for playing early text and graphic adventures.

## Features
* Amstrad PCW 8256 with Z80 CPU operating at 4Mhz and 256k memory
* Turbo support to 8Mhz (2x), 16 Mhz(4x) or 32 Mhz
* 3" Disk Drives with read and write support
* Compatible with standard emulator DSK format
* PAL (720x256) or NTSC (720x200) resolution support
* Full PC Keyboard mapping
* Joystick support for the following types of joystick:
  * Kempston Joystick
  * Spectravideo Joystick
  * Cascade Joystick
  * DKTronics Joystick
* Mouse support for the following types of mice:
  * AMX Mouse
  * Kempston Mouse
  * Keymouse
* Regular PCW Beeper sound and DKTronics Sound Generator (AY-3-8912) support
* No additional SDRAM or other expansions needed
 

## Using the core

Unlike many home comptuters of the mid 80's, the Amstrad PCW always included a disk drive and monitor and all PCW software is supplied in disk format only.  The emulator supports the standard DSK format images used by the two main PCW emulators, Joyce by John Elliot and CPM/Box by Habisoft.

The PCW also doesn't have a BOOT ROM but instead streams a small boot routine from the keyboard controller at power-up time.  The PCW core recreates this boot sequence and does not require any external ROM files to boot.  Just place your DSK images in the **/Games/Amstrad PCW/** directory in the root of the SD card.

Games are usually supplied on two disks, normally the first disk is a CP/M boot disk.  Insert this boot disk into the A: drive using the Menu, then select **RESET** from the OSD menu to load the disk. Once the boot disk has finished booting, you will be prompted by the PCW change the disk to the second disk and press return.

![](./docs/images/change_disk.jpg)

The main operating system the PCW used was called CP/M which was developed by Digital Research and was one of the worlds first cross platform operating systems.   More infomation about CP/M and some common commands available are documented on [The CP/M Wiki article](https://en.wikipedia.org/wiki/CP/M)

## PCW Models

The PCW core emulates two different models of the PCW:
* **PCW8256/8512** - Drive A 3" 180k (CF2), Drive B 3" 720k (CF2DD)
* **PCW9256/9512+** - Both drives 3.5" 720k (MF2DD)

These two different models of the PCW require different boot disks.  This is because each model has a different bootrom, which looks for different checksums on the boot sectors.

## Current issue
* SymbOS does not work correctly
* Mouse movements can be eratic
* PSI-5 Trading Company - Requires SCAN_EQUAL disk command which is not implemented

# Changes from previous release
* Support for PCW9512+ Mode and 3.5" drives
* Support for two disk drives
* SDRAM support to provide upto 2MB memory
* Fixed issues with disk corruption and boot errors
* Fixed corruption issue with RAMtest
* Daisywheel port emulation to allow PCW915+ CPM to boot
* CPC paging mode. Fixes issues with various games (e.g. Abadia, Head over Heels)

## Upcoming features
The following features are coming shortly
* Support for SymbOS
* Mouse movement improvements

## Game Notes
* Blagger - Will move left unless Joystick is set to Cascade

## Thanks
Special thanks to the following people:
* John Elliott for the fantastic [PCW Hardware guide](https://www.seasip.info/Unix/Joyce/hardware.pdf), the [Joyce emulator](https://www.seasip.info/Unix/Joyce/) and for answering my emails
* Javier Chocano from Habisoft for the [CP/M Box emulator](http://www.habisoft.com/pcw/) and early guidance
* [@zigazou](https://twitter.com/zigazou) for his detailed [wiki article](https://github.com/Zigazou/amstrad-pcw-technical-info/tree/master/video-memory) on the PCW screen memory structure
* [@asicguy](https://github.com/asicguy), [@alanswx](https://github.com/alanswx) and [@dshadoff](https://github.com/dshadoff) for the help, eoncouragment and support developing the core
* [@sorgelig](https://github.com/sorgelig) for the fantastic MiSTer project and work

