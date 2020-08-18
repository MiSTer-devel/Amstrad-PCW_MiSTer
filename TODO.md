# PCW_MiSTer - To Do's:

* Fixes and features to implement
  * Fix top line showing bottom line issue - Done
  * Check why resolution is now reporting 1440 x 256 - Fixed
  * Create boot from bios on startup - Done
  * Fix disk boot issue with fast CPU (ST0=40,ST1=04 in RW_DATA_EXEC3) - Done
  * Fix corruption writing at 16x and higher - Done
  * Fix Beeper sound - Done
  * Add DKTronics joystick and sound support - Done
  * Get Mouse support working - AMS Done, Kempston Done, Keymouse Done
  * Find issue with intermittent disk corruption
  * Find screen corruption issue with HoE at boot
  * RAMtest has corrupt screen at start
  * Fix Blagger stuck key
  * Create fake daisywheel module to boot 3.5" disks
  

  * CF2DD support
    * Disk 40 - Drive 40 = 1 : 1 Track
    * Disk 40 - Drive 80 = 1 : 2 Track
    * Disk 80 - Drive 40 = 1 : 1 Track - will error
    * Disk 80 - Drive 80 = 1 : 1 Tract
    * SENSE_INT_STATE - returns modified cylinder number
    * SEEK needs cylinder halving

* Future features
  * Add memory above 128K as SDRAM and options for different memory sizes
  * Get multiple drives working

* Notes
  * System timing is 64Mhz clock
  * Video clock is 16 Mhz

