    nolist

; DS1306+ control register addresses
DS_SECONDS                  equ 0x00
DS_MINUTES                  equ 0x01
DS_HOURS                    equ 0x02
DS_WEEK_DAY                 equ 0x03
DS_DAY                      equ 0x04
DS_MONTH                    equ 0x05
DS_YEAR                     equ 0x06
DS_SEC_ALM0                 equ 0x07
DS_MIN_ALM0                 equ 0x08
DS_HR_ALM0                  equ 0x09
DS_WEEK_DAY_ALM0            equ 0x0A
DS_SEC_ALM1                 equ 0x0B
DS_MIN_ALM1                 equ 0x0C
DS_HR_ALM1                  equ 0x0D
DS_WEEK_DAY_ALM1            equ 0x0E
DS_CTRL                     equ 0x0F
DS_STATUS                   equ 0x10
DS_TRICKLE                  equ 0x11
DS_WR                       equ 0x80    ; OR with above for writeable addresses

; Bits and masks
DS_CTRL_AIE0                equ 0x0     ; Alarm interrupt enable 0
DS_CTRL_AIE1                equ 0x1     ; Alarm interrupt enable 1
DS_CTRL_1HZ                 equ 0x2     ; 1Hz output enable
DS_CTRL_WP                  equ 0x6     ; Write protect (0=RW, 1=RO)

DS_STATUS_IRQF0             equ 0x0     ; Interrupt request flag 0
DS_STATUS_IRQF1             equ 0x1     ; Interrupt request flag 1

DS_TRICKLE_R2K              equ 0x01    ; xxxxxx11  2Kohm resistor
DS_TRICKLE_R4K              equ 0x02    ; xxxxxx11  4Kohm resistor
DS_TRICKLE_R8K              equ 0x03    ; xxxxxx11  8Kohm resistor
DS_TRICKLE_1DIODE           equ 0x04    ; xxxx11xx  1 diode
DS_TRICKLE_2DIODES          equ 0x08    ; xxxx11xx  2 diodes
DS_TRICKLE_ENABLE           equ 0xA0    ; 1111xxxx  Enable pattern
DS_TRICKLE_DISABLE          equ 0x50    ; 1111xxxx  Default disable pattern

    list
