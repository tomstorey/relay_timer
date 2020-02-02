;*******************************************************************************
;                                                                              *
;    Filename:                                                                 *
;    Date:         Sun 19 Jan 2020                                             *
;    File Version:                                                             *
;    Author:       Tom Storey                                                  *
;    Description:  Simple controller to connect/disconnect an attached battery *
;                  from a charger on a schedule.                               *
;                                                                              *
;                  Based on time from an external RTC, a relay is actuated as  *
;                  follows:                                                    *
;                                                                              *
;                  9am - actuate (connect battery)                             *
;                  4pm - release (disconnect battery)                          *
;                                                                              *
;                  This disconnects the battery outside of charging hours to   *
;                  prevent the attached battery from continuing to power the   *
;                  charger itself.                                             *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Revision History:                                                         *
;                                                                              *
;*******************************************************************************

;*******************************************************************************
; Processor Inclusion
;*******************************************************************************

    list    p=16f886

#include "p16f886.inc"
    
    extern init_idata

;*******************************************************************************
; Configuration Word Setup
;*******************************************************************************

 __config _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
 __config _CONFIG2, _BOR4V_BOR40V & _WRT_OFF

;*******************************************************************************
; Variable Definitions
;*******************************************************************************

WDTCON_WDTPS_1_32768        equ b'00010100'     ; xxx1111x
WDTCON_SWDTEN_EN            equ b'00000001'     ; xxxxxxx1
OPTION_REG_PS_1_64          equ b'00000101'     ; xxxxx111

; PORTB bits
PORTB_RTC_INT               equ 0x0
PORTB_TOG_BTN               equ 0x1

; PORTC bits
PORTC_HB_LED                equ 0x0
PORTC_RELAY                 equ 0x1
PORTC_RTC_CE                equ 0x2

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

#ifdef __DEBUG
set_rtc_time_udata udata
ctrl_temp                   res 1       ; Temp store for RTC control reg
#endif

#ifdef __DEBUG
start_udata_dbg udata
ctr1                        res 1       ; Counter values using during debugging
ctr2                        res 1
#endif

; These variables are considered globals
globals_idata idata
portc_out                   res 1       ; PORTC output states
hb_state                    res 1       ; Current state of heartbeat SM
btn_state                   res 1       ; Serviceable buttons!
btn_acks                    res 1       ; Button presses that have been ack'd
main_state                  res 1       ; Current state of main state machine
manual_toggle               res 1       ; Whether relay is manually toggled

globals_udata udata
W_temp                      res 1       ; ISR W temporary
STATUS_temp                 res 1       ; ISR STATUS temporary

;*******************************************************************************
; Reset Vector
;*******************************************************************************

reset_vect  code    0x0
    pagesel start
    goto    start
    nop

;*******************************************************************************
; Interrupt Vector
;*******************************************************************************

int_vect    code    0x4
    pagesel isr_handler
    goto    isr_handler
    nop

;*******************************************************************************
; INITIALISATION
;*******************************************************************************

init_peripherals code
init_peripherals
    ; Configure watchdog - 1 second period -------------------------------------
    banksel WDTCON
    clrwdt
    movlw   WDTCON_WDTPS_1_32768 | WDTCON_SWDTEN_EN
    movwf   WDTCON
    
    ; Configure ports ----------------------------------------------------------
    banksel TRISA
    movlw   b'00000000'
    movwf   TRISA                   ; PORTA directions
    
    movlw   b'00000011'
    ;               |+ RTC interrupt
    ;               +- Relay toggle button
    movwf   TRISB                   ; PORTB directions
    
    movlw   b'00010000'
    ;           |||||+ Heartbeat LED
    ;           ||||+- Relay
    ;           |||+-- RTC CE (positive logic)
    ;           ||+--- SPI SCK
    ;           |+---- SPI SDI
    ;           +----- SPI SDO
    movwf   TRISC                   ; PORTC directions
    
    movlw   0x0                     ; Disable all weak pull-ups on PORTB
    movwf   WPUB
    bsf     OPTION_REG, NOT_RBPU
    
    bsf     IOCB, IOCB0             ; Enable RB0 interrupt on change
    
    banksel ANSEL
    movwf   ANSEL                   ; PORTA all digital I/O
    movwf   ANSELH                  ; PORTB all digital I/O
    
    ; Configure MSSP for SPI master --------------------------------------------
    banksel SSPCON
    movlw   b'00100000'
    ;           ||++++ SSPM 0000: SPI master, clock = Fosc/4
    ;           |+---- CKP 0: Clock idle low
    ;           +----- SSPEN 1: Sync serial port enabled
    movwf   SSPCON
    
    banksel SSPSTAT
    movlw   b'00000000'
    ;         |+------ CKE 0: Tx data on clk rising edge
    ;         +------- SMP 0: Rx data on clk falling edge
    movwf   SSPSTAT
    
    ; Configure Timer0 for ~16ms interrupt -------------------------------------
    bcf     OPTION_REG, T0CS        ; Timer0 clock source = Fosc/4
    bcf     OPTION_REG, PSA         ; Prescaler assigned to Timer0
    bcf     OPTION_REG, PS1         ; Leave PS2 and PS0 set = 1:64 prescale
    
    return

;*******************************************************************************
; ISR HANDLER
;*******************************************************************************

isr_handler code
isr_handler
    banksel W_temp
    movwf   W_temp                  ; Save W and STATUS on entry to ISR
    swapf   STATUS, W
    movwf   STATUS_temp
    
    btfss   INTCON, T0IF            ; Timer0 IF set?
    goto    isr_handler_iocb        ; No, next interrupt
    
    pagesel hb_led
    call    hb_led                  ; Do heartbeat LED stuff
    pagesel buttons
    call    buttons                 ; Do button stuff
    
    bcf     INTCON, T0IF            ; Done with Timer0
    
isr_handler_iocb
    btfss   INTCON, RBIF            ; Port B IOC IF set?
    goto    isr_handler_done        ; No, next interrupt
    
    pagesel rtc_interrupt
    call    rtc_interrupt           ; Handle RTC interrupt
    
    bcf     INTCON, RBIF            ; Done with Port B IOC
    
isr_handler_done
    banksel W_temp
    swapf   STATUS_temp, W          ; Restore W and STATUS on exit
    movwf   STATUS
    swapf   W_temp, F
    swapf   W_temp, W
    
    retfie

;*******************************************************************************
; HEARTBEAT LED
;*******************************************************************************

hb_led_udata udata
hb_state2                   res 1   ; Temp state, used for case selection

hb_led_idata idata
hb_ctr                      res 1   ; Counter for LED flashing

hb_led code
hb_led
    banksel hb_state2
    movf    hb_state, W
    movwf   hb_state2

    ; STATE 0 ------------------------------------------------------------------
hb_led_state0
    btfss   STATUS, Z               ; STATE0 (hb_state was 0)?
    goto    hb_led_state1           ; No
    
    ; In state 0 we increment the HB counter and when it overflows to 0, move
    ; to the next state.
    ; 
    ; In state 0 the HB LED is OFF.
    
    movlw   ~(1 << PORTC_HB_LED)    ; Turn LED off
    andwf   portc_out, F
    movf    portc_out, W
    banksel PORTC
    movwf   PORTC
    
    banksel hb_ctr
    incfsz  hb_ctr, F               ; Overflowed to 0?
    goto    hb_led_done             ; No
    
    movlw   0x1                     ; Yes, go to STATE1
    movwf   hb_state
    movlw   0x1                     ; Put x in counter, LED will be on for this
    movwf   hb_ctr                  ; number of counts
    
    goto    hb_led_done
    
    ; STATE 1 ------------------------------------------------------------------
hb_led_state1
    decfsz  hb_state2, F            ; STATE1?
    goto    hb_led_state2           ; No
    
    ; In state 1 we turn the HB LED on and decrement the HB counter a couple of
    ; times to produce a short blip of the LED.
    
    movlw   1 << PORTC_HB_LED       ; Turn LED on
    iorwf   portc_out, F
    movf    portc_out, W
    banksel PORTC
    movwf   PORTC
    
    banksel hb_ctr
    decfsz  hb_ctr, F               ; LED on for x counts?
    goto    hb_led_done             ; No
    
    movlw   0x0                     ; Yes, to go STATE0
    movwf   hb_state
    
    goto    hb_led_done
    
    ; STATE 2 ------------------------------------------------------------------
hb_led_state2
    decfsz  hb_state2, F            ; STATE2?
    goto    hb_led_state0           ; If no, go to STATE0 (default)
    
    ; In state 2 we do a regular flash of the HB LED.
    decfsz  hb_ctr, F
    goto    hb_led_done
    
    movlw   1 << PORTC_HB_LED       ; Toggle LED
    xorwf   portc_out, F
    movf    portc_out, W
    banksel PORTC
    movwf   PORTC
    
    banksel hb_ctr
    movlw   d'61'                   ; Count this many cycles between LED toggles
    movwf   hb_ctr

hb_led_done
    return

;*******************************************************************************
; BUTTONS
;*******************************************************************************

buttons_idata idata
btn_debounce                res 1   ; Button presses currently being debounced

buttons_udata udata
btn_data                    res 1   ; Copy of PORTB
btn_serviced                res 1   ; Pressed buttons that are ack'd
btn_unserviced              res 1   ; Pressed buttons that are not ack'd

buttons code
buttons
    ; Debounce and present serviceable button presses.
    ; 
    ; Each button press is first debounced, which is achieved by recognising the
    ; button being consistently pressed for two consecutive passes of this
    ; routine. Once this condition is satisfied, a bit is set in btn_state to
    ; signify the button press is serviceable by the software.
    ; 
    ; Once a bit is set in btn_state, the software can acknowledge that it has
    ; seen the button press by setting the corresponding bit in btn_acks. This
    ; routine then hides that button press until the button is released.
    banksel PORTB
    movf    PORTB, W
    
    banksel btn_data
    xorlw   b'00000010'             ; Invert active low signals
    andlw   b'00000010'             ; Mask out non-button inputs
    movwf   btn_data
    
    banksel btn_acks
    andwf   btn_acks, W             ; AND state with acks to get svcd buttons
    banksel btn_serviced
    movwf   btn_serviced
    
    movf    btn_data, W             ; XOR state with serviced to get unserviced
    xorwf   btn_serviced, W
    movwf   btn_unserviced
    
    iorwf   btn_serviced, W         ; Mask to clear acks for released buttons
    banksel btn_acks
    andwf   btn_acks, F             ; Clear acks for released buttons
    
    banksel btn_debounce
    movf    btn_debounce, W
    andwf   btn_unserviced, W       ; Debounced AND unserviced = serviceable
    banksel btn_state
    movwf   btn_state
    
    banksel btn_unserviced
    movf    btn_unserviced, W       ; Unserviced becomes new debounced
    movwf   btn_debounce
    
    return

;*******************************************************************************
; REAL TIME CLOCK INTERRUPT
;*******************************************************************************

rtc_interrupt_idata idata
cur_timer_table             res 1
last_timer_table            res 1

rtc_interrupt_udata udata
rtc_interrupt_stat          res 1
temp_rtc_hours              res 1
temp_timer_val              res 1
rtc_hours                   res 1   ; Hour byte read from the RTC

rtc_interrupt code
rtc_interrupt
    ; When the RTC generates an interrupt, read the hour byte and clear the
    ; IRQ flag from the RTC status register.
    call    rtc_ce_high
    
    movlw   DS_STATUS               ; Tell RTC to read from status register
    pagesel spi_transfer
    call    spi_transfer
    call    spi_transfer            ; Second transfer actually reads
    banksel rtc_interrupt_stat
    movwf   rtc_interrupt_stat      ; Store result for later
    
    call    rtc_ce_low
    
    banksel rtc_interrupt_stat
    movf    rtc_interrupt_stat, W
    andlw   1 << DS_STATUS_IRQF0
    btfsc   STATUS, Z               ; IQRF0 flag was set?
    goto    rtc_interrupt_done      ; No
    
    call    rtc_ce_high
    
    movlw   DS_HOURS                ; Read from Hours register
    call    spi_transfer
    call    spi_transfer
    
    banksel rtc_hours
    movwf   rtc_hours               ; Save hours value for later
    
    call    spi_transfer            ; 5 more reads will read the Alarm 0 seconds
    call    spi_transfer            ; register, which will clear the IRQF0 flag.
    call    spi_transfer
    call    spi_transfer
    call    spi_transfer
    
    call    rtc_ce_low
    
    movlw   high timer_table        ; Setup PCLATH for timer_table call
    movwf   PCLATH
    
    ; Hours are in BCD, so convert to binary
    banksel rtc_hours
    swapf   rtc_hours, W            ; Tens to lower nibble
    andlw   0x0F                    ; Clear upper nibble, W = tens
    movwf   temp_rtc_hours          
    addwf   temp_rtc_hours, W       ; W = 2 * tens
    addwf   temp_rtc_hours, F       ; Temp = 3 * tens
    rlf     temp_rtc_hours, W       ; W = 6 * tens
    subwf   rtc_hours, W            ; rtc_hours = rtc_hours - 6 * tens
    
    ; Look up in a table what the relay output should be during this hour and
    ; set appropriately.
    pagesel timer_table
    call    timer_table
    
    banksel cur_timer_table
    movwf   cur_timer_table         ; Save table value
    
    xorwf   last_timer_table, W
    btfsc   STATUS, Z               ; Has the timer table value changed?
    goto    rtc_interrupt_toggle    ; No
    
    ; If timer value changed, clear manual_toggle
    movlw   0x0
    banksel manual_toggle
    movwf   manual_toggle
    
rtc_interrupt_toggle
    banksel cur_timer_table
    movf    cur_timer_table, W
    movwf   last_timer_table
    
    pagesel update_relay
    call    update_relay
    
rtc_interrupt_done
    return

;*******************************************************************************
; RELAY STATUS JUMP TABLE
;*******************************************************************************

timer_table code
timer_table
    addwf   PCL, F
    
    ; For each entry, return 0 for off, 1 for on
    ; Relay status    Hour
    retlw   0x0     ; 0
    retlw   0x0     ; 1
    retlw   0x0     ; 2
    retlw   0x0     ; 3
    retlw   0x0     ; 4
    retlw   0x0     ; 5
    retlw   0x0     ; 6
    retlw   0x0     ; 7
    retlw   0x0     ; 8
    retlw   0x0     ; 9
    retlw   0x1     ; 10
    retlw   0x1     ; 11
    retlw   0x1     ; 12
    retlw   0x1     ; 13
    retlw   0x1     ; 14
    retlw   0x1     ; 15
    retlw   0x0     ; 16
    retlw   0x0     ; 17
    retlw   0x0     ; 18
    retlw   0x0     ; 19
    retlw   0x0     ; 20
    retlw   0x0     ; 21
    retlw   0x0     ; 22
    retlw   0x0     ; 23

;*******************************************************************************
; REAL TIME CLOCK INITIALISATION
;*******************************************************************************

init_rtc_udata udata
init_rtc_ctrl               res 1   ; Working copy of control reg value

init_rtc code
init_rtc
    ; RTC is configured to cause an interrupt once per minute. On each interrupt
    ; the time will be read, and based on a table we determine whether the relay
    ; should be activated or not.
    ; 
    ; In this code we will only work with the RTC configuration registers.
    ; Separate code is used to configure the time as a setup/config process.
    
    ; Write new control register values
    pagesel spi_transfer
    call    rtc_ce_high
    
    movlw   DS_WR | DS_CTRL         ; Start writing from control register again
    call    spi_transfer
    
    movlw   1 << DS_CTRL_AIE0       ; Enable alarm interrupt 0
    banksel init_rtc_ctrl
    movwf   init_rtc_ctrl           ; Store for later, when we need to set WP
    call    spi_transfer
    
    movlw   0x0                     ; Write zeroes over status
    call    spi_transfer
    
    movlw   DS_TRICKLE_ENABLE | DS_TRICKLE_2DIODES | DS_TRICKLE_R8K
    call    spi_transfer            ; Enable trickle charger function
    
    call    rtc_ce_low
    
    ; Configure alarm 0 for once per minute
    call    rtc_ce_high
    
    movlw   DS_WR | DS_SEC_ALM0
    call    spi_transfer
    
    movlw   0x0                     ; Seconds value - MSb clear for interrupt
    call    spi_transfer            ; once per minute
    
    movlw   0x80                    ; Minutes, hours, day of week values
    call    spi_transfer            ; MSb set for interrupt once per minute
    movlw   0x80
    call    spi_transfer
    movlw   0x80
    call    spi_transfer
    
    call    rtc_ce_low
    
    ; Configure write protection
    call    rtc_ce_high
    
    movlw   DS_CTRL | DS_WR         ; Start writing from control register again
    call    spi_transfer
    
    banksel init_rtc_ctrl
    movf    init_rtc_ctrl, W        ; Set WP bit to lock
    iorlw   1 << DS_CTRL_WP
    call    spi_transfer
    
    call    rtc_ce_low
    
    ; Read an alarm 0 register to clear any pending IRQs
    call    rtc_ce_high
    
    movlw   DS_SEC_ALM0
    call    spi_transfer
    call    spi_transfer
    
    call    rtc_ce_low
    
    return

;*******************************************************************************
; SPI STUFF
;*******************************************************************************

spi_stuff code
spi_transfer
    ; Transfer a byte using SPI. TX byte is supplied in W, and RX byte is in W
    ; on return.
    banksel SSPBUF
    movwf   SSPBUF                  ; W to SSPBUF starts tx
    
    banksel SSPSTAT
    btfss   SSPSTAT, BF             ; Wait for buffer full flag
    goto    $-1
    
    banksel SSPBUF
    movf    SSPBUF, W               ; SSPBUF to W to return rx byte
    
    return

rtc_ce_high
    banksel portc_out
    movf    portc_out, W            ; Sets RTC CE pin high
    iorlw   1 << PORTC_RTC_CE
    movwf   portc_out
    banksel PORTC
    movwf   PORTC
    
    return

rtc_ce_low
    banksel portc_out
    movf    portc_out, W            ; Sets RTC CE pin low
    andlw   ~(1 << PORTC_RTC_CE)
    movwf   portc_out
    banksel PORTC
    movwf   PORTC
    
    return

;*******************************************************************************
; UPDATE RELAY
;*******************************************************************************

update_relay code
update_relay
    banksel last_timer_table
    movf    last_timer_table, W
    banksel manual_toggle
    xorwf   manual_toggle, W
    
    btfss   STATUS, Z               ; Z bit clear?
    goto    update_relay_on         ; No, go to turn relay on
    
    ; Turn relay off
    banksel portc_out
    movf    portc_out, W
    andlw   ~(1 << PORTC_RELAY)
    movwf   portc_out
    banksel PORTC
    movwf   PORTC
    
    ; When relay is off, heartbeat state machine should proceed to state 0 to
    ; produce a brief flash every once in a while
    movlw   0x0
    banksel hb_state
    movwf   hb_state
    
    goto    update_relay_done
    
update_relay_on
    ; Turn relay on
    banksel portc_out
    movf    portc_out, W
    iorlw   1 << PORTC_RELAY
    movwf   portc_out
    banksel PORTC
    movwf   PORTC
    
    ; When relay is on, heartbeat state machine should proceed to state 2 to
    ; produce a regular flash pattern
    movlw   0x2
    banksel hb_state
    movwf   hb_state
    
update_relay_done
    return

;*******************************************************************************
; TOGGLE RELAY BUTTON
;*******************************************************************************

toggle_relay_button code
toggle_relay_button
    ; Pressing the toggle relay button allows the current automated state to be
    ; overridden.
    ; 
    ; Ordinarily, the schedule for the relay is determined by the time kept by
    ; the RTC, and a lookup table to determine whether the relay should be
    ; activated or not depending on the hour of the day.
    ; 
    ; Pressing the toggle button will invert the current state. When the
    ; schedule changes, the toggle setting is reset so that the schedule will
    ; resume as normal.
    ; 
    ;   Schedule  Toggled  State
    ;   --------  -------  -----
    ;   Off       No       Off
    ;   Off       Yes      On
    ;   On        No       On
    ;   On        Yes      Off
    movlw   1 << PORTB_TOG_BTN
    banksel btn_acks
    iorwf   btn_acks, F             ; ACK button press
    
    movlw   0x01
    xorwf   manual_toggle, F        ; Update toggle bit
    
    pagesel update_relay
    call    update_relay            ; Update relay for immediate effect
    
    return

#ifdef __DEBUG
;*******************************************************************************
; SET TIME IN RTC
;*******************************************************************************
set_rtc_time code
set_rtc_time
    ; Quick and dirty routine to set the time/date of the RTC. Only needs to be
    ; called when setting up the RTC for the first time.
    nop
    nop
    
    ; Unlock control reg -------------------------------------------------------
    call    rtc_ce_high
    movlw   DS_CTRL
    call    spi_transfer
    call    spi_transfer
    movwf   ctrl_temp
    call    rtc_ce_low
    
    movlw   ~(1 << DS_CTRL_WP)
    andwf   ctrl_temp, F
    
    call    rtc_ce_high
    movlw   DS_WR | DS_CTRL
    call    spi_transfer
    movf    ctrl_temp, W
    call    spi_transfer
    movwf   ctrl_temp
    call    rtc_ce_low
    
    ; Configure time -----------------------------------------------------------
    call    rtc_ce_high
    ;movlw   DS_WR | DS_SECONDS      ; For writing
    movlw   DS_SECONDS              ; For reading
    call    spi_transfer
    
    ; All values are loaded in BCD
    movlw   0x30                    ; Seconds (00-59)
    call    spi_transfer
    nop
    nop
    
    movlw   0x33                    ; Minutes (00-59)
    call    spi_transfer
    nop
    nop
    
    movlw   0x23                    ; Hours (00-23)
    call    spi_transfer
    nop
    nop
    
    movlw   0x05                    ; Week day (1-7, arbitrary)
    call    spi_transfer
    nop
    nop
    
    movlw   0x31                    ; Day of month (1-31)
    call    spi_transfer
    nop
    nop
    
    movlw   0x01                    ; Month (1-12)
    call    spi_transfer
    nop
    nop
    
    movlw   0x20                    ; Year (00-99)
    call    spi_transfer
    nop
    nop
    
    call    rtc_ce_low
    
    ; Lock control reg ---------------------------------------------------------
    movlw   1 << DS_CTRL_WP
    iorwf   ctrl_temp, F
    
    call    rtc_ce_high
    movlw   DS_WR | DS_CTRL
    call    spi_transfer
    movf    ctrl_temp, W
    call    spi_transfer
    movwf   ctrl_temp
    call    rtc_ce_low
    
    return
#endif

;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

start code
start
#ifdef __DEBUG
    ; Eat up lots of cycles on start while debugging - let it settle down during
    ; programming.
    banksel ctr1
    
    movlw   0x0
    movwf   ctr1
    movwf   ctr2
    
    decfsz  ctr1
    goto    $-1
    
    decfsz  ctr2
    goto    $-3
#endif
    
    ; Initialise various stuff
    pagesel init_idata
    call    init_idata              ; Init IDATA sections
    pagesel init_peripherals
    call    init_peripherals        ; Init peripherals
    pagesel init_rtc
    call    init_rtc                ; Init the RTC
    
#ifdef __DEBUG
    ;call    set_rtc_time
#endif
    
    ; Ready to run, enable interrupts
    movlw   b'10101000'
    movwf   INTCON                  ; GIE, T0IE, RBIE enable
    
main_loop
    ; Check manual on/off button input
    clrwdt
    
    movf    btn_acks, W
    andlw   1 << PORTB_TOG_BTN
    btfss   STATUS, Z               ; Button press already ACKd?
    goto    skip_toggle_button      ; Yes
    
    movf    btn_state, W
    andlw   1 << PORTB_TOG_BTN
    btfss   STATUS, Z               ; Button is pressed?
    pagesel toggle_relay_button
    call    toggle_relay_button     ; Yes
    
skip_toggle_button
    goto    main_loop

    end
