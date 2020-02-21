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
;                  per the jump table timer_table.                             *
;                                                                              *
;                  This disconnects the battery outside of charging hours to   *
;                  prevent the attached battery from continuing to power the   *
;                  charger itself.                                             *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Revision History:                                                         *
;    Date        Who  Description                                              *
;    02/02/2020  TS   Initial release                                          *
;    03/02/2020  TS   Enable WDT and PWRT, changes to bit manipulations        *
;    08/02/2020  TS   Re-write, task based                                     *
;                                                                              *
;*******************************************************************************

    list p=16f886
    errorlevel -302

#include <p16f886.inc>
#include <main.inc>
#include <ds1306p.inc>
    
    extern init_idata

    __config _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_ON & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
    __config _CONFIG2, _BOR4V_BOR40V & _WRT_OFF

;*******************************************************************************
; VARIABLES, CONSTANTS, EQUATES, ETC
;*******************************************************************************

#define VALID_BUTTONS b'00000010'       ; Valid button mask
#define BUTTONS_ACT_LOW b'00000010'     ; Invert active low button signals

#define HB_LED_REG porta_out
#define HB_LED_BIT 0x6

#define RTC_CE_PORT PORTC
#define RTC_CE_REG portc_applied
#define RTC_CE_BIT 0x2

#define RELAY_REG portc_out
#define RELAY_1_PIN 0x0
#define RELAY_2_PIN 0x1

; Semaphore bits used with task_sem
#define RTC_UPDATED 0x0
#define RELAYS_CHANGING 0x1

; run_task bits
#define TASK_BTN_HANDLER 0x0
#define TASK_RTC_INT 0x1
#define TASK_UART 0x3
#define TASK_RELAY_SEQ 0x4
#define TASK_HB_LED 0x6
#define TASK_PORT_UPD 0x7

; State machine states
#define RELAY_SEQ_IDLE 0x0
#define RELAY_SEQ_RLYON 0x1
#define RELAY_SEQ_ONDELAY 0x2
#define RELAY_SEQ_RLYOFF 0x3

#define HB_LED_OFF 0x0
#define HB_LED_ON 0x1
#define HB_LED_TOG 0x2

; Button bits
#define TOGGLE_BTN 0x1

;*******************************************************************************
; GLOBAL VARIABLES
;*******************************************************************************

global_idata idata
porta_out                   res 1       ; PORTA output staging
portc_out                   res 1       ; PORTC output staging
porta_applied               res 1       ; PORTA output applied
portc_applied               res 1       ; PORTC output applied
timer0_flags                res 1       ; Per-task Timer0 interrupt flag
rtc_hour                    res 1       ; Copy of RTC hour register
rtc_month                   res 1       ; Copy of RTC month register
task_sem                    res 1       ; Task semaphores

;*******************************************************************************
; RESET VECTOR
;*******************************************************************************

reset_vect code
    pagesel startup
    goto    startup

;*******************************************************************************
; INTERRUPT VECTOR
;*******************************************************************************

int_vect_udata udata
W_temp                      res 1       ; Save W
STATUS_temp                 res 1       ; Save STATUS
PCLATH_temp                 res 1       ; Save PCLATH
FSR_temp                    res 1       ; Save FSR

int_vect code
int_vect
    ; Save critical registers on entry to ISR
    banksel W_temp
    movwf   W_temp
    swapf   STATUS, W
    movwf   STATUS_temp
    movf    PCLATH, W
    movwf   PCLATH_temp
    movf    FSR, W
    movwf   FSR_temp
    
    pagesel int_vect
    
    btfss   INTCON, T0IF            ; Timer0 IF set?
    goto    isr_intf                ; No, next interrupt
    
    ; Schedule tasks to be run on Timer0 interrupt
    schedule_task TASK_BTN_HANDLER
    schedule_task TASK_HB_LED
    
    banksel timer0_flags
    movlw   0xFF                    ; Set all bits in per-task T0 IF variable
    movwf   timer0_flags
    
    bcf     INTCON, T0IF            ; Done with Timer0
    
isr_intf
    btfss   INTCON, INTF            ; External interrupt IF set?
    goto    isr_rcif                ; No, next interrupt
    
    ; Schedule tasks to be run on INT pin interrupt
    schedule_task TASK_RTC_INT
    
    bcf     INTCON, INTF
    
isr_rcif
    banksel PIR1
    btfss   PIR1, RCIF              ; UART rx IF set?
    goto    isr_done                ; No, next interrupt
    
    schedule_task TASK_UART
    
    banksel PIR1
    bcf     PIR1, RCIF
    
isr_done
    ; Restore critical registers on exit
    banksel W_temp
    movf    FSR_temp, W
    movwf   FSR
    movf    PCLATH_temp, W
    movwf   PCLATH
    swapf   STATUS_temp, W
    movwf   STATUS
    swapf   W_temp, F
    swapf   W_temp, W
    
    retfie

;*******************************************************************************
; PERIPHERAL INITIALISATION
;*******************************************************************************

init_peripherals code
init_peripherals
    ; Configure watchdog -------------------------------------------------------
    clrwdt
    
    banksel WDTCON
    movlw   b'00010101'
    ;            ||||+- SWDTEN 1: WDT is turned on
    ;            ++++-- WDTPS: 1:32768
    movwf   WDTCON
    
    ; Configure ports ----------------------------------------------------------
    banksel TRISA
    movlw   b'00000000'
    ;         +-------- Heartbeat LED
    movwf   TRISA                   ; PORTA directions
    
    movlw   b'00000011'
    ;               |+- RTC interrupt
    ;               +-- Relay toggle button
    movwf   TRISB                   ; PORTB directions
    
    movlw   b'00010000'
    ;         |||||||+- Relay 1
    ;         ||||||+-- Relay 2
    ;         |||||+--- RTC CE (positive logic)
    ;         ||||+---- SPI SCK
    ;         |||+----- SPI SDI
    ;         ||+------ SPI SDO
    ;         |+------- UART TX (reserved)
    ;         +-------- UART RX (reserved)
    movwf   TRISC                   ; PORTC directions
    
    clrf    WPUB                    ; Disable all pull-ups on PORTB
    clrf    IOCB                    ; Disable interrupt on change on PORTB
    
    banksel ANSEL
    clrf    ANSEL                   ; PORTA all digital I/O
    clrf    ANSELH                  ; PORTB all digital I/O
    
    ; Configure MSSP for SPI master --------------------------------------------
    banksel SSPCON
    movlw   b'00100000'
    ;           ||++++- SSPM 0000: SPI master, clock = Fosc/4
    ;           |+----- CKP 0: Clock idle low
    ;           +------ SSPEN 1: Sync serial port enabled
    movwf   SSPCON
    
    banksel SSPSTAT
    movlw   b'00000000'
    ;         |+------- CKE 0: Tx data on clk rising edge
    ;         +-------- SMP 0: Rx data on clk falling edge
    movwf   SSPSTAT
    
    bcf     PIR1, SSPIF
    
    ; Configure Timer0, WPUBs, INT pin -----------------------------------------
    movlw   b'10010101'
    ;         ||| |+++- PS 101: Timer0 1:64 prescaler
    ;         ||| +---- Prescaler assigned to Timer0
    ;         ||+------ T0CS 0: Timer0 clock = Fosc/4
    ;         |+------- INTEDG 0: Interrupt on INT falling edge
    ;         +-------- WPUB 1: PORTB pull-ups disabled
    movwf   OPTION_REG
    
    return

;*******************************************************************************
; UTILITY ROUTINES
;*******************************************************************************

bcd_to_bin_udata udata
bcd_W                       res 1
bcd_temp                    res 1

bcd_to_bin code
bcd_to_bin
    banksel bcd_W
    movwf   bcd_W                   ; Store W for later
    
    swapf   bcd_W, W                ; Tens to lower nibble
    andlw   0x0F                    ; Clear upper nibble, W = tens
    movwf   bcd_temp          
    addwf   bcd_temp, W             ; W = 2 * tens
    addwf   bcd_temp, F             ; Temp = 3 * tens
    rlf     bcd_temp, W             ; W = 6 * tens
    subwf   bcd_W, W                ; W = bcd_W - 6 * tens
    
    return

;*******************************************************************************
; RTC INITIALISATION
; 
; The RTC is configured to cause an interrupt once per minute. On each interrupt
; the hour register will be read and stored for use by the application logic
; task.
; 
; In this routine, only the RTC control registers are adjusted to achieve the
; desired operation of alarms and interrupts.
;*******************************************************************************

init_rtc_udata udata
init_rtc_ctrl               res 1   ; Working copy of control reg value

init_rtc code
init_rtc
    ; Write new control register values
    pagesel spi_transfer
    call    rtc_ce_high
    
    movlw   DS_WR | DS_CTRL         ; Start writing from control register
    call    spi_transfer
    
    movlw   1 << DS_CTRL_AIE0       ; Enable alarm 0 interrupt. With the WP bit
                                    ; set to 0, writes will be allowed. 1Hz o/p
                                    ; and alarm 1 interrupt are also disabled.
    banksel init_rtc_ctrl
    movwf   init_rtc_ctrl           ; Store for later, when we need to set WP
    call    spi_transfer
    
    clrw
    call    spi_transfer            ; Write zeroes over status register
    
    ; Configure trickle charger function for the backup battery
    movlw   DS_TRICKLE_ENABLE | DS_TRICKLE_2DIODES | DS_TRICKLE_R8K
    call    spi_transfer
    
    call    rtc_ce_low              ; End this transaction
    
    ; Configure alarm 0 for once per minute
    call    rtc_ce_high
    
    movlw   DS_WR | DS_SEC_ALM0     ; Start writing from alarm 0 seconds reg
    call    spi_transfer
    
    clrw                            ; Seconds value - MSb clear for interrupt
    call    spi_transfer            ; once per minute
    
    movlw   0x80                    ; Minutes, hours, day of week values
    call    spi_transfer            ; MSb set for interrupt once per minute
    movlw   0x80
    call    spi_transfer
    movlw   0x80
    call    spi_transfer
    
    call    rtc_ce_low
    
    ; Configure write protection for control registers
    call    rtc_ce_high
    
    movlw   DS_CTRL | DS_WR         ; Start writing from control register again
    call    spi_transfer
    
    banksel init_rtc_ctrl           ; Set WP bit to lock
    bsf     init_rtc_ctrl, DS_CTRL_WP
    movf    init_rtc_ctrl, W
    call    spi_transfer
    
    call    rtc_ce_low
    
    ; Read one alarm 0 and alarm 1 register to clear any pending IRQs
    call    rtc_ce_high
    
    movlw   DS_WEEK_DAY_ALM0
    call    spi_transfer
    call    spi_transfer            ; Reads alarm 0 week day reg
    call    spi_transfer            ; Reads alarm 1 seconds reg
    
    call    rtc_ce_low
    
    return

;*******************************************************************************
; SPI UTILITIES
;*******************************************************************************

spi_util code
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
    ; Sets RTC CE pin high to begin a transaction
    banksel RTC_CE_REG
    bsf     RTC_CE_REG, RTC_CE_BIT
    movf    RTC_CE_REG, W           
    banksel RTC_CE_PORT
    movwf   RTC_CE_PORT
    
    return

rtc_ce_low
    ; Sets RTC CE pin low to end a transaction
    banksel RTC_CE_REG
    bcf     RTC_CE_REG, RTC_CE_BIT
    movf    RTC_CE_REG, W           
    banksel RTC_CE_PORT
    movwf   RTC_CE_PORT
    
    return

;*******************************************************************************
; BUTTON HANDLER TASK
; 
; Debounce and present serviceable button presses.
; 
; Each button press is first debounced, which is achieved by recognising the
; button being consistently pressed for two consecutive passes of this routine.
; Once this condition is satisfied, a bit is set in btn_state to signify the
; button press is serviceable by the software.
; 
; Once a bit is set in btn_state, the software can acknowledge that it has seen
; the button press by setting the corresponding bit in btn_acks. This routine
; then hides that button press until the button is released.
;*******************************************************************************

buttons_idata idata
btn_state                   res 1   ; Serviceable buttons!
btn_acks                    res 1   ; Button presses that have been ack'd
btn_debounce                res 1   ; Button presses currently being debounced
btn_valid                   db  VALID_BUTTONS ; Valid button mask

buttons_udata udata
btn_data                    res 1   ; Copy of PORTB masked for valid button bits
btn_serviced                res 1   ; Pressed buttons that are ack'd
btn_unserviced              res 1   ; Pressed buttons that are not ack'd

buttons code
buttons
    banksel PORTB
    movf    PORTB, W
    
    banksel btn_data
    xorlw   BUTTONS_ACT_LOW         ; Invert active low signals
    andwf   btn_valid, W            ; Mask out ignorable inputs
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
    
    movf    btn_debounce, W
    banksel btn_unserviced
    andwf   btn_unserviced, W       ; Debounced AND unserviced = serviceable
    banksel btn_state
    movwf   btn_state
    
    banksel btn_unserviced
    movf    btn_unserviced, W       ; Unserviced becomes new debounced
    banksel btn_debounce
    movwf   btn_debounce
    
    deschedule_task TASK_BTN_HANDLER
    
    return

;*******************************************************************************
; RTC INTERRUPT TASK
; 
; This task will read the current hour from the RTC. The hours value is used in
; the application logic task to determine the state of the relays.
; 
; It will first check the STATUS register of the RTC to ensure that the
; interrupt that has been received is for the correct Alarm.
;*******************************************************************************

rtc_interrupt_udata udata
rtc_int_status              res 1   ; Copy of status register

rtc_interrupt code
rtc_interrupt
    pagesel spi_transfer
    call    rtc_ce_high
    
    movlw   DS_STATUS               ; Start reading from status register
    call    spi_transfer
    call    spi_transfer            ; Second transfer reads register
    
    banksel rtc_int_status
    movwf   rtc_int_status          ; Store status for later
    
    call    rtc_ce_low
    
    ; Alarm 0 interrupt occurred?
    pagesel rtc_interrupt
    banksel rtc_int_status
    btfss   rtc_int_status, DS_STATUS_IRQF0
    goto    rtc_interrupt_done      ; No
    
    ; Yes, read hours register and store it
    pagesel spi_transfer
    call    rtc_ce_high
    
    movlw   DS_HOURS                ; Read from hour register
    call    spi_transfer
    call    spi_transfer
    
    banksel rtc_hour
    movwf   rtc_hour                ; Save BCD hour value for now
    
    call    spi_transfer            ; Read but ignore day register
    call    spi_transfer            ; Read but ignore date register
    call    spi_transfer            ; Read month register
    
    ; Convert BCD month value to binary
    pagesel bcd_to_bin
    call    bcd_to_bin
    banksel rtc_month
    movwf   rtc_month               ; Save BCD month value for now
    
    ; Convert BCD hour value to binary
    movf    rtc_hour, W
    call    bcd_to_bin
    banksel rtc_hour
    movwf   rtc_hour
    
    pagesel rtc_ce_low
    call    rtc_ce_low
    
    ; Read one alarm 0 and alarm 1 register to clear any IRQs
    call    rtc_ce_high
    
    movlw   DS_WEEK_DAY_ALM0
    call    spi_transfer
    call    spi_transfer            ; Reads alarm 0 week day reg
    call    spi_transfer            ; Reads alarm 1 seconds reg
    
    call    rtc_ce_low
    
rtc_interrupt_done
    deschedule_task TASK_RTC_INT
    sem_v RTC_UPDATED
    
    return

;*******************************************************************************
; APPLICATION LOGIC
; 
; This is a task which ALWAYS runs. It cannot be (de)scheduled via the run_tasks
; variable.
; 
; The application logic task takes all of the inputs from tasks that preceeded
; it, and generates new state for output tasks that follow it.
;*******************************************************************************

app_idata idata
cur_timer                   res 1
last_timer                  res 1
manual_toggle               res 1

app_udata udata
app_changes                 res 1

app code
app
    banksel app_changes
    clrf    app_changes
    
    ; Check if toggle button was pressed
    banksel btn_state
    btfss   btn_state, TOGGLE_BTN
    goto    check_rtc               ; Skip if button not pressed
    
    bsf     btn_acks, TOGGLE_BTN    ; Ack the button press
    schedule_task TASK_BTN_HANDLER  ; Will mask out the ack'd button
    
    banksel manual_toggle
    movlw   0x1                     ; Update toggle bit
    xorwf   manual_toggle, F
    
    banksel app_changes
    bsf     app_changes, 0x0        ; Indicate changes have been made
    
check_rtc
    ; Has the RTC data been updated?
    sem_p RTC_UPDATED, check_changes
    
    ; Yes, determine if relay should be on or off
    movlw   high timer_table        ; Setup new PCLATH for timer_table call
    movwf   PCLATH
    
    banksel rtc_hour
    movf    rtc_hour, W             ; Put hour in W
    
    call    timer_table             ; Call table
    
    banksel cur_timer
    movwf   cur_timer               ; Save for now
    
    movlw   high app                ; Restore PCLATH after table call
    movwf   PCLATH
    
    ; Work out if we are in month Apr-Aug, swap nibbles of cur_timer if so
    banksel rtc_month
    movf    rtc_month, W
    andlw   0x0C                    ; Keep 2 MSb of lower nibble
    sublw   0x4                     ; Subtract 4
    banksel cur_timer
    btfsc   STATUS, Z
    swapf   cur_timer, F            ; If zero, month is Apr-Jul, swap nibbles
    
    banksel rtc_month
    movf    rtc_month, W
    sublw   0x8                     ; Subtract 8
    banksel cur_timer
    btfsc   STATUS, Z
    swapf   cur_timer, F            ; If zero, month is Aug, swap nibbles
    
    ; Clear manual toggle if timer table changed
    movf    cur_timer, W            ; Save only the LSb
    andlw   0x1
    movwf   cur_timer
    xorwf   last_timer, W           ; Has the value changed?
    btfsc   STATUS, Z
    goto    save_cur_to_last        ; No
    
    clrf    manual_toggle           ; Yes, clear manual toggle bit
    banksel app_changes
    bsf     app_changes, 0x0        ; And indicate changes have been made
    
save_cur_to_last
    banksel cur_timer
    movf    cur_timer, W            ; Cur value becomes last value
    movwf   last_timer
    
check_changes
    banksel app_changes
    btfss   app_changes, 0x0        ; Any pending changes?
    goto    app_done                ; No
    
    banksel last_timer
    movf    last_timer, W           ; XOR last with manual toggle to determine
    xorwf   manual_toggle, W        ; new relay state
    
    btfss   STATUS, Z
    goto    turn_relays_on
    
    ; Turn relays off
    banksel relay_state
    movlw   RELAY_SEQ_RLYOFF        ; Update relay sequencer state
    movwf   relay_state
    banksel hb_state
    movlw   HB_LED_OFF              ; Update hearbeat LED state
    movwf   hb_state
    
    schedule_task TASK_RELAY_SEQ
    
    goto    app_done
    
turn_relays_on
    banksel relay_state
    movlw   RELAY_SEQ_RLYON         ; Update relay sequencer state
    movwf   relay_state
    banksel hb_state
    movlw   HB_LED_TOG              ; Update hearbeat LED state
    movwf   hb_state
    
    schedule_task TASK_RELAY_SEQ
    
app_done
    return

;*******************************************************************************
; TIMER JUMP TABLE
; 
; Given the current hour as an input, return a literal indicating whether the
; relays should be switched on or off.
; 
; The table returns a value with two potential settings.
; 
; In the least significant nibble is the setting for months Sep-Mar.
; 
; In the most significant nibble is the setting for months Apr-Aug.
;*******************************************************************************

timer_table code
timer_table
    addwf   PCL, F
    
    ; Relay status    Hour
    retlw   0x00    ; 0
    retlw   0x00    ; 1
    retlw   0x00    ; 2
    retlw   0x00    ; 3
    retlw   0x00    ; 4
    retlw   0x00    ; 5
    retlw   0x10    ; 6
    retlw   0x10    ; 7
    retlw   0x11    ; 8
    retlw   0x11    ; 9
    retlw   0x11    ; 10
    retlw   0x11    ; 11
    retlw   0x11    ; 12
    retlw   0x11    ; 13
    retlw   0x11    ; 14
    retlw   0x11    ; 15
    retlw   0x10    ; 16
    retlw   0x10    ; 17
    retlw   0x00    ; 18
    retlw   0x00    ; 19
    retlw   0x00    ; 20
    retlw   0x00    ; 21
    retlw   0x00    ; 22
    retlw   0x00    ; 23

;*******************************************************************************
; RELAY SEQUENCER TASK
; 
; When the relays are to be turned on, they are turned on in a sequence as
; follows:
; 
; 1. Relay 1 is turned on
; 2. Delay
; 3. Relay 2 is turned on
; 
; This sequence is intended to permit certain types of chargers to have their
; solar and battery feeds connected in the correct sequence to ensure correct
; voltage operation.
; 
; The relay sequencer is only needed for turning relays on. When turning relays
; off, they can both be switched off together.
; 
; The state machine implemented here has the following states:
; 
; 0. Idle, no operating being performed, task descheduled
; 1. Switch on each relay successively, proceed to 2
; 2. Perform delay, proceed to 1
; 3. Switch off both relays, proceed to 0
;*******************************************************************************

relay_seq_idata idata
relay_state                 res 1   ; Relay sequencer state machine state
relay_ctr                   res 1   ; Counter for producing delay

relay_seq_udata udata
relay_temp                  res 1

relay_seq code
relay_seq
    banksel relay_state
    
    ; State 0 ------------------------------------------------------------------
    ; 
    ; In state 0 nothing happens and the task is descheduled.
relay_seq_state0
    movf    relay_state, F
    btfss   STATUS, Z               ; In state 0?
    goto    relay_seq_state1        ; No
    
    ; Nothing happens in state 0, and when this state is reached, the task
    ; should be descheduled.
    deschedule_task TASK_RELAY_SEQ
    
    banksel btn_valid
    movlw   VALID_BUTTONS           ; Restore button mask to allow manual toggle
    movwf   btn_valid
    
    goto    relay_seq_done
    
    ; State 1 ------------------------------------------------------------------
    ; 
    ; In state 1, each relay will be turned on through successive iterations.
    ; 
    ; When both relays are on, proceed to state 0.
relay_seq_state1
    movf    relay_state, W
    sublw   RELAY_SEQ_RLYON
    btfss   STATUS, Z               ; In state 1?
    goto    relay_seq_state2        ; No
    
    banksel btn_valid
    clrf    btn_valid               ; Disallow button presses during transition
    
    banksel RELAY_REG
    movf    RELAY_REG, W            ; Take copy of relay port staging register
    andlw   0x3                     ; Keep 2 LSb's
    banksel relay_temp
    movwf   relay_temp
    bsf     STATUS, C               ; Set the carry flag
    rlf     relay_temp, F           ; Rotate carry into LSb position
    
    banksel RELAY_REG
    movf    RELAY_REG, W            ; Take copy of relay port staging register
    andlw   ~0x03                   ; Clear 2 LSB's
    banksel relay_temp
    iorwf   relay_temp, W           ; OR in relay state
    banksel RELAY_REG
    movwf   RELAY_REG               ; Put back to port staging reg
    
    schedule_task TASK_PORT_UPD
    
    banksel relay_temp
    movf    relay_temp, W
    sublw   0x3
    btfss   STATUS, Z               ; Are both relays on?
    goto    relay_seq_state1_no
    
    ; Both relays are on, go to state 0
    movlw   RELAY_SEQ_IDLE
    banksel relay_state
    movwf   relay_state
    
    goto    relay_seq_done
    
relay_seq_state1_no
    ; One relay on, perform delay
    movlw   RELAY_SEQ_ONDELAY       ; Go to state 2
    banksel relay_state
    movwf   relay_state
    
    movlw   0x40                    ; Delay for this many counts
    movwf   relay_ctr
    
    goto    relay_seq_done
    
    ; State 2 ------------------------------------------------------------------
    ; 
    ; In state 2 a delay is produced between switching each relay on. Only
    ; decrement when the allocated timer0_flags bit is set.
relay_seq_state2
    movf    relay_state, W
    sublw   RELAY_SEQ_ONDELAY
    btfss   STATUS, Z               ; In state 2?
    goto    relay_seq_state3        ; No
    
    ; Is the relay sequencer Timer0 flag set?
    banksel timer0_flags
    btfss   timer0_flags, TASK_RELAY_SEQ
    goto    relay_seq_done          ; No
    
    ; Yes, clear until next Timer0 event
    bcf     timer0_flags, TASK_RELAY_SEQ
    movlw   RELAY_SEQ_RLYON         ; Setup next state value
    
    banksel relay_ctr
    decf    relay_ctr, F            ; Decrement counter
    btfsc   STATUS, Z               ; Is counter zero?
    movwf   relay_state             ; Yes, apply new state
    
    goto    relay_seq_done
    
    ; State 3 ------------------------------------------------------------------
    ; 
    ; In state 3 both relays will be turned off.
relay_seq_state3
    movf    relay_state, W
    sublw   RELAY_SEQ_RLYOFF
    btfss   STATUS, Z               ; In state 3?
    goto    relay_seq_done          ; No
    
    banksel RELAY_REG
    movf    RELAY_REG, W            ; Take copy of relay port staging register
    andlw   ~0x03                   ; Clear 2 LSB's
    movwf   RELAY_REG               ; Put back to port staging reg
    
    schedule_task TASK_PORT_UPD
    
    goto    relay_seq_done
    
    ; Default ------------------------------------------------------------------
relay_seq_default
    banksel relay_state
    clrf    relay_state             ; Default to state 0
    
    goto    relay_seq
    
relay_seq_done
    return

;*******************************************************************************
; HEARTBEAT LED TASK
; 
; A simple state machine which produces different flash patterns on the
; heartbeat LED to indicate different states or modes of operation.
;*******************************************************************************

hb_led_idata idata
hb_state                    res 1   ; Current state of heartbeat SM
hb_ctr                      res 1   ; Counter for LED flashing

hb_led code
hb_led
    banksel hb_state
    
    ; State 0 ------------------------------------------------------------------
    ; 
    ; In state 0 the HB LED is off while the HB counter is incremented. Upon
    ; overflow to 0, move to the next state.
hb_led_state0
    movf    hb_state, F
    btfss   STATUS, Z               ; In state 0?
    goto    hb_led_state1           ; No
    
    banksel HB_LED_REG
    btfss   HB_LED_REG, HB_LED_BIT
    goto    hb_led_state0_inc
    
    bcf     HB_LED_REG, HB_LED_BIT  ; Turn LED off
    schedule_task TASK_PORT_UPD
    
hb_led_state0_inc
    banksel hb_ctr
    incfsz  hb_ctr, F               ; Overflowed to 0?
    goto    hb_led_done             ; No
    
    movlw   HB_LED_ON               ; Yes, go to state 1
    movwf   hb_state
    movlw   0x1                     ; LED will be on for this number of counts
    movwf   hb_ctr
    
    goto    hb_led_done
    
    ; State 1 ------------------------------------------------------------------
    ; 
    ; In state 1 the HB LED is on while the HB counter is decremented to zero to
    ; produce a nominally short blip of the LED.
hb_led_state1
    movf    hb_state, W
    sublw   HB_LED_ON
    btfss   STATUS, Z               ; In state 1?
    goto    hb_led_state2           ; No
    
    banksel HB_LED_REG
    btfsc   HB_LED_REG, HB_LED_BIT
    goto    hb_led_state1_dec
    
    bsf     HB_LED_REG, HB_LED_BIT  ; Turn LED on
    schedule_task TASK_PORT_UPD
    
hb_led_state1_dec
    banksel hb_ctr
    decfsz  hb_ctr, F               ; LED on for x counts?
    goto    hb_led_done             ; No
    
    movlw   HB_LED_OFF
    movwf   hb_state                ; Yes, to go state 0
    
    goto    hb_led_done
    
    ; State 2 ------------------------------------------------------------------
    ; 
    ; In state 2 a regular flash of the HB LED is produced.
hb_led_state2
    movf    hb_state, W
    sublw   HB_LED_TOG
    btfss   STATUS, Z               ; In state 2?
    goto    hb_led_default          ; No
    
    banksel hb_ctr
    decfsz  hb_ctr, F
    goto    hb_led_done
    
    banksel HB_LED_REG
    movlw   1 << HB_LED_BIT         ; Toggle LED
    xorwf   HB_LED_REG, F
    
    schedule_task TASK_PORT_UPD
    
    banksel hb_ctr
    movlw   0x40                    ; This many counts between LED toggles
    movwf   hb_ctr
    
    goto    hb_led_done
    
    ; Default ------------------------------------------------------------------
hb_led_default
    banksel hb_state
    clrf    hb_state                ; Default to state 0
    
    goto    hb_led

hb_led_done
    deschedule_task TASK_HB_LED
    
    return

;*******************************************************************************
; PORT UPDATER TASK
; 
; The PORT updater task takes each of the staged PORT variables and applies them
; to their respective PORT registers to apply any changes that have been
; computed from recently run tasks.
; 
; The x_applied variables are used to provide an out-of-band method for things
; like the SPI chip select routines to fiddle port bits without applying
; port states which are still being computed by other tasks.
;*******************************************************************************

port_upd code
port_upd
    ; Apply PORTA updates
    banksel porta_out
    movf    porta_out, W            ; Config that we want to apply
    movwf   porta_applied           ; Save to applied
    banksel PORTA
    movwf   PORTA                   ; Apply to PORT register
    
    ; Apply PORTC updates
    banksel portc_out
    movf    portc_out, W
    movwf   portc_applied
    banksel PORTC
    movwf   PORTC
    
    deschedule_task TASK_PORT_UPD
    
    return

;*******************************************************************************
; STARTUP
;*******************************************************************************

#ifdef __DEBUG
startup_debug_udata udata
debug_ctr1                  res 1       ; Counter values using during debugging
debug_ctr2                  res 1
#endif

startup code
startup
#ifdef __DEBUG
    ; When debugging, the PIC can encounter a number of resets. Eat up a lot of
    ; cycles on start during debugging to prevent the application from partially
    ; executing between these resets.
    banksel debug_ctr1
    
    clrf    debug_ctr1
    clrf    debug_ctr2
    
    decfsz  debug_ctr1, F
    goto    $-1
    
    decfsz  debug_ctr2, F
    goto    $-3
#endif
    
    pagesel init_idata
    call    init_idata              ; Init IDATA sections
    pagesel init_peripherals
    call    init_peripherals        ; Init peripherals
    pagesel init_rtc
    call    init_rtc                ; Init the RTC
    
    ; Ready to run, enable interrupts
    movlw   b'10110000'
    movwf   INTCON                  ; GIE, T0IE, INTE enable
    
    pagesel task_runner
    goto    task_runner
    
;*******************************************************************************
; TASK RUNNER
; 
; task_runner executes all tasks due to be run by looking at the respective
; bits in the run_tasks variable.
; 
; Tasks are simply run in the order of their respective run_task bit.
; 
; Some tasks will be scheduled via the ISR, while others can be scheduled by
; another task. For example, the application logic task may determine that the
; relays are due to be switched on, thus will schedule the relay sequencer task
; by setting its task bit.
; 
; run_tasks has the following bit (task) assignments:
; 
; 0: Button handler
; 1: RTC interrupt
; 2: 
; 3: 
; 4: Relay sequencer
; 5: 
; 6: Heartbeat LED
; 7: PORT updater
; 
; Each task is responsible for descheduling itself. The application logic task
; always runs between tasks 3 and 4.
;*******************************************************************************

task_runner_idata idata
run_tasks                   res 1       ; Tasks scheduled to run

task_runner code
task_runner
    ; Button handler task ------------------------------------------------------
    banksel run_tasks
    btfss   run_tasks, TASK_BTN_HANDLER ; Button handler scheduled?
    goto    task_rtc_int                ; No
    
    pagesel buttons
    call    buttons
    
    ; RTC interrupt task -------------------------------------------------------
task_rtc_int
    banksel run_tasks
    btfss   run_tasks, TASK_RTC_INT     ; RTC interrupt scheduled?
    goto    task_app                    ; No
    
    pagesel rtc_interrupt
    call    rtc_interrupt
    
    ; Application logic task ---------------------------------------------------
task_app
    pagesel app
    call    app
    
    ; Relay sequencer task -----------------------------------------------------
task_relay_seq
    banksel run_tasks
    btfss   run_tasks, TASK_RELAY_SEQ   ; Relay sequencer scheduled?     
    goto    task_hb_led                 ; No
    
    pagesel relay_seq
    call    relay_seq
    
    ; Heartbeat LED task -------------------------------------------------------
task_hb_led
    banksel run_tasks
    btfss   run_tasks, TASK_HB_LED      ; Heartbeat LED scheduled?
    goto    task_port_upd               ; No
    
    pagesel hb_led
    call    hb_led
    
    ; PORT updater task -------------------------------------------------------
task_port_upd
    banksel run_tasks
    btfss   run_tasks, TASK_PORT_UPD    ; PORT updater scheduled?
    goto    task_runner_done            ; No
    
    pagesel port_upd
    call    port_upd
    
task_runner_done
    clrwdt                          ; Clear watchdog counter after each run
    
    goto    task_runner             ; Run tasks again as needed
    
    end
