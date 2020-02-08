#include <p16f886.inc>

;*******************************************************************************
; INITIALISE IDATA SECTIONS
; 
; IDATA sections contain initialised data, but the initial values must be loaded
; into RAM from ROM. This is achieved through the use of a table stored in ROM
; that describe how many bytes should be copied from and to where.
; 
; The location of that table in ROM is identified by the _cinit variable.
; 
; The format of the table is as follows (in DWs):
; 
;   Addr    Purpose
;   _cinit  The number of sections to be initialised
;   + 1     Address in ROM where source data starts
;   + 2     Address in RAM where the IDATA section begins
;   + 3     Number of bytes to copy for that section
; 
; The last 3 entries are repeated for the number of sections to be initialised,
; as indicated by the first word.
; 
; Data is loaded from ROM as a series of calls to a jump table, using RETLW
; instructions to return the data.
; 
; Call init_idata at the beginning of a program to initialise IDATA sections.
; 
;*******************************************************************************

    extern      _cinit              ; Location of .cinit table in ROM
    global      init_idata          ; Export init_data

;-------------------------------------------------------------------------------
; set_dest_ptr
; 
; When reading from the .cinit table, configure a pointer where the word will
; be stored in RAM.
; 
; Arguments:
;       reg: A label/name
;-------------------------------------------------------------------------------
set_dest_ptr macro reg
    movlw   low reg
    banksel dest_ptr
    movwf   dest_ptr
    movlw   high reg
    movwf   dest_ptr + 1
    
    endm

;-------------------------------------------------------------------------------
; inc_dw
; 
; Increment a data word reservation - increment the LSB, and then on overflow
; also increment the MSB.
; 
; Arguments:
;       reg: A label/name
;-------------------------------------------------------------------------------
inc_dw macro reg
    banksel reg
    incf    reg, F
    btfsc   STATUS, Z
    incf    reg + 1, F
    
    endm

;-------------------------------------------------------------------------------
; dec_dw
; 
; Decrement a data word reservation - if the LSB is already zero, decrement the
; MSB and then the LSB, otherwise decrement the LSB only.
; 
; Arguments:
;       reg: A label/name
;-------------------------------------------------------------------------------
dec_dw macro reg
    banksel reg
    movf    reg, F
    btfsc   STATUS, Z
    decf    reg + 1, F
    decf    reg, F
    
    endm

init_udata udata
table_addr                  res 2   ; Next table read address
dest_ptr                    res 2   ; Where to store data read from table
table_ctr                   res 1   ; Number of bytes to load from table
num_init                    res 2   ; Number of sections to be initialised
rom_addr                    res 2   ; ROM start location for a section (source)
ram_addr                    res 2   ; RAM start location for a section (dest)
init_sz                     res 2   ; Number of bytes to be copied for section

init_idata code
init_idata
    ; Initialise table_addr to the value of _cinit address
    movlw   low _cinit
    movwf   table_addr
    movlw   high _cinit
    movwf   table_addr + 1
    
    ; Start by loading the number of sections to be initialised
    set_dest_ptr num_init           ; Point dest_ptr to num_init
    call    table_read_dw           ; Read the word in
    
num_init_loop
    banksel num_init
    movf    num_init, W             ; OR two bytes of num_init to test for zero
    iorwf   num_init + 1, W
    btfsc   STATUS, Z               ; num_init zero?
    return                          ; Yes, no more sections to copy
    
    ; No, start copying bytes of data in this section.
    ; Copy 3 words: ROM start, RAM start, section size
    
    ; ROM start address
    set_dest_ptr rom_addr
    call    table_read_dw
    
    ; RAM start address
    set_dest_ptr ram_addr
    call    table_read_dw
    
    ; Section size
    set_dest_ptr init_sz
    call    table_read_dw
    
copy_section_loop
    ; Check size of section
    banksel init_sz
    movf    init_sz, W
    iorwf   init_sz + 1, W
    btfsc   STATUS, Z               ; init_sz zero?
    goto    next_section            ; Yes
    
    ; Use FSR for RAM destination
    banksel ram_addr
    movf    ram_addr, W
    movwf   FSR
    
    call    call_table              ; CALL the table entry to get byte in W
    
    bcf     STATUS, IRP             ; Default to bank 0/1 (RAM 0x0-0xFF)
    movf    ram_addr + 1, F
    btfss   STATUS, Z               ; Is the MSB of the word zero?
    bsf     STATUS, IRP             ; Yes, working in bank 2/3 (RAM 0x100-1FF)
    movwf   INDF                    ; Store W to initialise this RAM address
    
    ; Increment/decrement various counters
    inc_dw rom_addr
    inc_dw ram_addr
    dec_dw init_sz
    
    ; Proceed to copy next byte of data in this section
    goto    copy_section_loop

next_section
    banksel num_init
    movf    num_init, F
    btfsc   STATUS, Z               ; Is LSB of num_init zero?
    decf    num_init + 1, F         ; Yes, decrement MSB
    decf    num_init, F             ; No, decrement LSB only
    
    goto    num_init_loop           ; Proceed to copy next section

;-------------------------------------------------------------------------------
; call_table
; 
; Performs a long jump to the instruction addressed in rom_addr. Used to read
; values for initialised registers.
;-------------------------------------------------------------------------------
call_table
    movf    rom_addr + 1, W
    movwf   PCLATH
    movf    rom_addr, W
    movwf   PCL

;-------------------------------------------------------------------------------
; table_read_dw
; 
; After using set_dest_ptr to configure a destination, read two bytes from ROM
; and store them in RAM to form a DW.
;-------------------------------------------------------------------------------
table_read_dw
    movlw   0x2                     ; Each word is 2 bytes
    movwf   table_ctr

table_read_loop
    banksel dest_ptr
    movf    dest_ptr, W             ; Configure FSR to point to addr in dest_ptr
    movwf   FSR
    
    bcf     STATUS, IRP             ; Default to bank 0/1 (RAM 0x0-0xFF)
    movf    dest_ptr + 1, F
    btfss   STATUS, Z               ; Is the MSB of dest_ptr zero?
    bsf     STATUS, IRP             ; Yes, working in bank 2/3 (RAM 0x100-1FF)
    
    call    table_read         ; Read a value from the table
    movwf   INDF                    ; Move that value to its destination
    inc_dw dest_ptr
    
    decfsz  table_ctr, F            ; Loop again if needed
    goto    table_read_loop
    
    return

table_read
    banksel table_addr
    movf    table_addr + 1, W
    movwf   PCLATH                  ; table_addr high byte to PCLATH
    
    movf    table_addr, W           ; table_addr low byte will be loaded to PCL
    
    incf    table_addr, F           ; Increment table_addr per read
    btfsc   STATUS, Z
    incf    table_addr + 1, F
    
    movwf   PCL                     ; Load W to PCL to perform call
    
    end
