    list

;-------------------------------------------------------------------------------
; schedule_task
; 
; Sets the specified bit in run_tasks to schedule that task to run.
; 
; Arguments:
;       task_bit: run_task bit to set
;-------------------------------------------------------------------------------
schedule_task macro task_bit
    banksel run_tasks
    bsf     run_tasks, task_bit
    
    endm

;-------------------------------------------------------------------------------
; deschedule_task
; 
; Clears the specified bit in run_tasks to deschedule that task from running.
; 
; Arguments:
;       task_bit: run_task bit to clear
;-------------------------------------------------------------------------------
deschedule_task macro task_bit
    banksel run_tasks
    bcf     run_tasks, task_bit
    
    endm

;-------------------------------------------------------------------------------
; sem_v
; 
; Sets the specified semaphore bit in task_sem.
; 
; Arguments:
;       sem_bit: semaphore bit to set
;-------------------------------------------------------------------------------
sem_v macro sem_bit
    banksel task_sem
    bsf     task_sem, sem_bit
    
    endm

;-------------------------------------------------------------------------------
; sem_p
; 
; Tests the specified semaphore bit to determine whether to proceed or skip. If
; the test passes and proceeds, the semaphore is then cleared. Otherwise goto
; the label supplied in skip_label.
; 
; Arguments:
;       sem_bit: semaphore bit to test
;       skip_label: where to go if the semaphore bit is clear
;-------------------------------------------------------------------------------
sem_p macro sem_bit, skip_label
    banksel task_sem
    btfss   task_sem, sem_bit
    goto    skip_label
    
    bcf	    task_sem, sem_bit
    
    endm
