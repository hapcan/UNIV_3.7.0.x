;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2014 hapcan.com
;
;   This program is free software: you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation, either version 3 of the License, or
;   (at your option) any later version.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;   Filename:               univ_3-7-0-0.asm
;   Associated diagram:     univ_3-7-0-x.sch
;   Author:                 Jacek Siwilo
;   Note:                   AC motor blind controller
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     04.2014   Original version
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .7                            ;application type [0-255]
    #define    AVERS    .0                         ;application version [0-255]
    #define    FVERS    .0                            ;firmware version [0-255]

    #define    FREV     .0                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-7-0-0-rev0.inc"                         ;project variables
    #include "univ3-fake_bootloader-rev2.inc"    ;fake bootloader for debugging
INCLUDEDFILES   code  
    #include "univ3-routines-rev3.inc"                     ;UNIV 3 CPU routines

;==============================================================================
;===  CONFIG  DATA  ===========================================================
;==============================================================================
EEPROM      code                                                ;default config
    org 0xF00008
    DE      0x09,0x09,0x09,0xFF              ;10s running time for each channel
;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x65, 0x29, 0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV>>8,FREV
;            |     |     |     |     |     |     |_____|_____ firmware revision
;            |     |     |     |     |     |__________________ firmware version
;            |     |     |     |     |_____________________ application version
;            |     |     |     |______________________________ application type
;            |     |     |________________________________ hardware version '3'
;            |_____|______________________________________ hardware type 'UNIV'
;==============================================================================
;===  MOVED VECTORS  ==========================================================
;==============================================================================
;PROGRAM RESET VECTOR
FIRMRESET   code    0x1020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x1030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x1040
        call    LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff   STATUS,STATUS_LOW           ;save STATUS register
        movff   WREG,WREG_LOW               ;save working register
        movff   BSR,BSR_LOW                 ;save BSR register
        movff   FSR0L,FSR0L_LOW             ;save other registers used in high int
        movff   FSR0H,FSR0H_LOW
        movff   FSR1L,FSR1L_LOW
        movff   FSR1H,FSR1H_LOW

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        banksel CANFULL
        btfsc   CANFULL,0                   ;check if CAN received anything
        call    CANInterrupt                ;proceed with CAN interrupt

ExitLowInterrupt
        movff   BSR_LOW,BSR                 ;restore BSR register
        movff   WREG_LOW,WREG               ;restore working register
        movff   STATUS_LOW,STATUS           ;restore STATUS register
        movff   FSR0L_LOW,FSR0L             ;restore other registers used in high int
        movff   FSR0H_LOW,FSR0H
        movff   FSR1L_LOW,FSR1L
        movff   FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff   STATUS,STATUS_HIGH          ;save STATUS register
        movff   WREG,WREG_HIGH              ;save working register
        movff   BSR,BSR_HIGH                ;save BSR register
        movff   FSR0L,FSR0L_HIGH            ;save other registers used in high int
        movff   FSR0H,FSR0H_HIGH
        movff   FSR1L,FSR1L_HIGH
        movff   FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitHighInterrupt           ;main firmware is not ready yet
    ;Timer0
        btfsc   INTCON,TMR0IF               ;Timer0 interrupt? (1000ms)
        rcall   Timer0Interrupt
    ;Timer2    
        btfsc   PIR1,TMR2IF                 ;Timer2 interrupt? (20ms)
        rcall   Timer2Interrupt

ExitHighInterrupt
        movff   BSR_HIGH,BSR                ;restore BSR register
        movff   WREG_HIGH,WREG              ;restore working register
        movff   STATUS_HIGH,STATUS          ;restore STATUS register
        movff   FSR0L_HIGH,FSR0L            ;restore other registers used in high int
        movff   FSR0H_HIGH,FSR0H
        movff   FSR1L_HIGH,FSR1L
        movff   FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:          CAN INTERRUPT
;------------------------------------------------------------------------------
; Overview:         Checks CAN message for response and RTR and saves to FIFO
;------------------------------------------------------------------------------
CANInterrupt
        banksel CANFRAME2
        btfsc   CANFRAME2,0                 ;response message?
    return                                  ;yes, so ignore it and exit
        btfsc   CANFRAME2,1                 ;RTR (Remote Transmit Request)?
    return                                  ;yes, so ignore it and exit
        call    Copy_RXB_RXFIFOIN           ;copies received message to CAN RX FIFO input buffer
        call    WriteToCanRxFIFO            ;saves message to FIFO
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 0 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         1000ms periodical interrupt
;------------------------------------------------------------------------------
Timer0Interrupt:
        call    Timer0Initialization8MHz    ;restart 1000ms Timer   
        call    UpdateUpTime                ;counts time from restart
        call    UpdateTransmitTimer         ;increment transmit timer (seconds after last transmission)
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         20ms periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Initialization8MHz    ;restart timer
        banksel TIMER2_20ms
        setf    TIMER2_20ms                 ;timer 2 interrupt occurred flag
    return
;-------------------------------
Timer2Initialization8MHz
        movlb   0xF
        bcf     PMD1,TMR2MD                 ;enable timer 2
        movlw   0x3F          
        movwf   TMR2                        ;set 20ms (19.999500)
        movlw   b'01001111'                 ;start timer, prescaler=16, postscaler=10
        movwf   T2CON
        bsf     IPR1,TMR2IP                 ;high priority for interrupt
        bcf     PIR1,TMR2IF                 ;clear timer's flag
        bsf     PIE1,TMR2IE                 ;interrupt on
    return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        call    DisAllInt                   ;disable all interrupt
    ;firmware initialization
        rcall   PortInitialization          ;prepare processor ports
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare FIFO buffers
        call    Timer0Initialization8MHz    ;Timer 0 initialization for 1s periodical interrupt 
        call    Timer2Initialization8MHz    ;Timer 2 initialization for 20ms periodical interrupt
        call    Blind_PowerUpStates         ;set blind power up states
    ;firmware ready
        banksel FIRMREADY
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        call    EnAllInt                    ;enable all interrupts

;-------------------------------
Loop:                                       ;main loop
        clrwdt                              ;clear Watchdog timer
        call    ReceiveProcedure            ;check if any msg in RX FIFO and if so - process the msg
        call    TransmitProcedure           ;check if any msg in TX FIFO and if so - transmit it
        rcall   OnceA20ms                   ;do routines only after 20ms interrupt 
        rcall   OnceA1000ms                 ;do routines only after 1000ms interrupt
    bra     Loop

;-------------------------------
OnceA20ms                                   ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER2_20ms
        tstfsz  TIMER2_20ms                 ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    Blind_UpdateTimers          ;update bind timers
        call    Blind_SetRelays             ;set new relay states when needed  
        call    Blind_CalcPos               ;calculate bind position
        call    Blind_SendStates            ;send new states if they changed
        banksel TIMER2_20ms
        clrf    TIMER2_20ms
    return
;-------------------------------
OnceA1000ms                                 ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER0_1000ms
        tstfsz  TIMER0_1000ms               ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    UpdateDelayTimers           ;updates channel timers 
        call    UpdateHealthRegs            ;saves health maximums to eeprom
        banksel TIMER0_1000ms
        clrf    TIMER0_1000ms
    return


;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          PORT INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         It sets processor pins. All unused pins should be set as
;                   outputs and driven low
;------------------------------------------------------------------------------
PortInitialization                          ;default all pins set as analog (portA,B) or digital (portB,C) inputs 
    ;PORT A
        banksel ANCON0                      ;select memory bank
        ;0-digital, 1-analog input
        movlw   b'00000011'                 ;(x,x,x,AN4,AN3,AN2,AN1-boot_mode,AN0-volt)
        movwf   ANCON0
        ;output level
        clrf    LATA                        ;all low
        ;0-output, 1-input
        movlw   b'00000011'                 ;all outputs except, bit<1>-boot_mode, bit<0>-volt
        movwf   TRISA        
    ;PORT B
        ;0-digital, 1-analog input
        movlw   b'00000000'                 ;(x,x,x,x,x,AN10,AN9,AN8)
        movwf   ANCON1
        ;output level
        clrf    LATB                        ;all low
        ;0-output, 1-input
        movlw   b'00001000'                 ;all output except CANRX
        movwf   TRISB
    ;PORT C
        ;output level
        clrf    LATC                        ;all low
        ;0-output, 1-input
        movlw   b'00000000'                 ;all output 
        movwf   TRISC
    return

;------------------------------------------------------------------------------
; Routine:          NODE STATUS
;------------------------------------------------------------------------------
; Overview:         It prepares status messages when status request was
;                   received
;------------------------------------------------------------------------------
NodeStatusRequest
        banksel TXFIFOIN0
        movlw   0x01                        ;this is Blind 1
        movwf   TXFIFOIN6
        movff   Ch1PosU,TXFIFOIN7           ;blind position
        movlw   0x01                        ;set pre value 0x01 (means blind going DOWN)
        movwf   TXFIFOIN8 
        btfsc   RelStateNow,K2              ;is it going DOWN        
        incf    TXFIFOIN8                   ;no, so set 0x02 (means blind going UP)    
        btfss   RelStateNow,K1              ;is channel on
        clrf    TXFIFOIN8                   ;no, so set 0x00 (means blind is stoped)
        movff   Instr1Ch1,TXFIFOIN9         ;info what instruction is waiting for execution
        movlw   0x01
        movwf   TXFIFOIN10                  ;info what instruction is waiting for execution (channel)
        movff   TimerCh1,TXFIFOIN11         ;value of channel timer
        rcall   SendBlindStatus
        ;------------------
        movlw   0x02                        ;this is Blind 2
        movwf   TXFIFOIN6
        movff   Ch2PosU,TXFIFOIN7
        movlw   0x01
        movwf   TXFIFOIN8 
        btfsc   RelStateNow,K4   
        incf    TXFIFOIN8   
        btfss   RelStateNow,K3
        clrf    TXFIFOIN8
        movff   Instr1Ch2,TXFIFOIN9
        movlw   0x02
        movwf   TXFIFOIN10
        movff   TimerCh2,TXFIFOIN11
        rcall   SendBlindStatus
        ;------------------
        movlw   0x03                         ;this is Blind 3
        movwf   TXFIFOIN6
        movff   Ch3PosU,TXFIFOIN7
        movlw   0x01
        movwf   TXFIFOIN8 
        btfsc   RelStateNow,K6   
        incf    TXFIFOIN8   
        btfss   RelStateNow,K5
        clrf    TXFIFOIN8
        movff   Instr1Ch3,TXFIFOIN9
        movlw   0x04
        movwf   TXFIFOIN10
        movff   TimerCh3,TXFIFOIN11
        rcall   SendBlindStatus
    return

;------------------
SendBlindStatus
        movlw   0x30                         ;set relay frame
        movwf   TXFIFOIN0
        movlw   0x70
        movwf   TXFIFOIN1
        bsf     TXFIFOIN1,0                  ;response bit
        movff   NODENR,TXFIFOIN2             ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                    ;unused
        setf    TXFIFOIN5                    ;unused
        call    WriteToCanTxFIFO
    return

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:         Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest
        ;check if timer is needed
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;timer is in INSTR3 for this firmware
        call    DoInstructionLater          ;save instruction for later execution
    return

        ;allow only known values
        banksel INSTR1
        movlw   0x06                        ;INSTR less than?
        cpfslt  INSTR1
    bra     ExitDoInstructionRequest        ;no, so exit
        ;recognize instruction
        movf    INSTR1,W                    
        call    ComputedGoto                ;modify PC according to WREG value
        bra     Instr00                     ;instruction 00
        bra     Instr01                     ;instruction 01
        bra     Instr02                     ;instruction 02
        bra     Instr03                     ;instruction 03
        bra     Instr04                     ;instruction 04
        bra     Instr05                     ;instruction 05

;----------------
ExitDoInstructionRequest
		setf	INSTR1			    	    ;clear instruction
        clrf    TIMER                       ;clear timer
		call	DoInstructionLater          ;clear waiting instruction for channel indicated in INSTR2
    return

;-------------------------------
;Instruction execution
Instr00                                     ;STOP
        call    Channels_OFF
        bra     ExitDoInstructionRequest
Instr01                                     ;UP/STOP
        call    Channels_UP_STOP
        bra     ExitDoInstructionRequest
Instr02                                     ;DOWN/STOP
        call    Channels_DOWN_STOP
        bra     ExitDoInstructionRequest
Instr03                                     ;UP
        call    Channels_UP
        bra     ExitDoInstructionRequest
Instr04                                     ;DOWN
        call    Channels_DOWN
        bra     ExitDoInstructionRequest
Instr05                                     ;START
        call    Channels_START
        bra     ExitDoInstructionRequest

;----------------
;Channels OFF
Channels_OFF
        btfsc   INSTR2,0                    ;channel 1
        rcall   Channel_1_OFF
        btfsc   INSTR2,1                    ;channel 2
        rcall   Channel_2_OFF
        btfsc   INSTR2,2                    ;channel 3
        rcall   Channel_3_OFF
    return
Channel_1_OFF
        bcf     RelStateReq,K1              ;K1 off (channel 1 off)
        bcf     RelStateReq,K2              ;K2 off (channel 1 down)
        return
Channel_2_OFF
        bcf     RelStateReq,K3
        bcf     RelStateReq,K4
        return
Channel_3_OFF
        bcf     RelStateReq,K5
        bcf     RelStateReq,K6
        return

;----------------
;Channels DOWN
Channels_DOWN
        btfsc   INSTR2,0                    ;channel 1
        rcall   Channel_1_DOWN
        btfsc   INSTR2,1                    ;channel 2
        rcall   Channel_2_DOWN
        btfsc   INSTR2,2                    ;channel 3
        rcall   Channel_3_DOWN
    return
Channel_1_DOWN
        bsf     RelStateReq,K1              ;K1 on (channel 1 on)
        bcf     RelStateReq,K2              ;K2 off (channel 1 down)  
        return
Channel_2_DOWN
        bsf     RelStateReq,K3
        bcf     RelStateReq,K4
        return
Channel_3_DOWN
        bsf     RelStateReq,K5
        bcf     RelStateReq,K6
        return

;----------------
;Channels UP
Channels_UP
        btfsc   INSTR2,0                    ;channel 1
        rcall   Channel_1_UP
        btfsc   INSTR2,1                    ;channel 2
        rcall   Channel_2_UP
        btfsc   INSTR2,2                    ;channel 3
        rcall   Channel_3_UP
    return
Channel_1_UP
        bsf     RelStateReq,K1              ;K1 on (channel 1 on)
        bsf     RelStateReq,K2              ;K2 on (channel 1 up)    
        return
Channel_2_UP
        bsf     RelStateReq,K3
        bsf     RelStateReq,K4
        return
Channel_3_UP
        bsf     RelStateReq,K5
        bsf     RelStateReq,K6
        return

;----------------
;Channels DOWN_STOP
Channels_DOWN_STOP
        btfsc   INSTR2,0                    ;channel 1
        rcall   Channel_1_DOWN_STOP
        btfsc   INSTR2,1                    ;channel 2
        rcall   Channel_2_DOWN_STOP
        btfsc   INSTR2,2                    ;channel 3
        rcall   Channel_3_DOWN_STOP
    return
Channel_1_DOWN_STOP
        btfss   RelStateNow,K1              ;is channel on?
        bra     $ + .8                      ;no, so turn down
        bcf     RelStateReq,K1              ;K1 off (channel 1 off)
        bcf     RelStateReq,K2              ;K2 off (channel 1 down)
        return
        bsf     RelStateReq,K1              ;K1 on (channel 1 on)
        bcf     RelStateReq,K2              ;K2 off (channel 1 down)  
        return
Channel_2_DOWN_STOP
        btfss   RelStateNow,K3
        bra     $ + .8
        bcf     RelStateReq,K3
        bcf     RelStateReq,K4
        return
        bsf     RelStateReq,K3
        bcf     RelStateReq,K4
        return
Channel_3_DOWN_STOP
        btfss   RelStateNow,K5
        bra     $ + .8
        bcf     RelStateReq,K5
        bcf     RelStateReq,K6
        return
        bsf     RelStateReq,K5
        bcf     RelStateReq,K6
        return

;----------------
;Channels UP_STOP
Channels_UP_STOP
        btfsc   INSTR2,0                    ;channel 1
        rcall   Channel_1_UP_STOP
        btfsc   INSTR2,1                    ;channel 2
        rcall   Channel_2_UP_STOP
        btfsc   INSTR2,2                    ;channel 3
        rcall   Channel_3_UP_STOP
    return
Channel_1_UP_STOP
        btfss   RelStateNow,K1              ;is channel on?
        bra     $ + .8                      ;no, so turn up
        bcf     RelStateReq,K1              ;K1 off (channel 1 off)
        bcf     RelStateReq,K2              ;K2 off (channel 1 down)
        return
        bsf     RelStateReq,K1              ;K1 on (channel 1 on)
        bsf     RelStateReq,K2              ;K2 on (channel 1 up)  
        return
Channel_2_UP_STOP
        btfss   RelStateNow,K3
        bra     $ + .8
        bcf     RelStateReq,K3
        bcf     RelStateReq,K4
        return
        bsf     RelStateReq,K3
        bsf     RelStateReq,K4
        return
Channel_3_UP_STOP
        btfss   RelStateNow,K5
        bra     $ + .8
        bcf     RelStateReq,K5
        bcf     RelStateReq,K6
        return
        bsf     RelStateReq,K5
        bsf     RelStateReq,K6
        return

;----------------
;Channels_START (one button operating)
Channels_START
        btfsc   INSTR2,0                    ;channel 1
        rcall   Channel_1_START
        btfsc   INSTR2,1                    ;channel 2
        rcall   Channel_2_START
        btfsc   INSTR2,2                    ;channel 3
        rcall   Channel_3_START
    return
Channel_1_START
        btfss   RelStateNow,K1              ;is channel on?
        bra     $ + .8                      ;no, so turn on
        bcf     RelStateReq,K1              ;K1 off (channel 1 off)
        bcf     RelStateReq,K2              ;K2 off (channel 1 down)
        return
        btfsc   BlindLastDir,K2             ;was last movement DOWN
        bcf     RelStateReq,K2              ;no, so K2 off (channel 1 down)
        btfss   BlindLastDir,K2             ;was last movement UP
        bsf     RelStateReq,K2              ;no, so K2 on (channel 1 up)
        bsf     RelStateReq,K1              ;K1 on 
        return        
Channel_2_START
        btfss   RelStateNow,K3
        bra     $ + .8
        bcf     RelStateReq,K3
        bcf     RelStateReq,K4
        return
        btfsc   BlindLastDir,K4
        bcf     RelStateReq,K4
        btfss   BlindLastDir,K4
        bsf     RelStateReq,K4
        bsf     RelStateReq,K3
        return
Channel_3_START
        btfss   RelStateNow,K5
        bra     $ + .8
        bcf     RelStateReq,K5
        bcf     RelStateReq,K6
        return
        btfsc   BlindLastDir,K6
        bcf     RelStateReq,K6
        btfss   BlindLastDir,K6
        bsf     RelStateReq,K6
        bsf     RelStateReq,K5
        return

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION LATER
;------------------------------------------------------------------------------
; Overview:         It saves instruction for particular channel for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionLater
        call    SetTimer                    ;update SUBTIMER1 & SUBTIMER2 registers
        ;identify channels
        banksel INSTR2
        btfsc   INSTR2,0                    ;channel 1
        call    SetChanel1
        btfsc   INSTR2,1                    ;channel 2
        call    SetChanel2
        btfsc   INSTR2,2                    ;channel 3
        call    SetChanel3
ExitDoInstructionLater
    return

;-------------------------------
SetChanel1
        movff   INSTR1,Instr1Ch1            ;copy registers
        movlw   b'00000001'
        movff   WREG,Instr2Ch1
        movff   TIMER,TimerCh1
        movff   SUBTIMER1,SubTmr1Ch1
        movff   SUBTIMER2,SubTmr2Ch1
    return
SetChanel2
        movff   INSTR1,Instr1Ch2
        movlw   b'00000010'
        movff   WREG,Instr2Ch2
        movff   TIMER,TimerCh2
        movff   SUBTIMER1,SubTmr1Ch2
        movff   SUBTIMER2,SubTmr2Ch2
    return
SetChanel3
        movff   INSTR1,Instr1Ch3
        movlw   b'00000100'
        movff   WREG,Instr2Ch3
        movff   TIMER,TimerCh3
        movff   SUBTIMER1,SubTmr1Ch3
        movff   SUBTIMER2,SubTmr2Ch3
    return

;------------------------------------------------------------------------------
; Routine:          BLIND POWER UP STATES
;------------------------------------------------------------------------------
; Overview:         Sets power up states according to configuration
;------------------------------------------------------------------------------
Blind_PowerUpStates
        clrf    RelStateReq                 ;clear requested state register
        clrf    RelStateNow                 ;clear current state register
        clrf    Ch1SwDe                     ;clear switch timer
        clrf    Ch2SwDe                     ;clear switch timer
        clrf    Ch3SwDe                     ;clear switch timer
        ;calculate step for given running time of blind 3
        movff   Ch3RunningTime,WREG         ;get running time of blind 3
        rcall   Blind_CalcStep              ;calculate step
        movff   Ch1PosStepU,Ch3PosStepU     ;move result to step register
        movff   Ch1PosStepH,Ch3PosStepH
        movff   Ch1PosStepL,Ch3PosStepL
        ;calculate step for given running time of blind 2
        movff   Ch2RunningTime,WREG         ;get running time of blind 2
        rcall   Blind_CalcStep              ;calculate step
        movff   Ch1PosStepU,Ch2PosStepU     ;move result to step register
        movff   Ch1PosStepH,Ch2PosStepH
        movff   Ch1PosStepL,Ch2PosStepL
        ;calculate step for given running time of blind 1
        movff   Ch1RunningTime,WREG         ;get running time of blind 1
        rcall   Blind_CalcStep              ;calculate step
    return

;------------------------------------------------------------------------------
; Routine:          CALCULATE BLIND STEP VALUE
;------------------------------------------------------------------------------
; Overview:         It calculates blind step for each 20ms to achieve 255.9999
;                   =0xFF.FFFF (100%) within blind running time.
; Input:            WREG register with value (0-255) which represents (1s-256s)
;                   blind running time.
; Output:           24 bit result (8bit int, 16bit fraction) 
;                   in Ch1PosStep= 0xFFFFFF/(WREG*50+50)
;------------------------------------------------------------------------------
Blind_CalcStep
        mullw   .50                         ;step 50 times per second for 20ms interrupt
        movff   PRODL,DIVISOR_L             ;WREG*50
        movff   PRODH,DIVISOR_H
        movlw   .50                         ;WREG*50+50
        addwf   DIVISOR_L
        clrf    WREG                        ;check if carry
        addwfc  DIVISOR_H     
        setf    DIVIDEND_U                  ;set divident 255.9999 = 0xFF.FFFF (max blind position value)
        setf    DIVIDEND_H
        setf    DIVIDEND_L
        clrf    Ch1PosStepU                 ;clear result         
        clrf    Ch1PosStepH
        clrf    Ch1PosStepL
        clrf    REMAINDER_H                 ;clear reminder
        clrf    REMAINDER_L

        movlw   .24                         ;255/(WREG*50+50)
        movwf   BIT_COUNTER                 ;shift 24 bits
Blind_CalcStep_Loop        
        ;rotate bit by bit
        rlcf    DIVIDEND_L                  ;rotate divident
        rlcf    DIVIDEND_H
        rlcf    DIVIDEND_U
        rlcf    REMAINDER_L,F               ;rotate remainder through MSBit of dividend
        rlcf    REMAINDER_H,F
        ;check if divisor can be substracted from remainder
        movf    DIVISOR_L,W
        subwf   REMAINDER_L,W
        movf    DIVISOR_H,W
        subwfb  REMAINDER_H,W
        btfss   STATUS,C
        bra     Blind_CalcStep_ShiftResult  ;C=!B=0, borrowing, so do not substruct
        ;substract divisor from remainder
        movf    DIVISOR_L,W
        subwf   REMAINDER_L,F
        movf    DIVISOR_H,W
        subwfb  REMAINDER_H,F
Blind_CalcStep_ShiftResult      
        rlcf    Ch1PosStepL                 ;rotate result         
        rlcf    Ch1PosStepH
        rlcf    Ch1PosStepU

        decfsz  BIT_COUNTER                 ;all bits done?
        bra     Blind_CalcStep_Loop         ;not yet
    return

;------------------------------------------------------------------------------
; Routine:          CALCULATE BLIND POSITION
;------------------------------------------------------------------------------
; Overview:         It calculates blind position by watching running time.
;------------------------------------------------------------------------------
Blind_CalcPos
        rcall   Blind_CalcPosCh1            ;blind 1
        rcall   Blind_CalcPosCh2            ;blind 2
        rcall   Blind_CalcPosCh3            ;blind 3
    return
;-------------------------------
Blind_CalcPosCh1
        btfss   RelStateNow,K1              ;is blind working?
    return                                  ;no, so exit
        btfss   RelStateNow,K2              ;yes, is it going UP?
        bra     Blind_CalcPosCh1DOWN        ;no
        movf    Ch1PosStepL,W               ;subtract step from current position
        subwf   Ch1PosL
        movf    Ch1PosStepH,W
        subwfb  Ch1PosH
        movf    Ch1PosStepU,W
        subwfb  Ch1PosU
        btfsc   STATUS,C                    ;is value below 0x000000? (borrow=1) carry=!borrow
    return                                  ;no, so exit
        clrf    Ch1PosU                     ;yes, so set 0x000000
        clrf    Ch1PosH
        clrf    Ch1PosL
    return
Blind_CalcPosCh1DOWN
        movf    Ch1PosStepL,W               ;add step to current position
        addwf   Ch1PosL
        movf    Ch1PosStepH,W
        addwfc  Ch1PosH
        movf    Ch1PosStepU,W
        addwfc  Ch1PosU
        btfss   STATUS,C                    ;value is over 0xFFFFFF? (carry=1)
    return                                  ;no, so exit
        setf    Ch1PosU                     ;yes, so set 0xFFFFFF
        setf    Ch1PosH
        setf    Ch1PosL
    return
;----------------
Blind_CalcPosCh2
        btfss   RelStateNow,K3
    return
        btfss   RelStateNow,K4
        bra     Blind_CalcPosCh2DOWN
        movf    Ch2PosStepL,W
        subwf   Ch2PosL
        movf    Ch2PosStepH,W
        subwfb  Ch2PosH
        movf    Ch2PosStepU,W
        subwfb  Ch2PosU
        btfsc   STATUS,C
    return
        clrf    Ch2PosU
        clrf    Ch2PosH
        clrf    Ch2PosL
    return
Blind_CalcPosCh2DOWN
        movf    Ch2PosStepL,W
        addwf   Ch2PosL
        movf    Ch2PosStepH,W
        addwfc  Ch2PosH
        movf    Ch2PosStepU,W
        addwfc  Ch2PosU
        btfss   STATUS,C
    return
        setf    Ch2PosU
        setf    Ch2PosH
        setf    Ch2PosL
    return
;----------------
Blind_CalcPosCh3
        btfss   RelStateNow,K5
    return
        btfss   RelStateNow,K6
        bra     Blind_CalcPosCh3DOWN
        movf    Ch3PosStepL,W
        subwf   Ch3PosL
        movf    Ch3PosStepH,W
        subwfb  Ch3PosH
        movf    Ch3PosStepU,W
        subwfb  Ch3PosU
        btfsc   STATUS,C
    return
        clrf    Ch3PosU
        clrf    Ch3PosH
        clrf    Ch3PosL
    return
Blind_CalcPosCh3DOWN
        movf    Ch3PosStepL,W
        addwf   Ch3PosL
        movf    Ch3PosStepH,W
        addwfc  Ch3PosH
        movf    Ch3PosStepU,W
        addwfc  Ch3PosU
        btfss   STATUS,C
    return
        setf    Ch3PosU
        setf    Ch3PosH
        setf    Ch3PosL
    return

;------------------------------------------------------------------------------
; Routine:          BLIND UPDATE TIMERS
;------------------------------------------------------------------------------
; Overview:         It updates running and switching timers. Routine must be
;                   called every 20ms.
;------------------------------------------------------------------------------
Blind_UpdateTimers
        rcall   Blind_UpdateSwitchTimersCh1 ;update dalay between switching ON/OFF
        rcall   Blind_UpdateSwitchTimersCh2
        rcall   Blind_UpdateSwitchTimersCh3
        rcall   Blind_UpdateRunTimersCh1    ;update blind running time
        rcall   Blind_UpdateRunTimersCh2
        rcall   Blind_UpdateRunTimersCh3
    return

;-------------------------------
;UPDATE SWITCHING TIMERS
Blind_UpdateSwitchTimersCh1
        tstfsz  Ch1SwDe                     ;already zero?
        decf    Ch1SwDe                     ;no, so decrement
    return
Blind_UpdateSwitchTimersCh2
        tstfsz  Ch2SwDe
        decf    Ch2SwDe
    return
Blind_UpdateSwitchTimersCh3
        tstfsz  Ch3SwDe
        decf    Ch3SwDe
    return

;-------------------------------
;UPDATE RUNNING TIMERS
Blind_UpdateRunTimersCh1
        movf    Ch1UpTiH,W                  ;timer aready zero?
        iorwf   Ch1UpTiL,W
        bnz     $ + .4
    return                                  ;yes, so exit
        clrf    WREG                        ;decrement timer
        decf    Ch1UpTiL
        subwfb  Ch1UpTiH
        movf    Ch1UpTiH,W                  ;timer is zero?
        iorwf   Ch1UpTiL,W
        bz      $ + .4
    return                                  ;no, so exit
        bcf     RelStateReq,K1              ;yes, so request stopping
        bcf     RelStateReq,K2
    return
Blind_UpdateRunTimersCh2
        movf    Ch2UpTiH,W
        iorwf   Ch2UpTiL,W
        bnz     $ + .4
    return
        clrf    WREG
        decf    Ch2UpTiL
        subwfb  Ch2UpTiH
        movf    Ch2UpTiH,W
        iorwf   Ch2UpTiL,W
        bz      $ + .4
    return
        bcf     RelStateReq,K3
        bcf     RelStateReq,K4
    return
Blind_UpdateRunTimersCh3
        movf    Ch3UpTiH,W
        iorwf   Ch3UpTiL,W
        bnz     $ + .4
    return
        clrf    WREG
        decf    Ch3UpTiL
        subwfb  Ch3UpTiH
        movf    Ch3UpTiH,W
        iorwf   Ch3UpTiL,W
        bz      $ + .4
    return
        bcf     RelStateReq,K5
        bcf     RelStateReq,K6
    return

;-------------------------------
;LOAD SWITCHING TIMERS
Blind_LoadSwitchTimerCh1
        movlw   .50                         ;load 50*20ms=1000ms delay
        movwf   Ch1SwDe
    return
Blind_LoadSwitchTimerCh2
        movlw   .50
        movwf   Ch2SwDe
    return
Blind_LoadSwitchTimerCh3
        movlw   .50
        movwf   Ch3SwDe
    return

;-------------------------------
;LOAD RUNNING TIMERS                        ;load (50*running time+50)*20ms
Blind_LoadRunTimerCh1                       ;runnig time (0-255) -> runnig timer (1s-256s)
        banksel Ch1RunningTime
        movlw   .50  
        mulwf   Ch1RunningTime              ;50*running time
        movff   PRODH,Ch1UpTiH
        movff   PRODL,Ch1UpTiL
        addwf   Ch1UpTiL                    ;+50
        clrf    WREG
        addwfc  Ch1UpTiH
    return
Blind_LoadRunTimerCh2
        banksel Ch2RunningTime
        movlw   .50
        mulwf   Ch2RunningTime
        movff   PRODH,Ch2UpTiH
        movff   PRODL,Ch2UpTiL
        addwf   Ch2UpTiL
        clrf    WREG
        addwfc  Ch2UpTiH
    return
Blind_LoadRunTimerCh3
        banksel Ch3RunningTime
        movlw   .50
        mulwf   Ch3RunningTime
        movff   PRODH,Ch3UpTiH
        movff   PRODL,Ch3UpTiL
        addwf   Ch3UpTiL
        clrf    WREG
        addwfc  Ch3UpTiH
    return

;------------------------------------------------------------------------------
; Routine:          BLIND SET RELAYS
;------------------------------------------------------------------------------
; Overview:         It sets monostable relays according to RelStateReq reg.
;                   Only one relay is set at a time.
;------------------------------------------------------------------------------
Blind_SetRelays
        ;control channel 1
        rcall   Blind_SetRelaysCh1OFF       ;check if should go OFF
        tstfsz  WREG                        ;any relay was switched? (then WREG=1)
        bra     EndBlind_SetRelays          ;yes, so exit (one relay at a time)
        rcall   Blind_SetRelaysCh1DOWN      ;check if should go DOWN
        tstfsz  WREG                        ;any relay was switched? (then WREG=1)
        bra     EndBlind_SetRelays          ;yes, so exit (one relay at a time)
        rcall   Blind_SetRelaysCh1UP        ;check if should go UP
        tstfsz  WREG                        ;any relay was switched? (then WREG=1)
        bra     EndBlind_SetRelays          ;yes, so exit (one relay at a time)
        ;control channel 2
        rcall   Blind_SetRelaysCh2OFF
        tstfsz  WREG
        bra     EndBlind_SetRelays
        rcall   Blind_SetRelaysCh2DOWN
        tstfsz  WREG
        bra     EndBlind_SetRelays
        rcall   Blind_SetRelaysCh2UP
        tstfsz  WREG
        bra     EndBlind_SetRelays
        ;control channel 3
        rcall   Blind_SetRelaysCh3OFF
        tstfsz  WREG
        bra     EndBlind_SetRelays
        rcall   Blind_SetRelaysCh3DOWN
        tstfsz  WREG
        bra     EndBlind_SetRelays
        rcall   Blind_SetRelaysCh3UP
        tstfsz  WREG
        bra     EndBlind_SetRelays
EndBlind_SetRelays
        movff   RelStateNow,LATC            ;move new values to LAT register
    return

;-------------------------------
;channel1
Blind_SetRelaysCh1OFF
        ;check request
        btfsc   RelStateReq,K1              ;channel should go off?
        retlw   0x00                        ;no, so exit with "no changes" flag
        ;check current state and operate
        btfss   RelStateNow,K1              ;channel is on?
        bra     $ + .8                      ;no, so check direction relay
        bcf     RelStateNow,K1              ;yes, so turn if off
        rcall   Blind_LoadSwitchTimerCh1    ;load switching timer
        retlw   0x01                        ;exit with "relay changed" flag
        btfss   RelStateNow,K2              ;direction relay is on?
        retlw   0x00                        ;no, so exit with "no changes" flag
        bcf     RelStateNow,K2              ;turn off direction relay
        retlw   0x01                        ;exit with "relay changed" flag
;----------------
Blind_SetRelaysCh1DOWN
        ;check if busy
        tstfsz  Ch1SwDe                     ;channel busy?
        retlw   0x00                        ;yes, so exit with "no changes" flag 
        ;check request
        btfss   RelStateReq,K1              ;channel should go on?
        retlw   0x00                        ;no, so exit with "no changes" flag 
        btfsc   RelStateReq,K2              ;channel should go DOWN?
        retlw   0x00                        ;no, so exit with "no changes" flag 
        ;check current state and operate
        btfss   RelStateNow,K1              ;channel is on?
        bra     Down_Ch1IsOff               ;no
        bra     Down_Ch1IsOn                ;yes
Down_Ch1IsOff
        btfss   RelStateNow,K2              ;channel is DOWN?
        bra     $ + .6                      ;yes
        bcf     RelStateNow,K2              ;no, set it down
        retlw   0x01                        ;exit with "relay changed" flag        
        bsf     RelStateNow,K1              ;set channel on
        bcf     BlindLastDir,K2             ;set last direction as DOWN
        rcall   Blind_LoadRunTimerCh1       ;load running timer
        retlw   0x01                        ;exit with "relay changed" flag
Down_Ch1IsOn
        btfss   RelStateNow,K2              ;channel is DOWN?
        retlw   0x00                        ;yes, so exit with "no changes" flag 
        bcf     RelStateNow,K1              ;no, so turn channel off
        rcall   Blind_LoadSwitchTimerCh1    ;load switching timer
        retlw   0x01                        ;exit with "relay changed" flag
;----------------
Blind_SetRelaysCh1UP
        ;check if busy
        tstfsz  Ch1SwDe                     ;channel busy?
        retlw   0x00                        ;yes, so exit with "no changes" flag 
        ;check request
        btfss   RelStateReq,K1              ;channel should go on?
        retlw   0x00                        ;no, so exit with "no changes" flag 
        btfss   RelStateReq,K2              ;channel should go UP?
        retlw   0x00                        ;no, so exit with "no changes" flag 
        ;check current state and operate
        btfss   RelStateNow,K1              ;channel is on?
        bra     Up_Ch1IsOff                 ;no
        bra     Up_Ch1IsOn                  ;yes
Up_Ch1IsOff
        btfsc   RelStateNow,K2              ;channel is UP?
        bra     $ + .6                      ;yes
        bsf     RelStateNow,K2              ;no, set it up
        retlw   0x01                        ;exit with "relay changed" flag        
        bsf     RelStateNow,K1              ;set channel on
        bsf     BlindLastDir,K2             ;set last direction as UP
        rcall   Blind_LoadRunTimerCh1       ;load running timer
        retlw   0x01                        ;exit with "relay changed" flag
Up_Ch1IsOn
        btfsc   RelStateNow,K2              ;channel is UP?
        retlw   0x00                        ;yes, so exit with "no changes" flag 
        bcf     RelStateNow,K1              ;no, so turn channel off
        rcall   Blind_LoadSwitchTimerCh1    ;load switching timer
        retlw   0x01                        ;exit with "relay changed" flag

;-------------------------------
;channel2
Blind_SetRelaysCh2OFF
        ;check request
        btfsc   RelStateReq,K3
        retlw   0x00
        ;check current state and operate
        btfss   RelStateNow,K3
        bra     $ + .8
        bcf     RelStateNow,K3
        rcall   Blind_LoadSwitchTimerCh2
        retlw   0x01
        btfss   RelStateNow,K4
        retlw   0x00
        bcf     RelStateNow,K4
        retlw   0x01
;----------------
Blind_SetRelaysCh2DOWN
        ;check if busy
        tstfsz  Ch2SwDe
        retlw   0x00
        ;check request
        btfss   RelStateReq,K3
        retlw   0x00
        btfsc   RelStateReq,K4
        retlw   0x00
        ;check current state and operate
        btfss   RelStateNow,K3
        bra     Down_Ch2IsOff 
        bra     Down_Ch2IsOn
Down_Ch2IsOff
        btfss   RelStateNow,K4
        bra     $ + .6
        bcf     RelStateNow,K4
        retlw   0x01     
        bsf     RelStateNow,K3
        bcf     BlindLastDir,K4
        rcall   Blind_LoadRunTimerCh2
        retlw   0x01
Down_Ch2IsOn
        btfss   RelStateNow,K4
        retlw   0x00
        bcf     RelStateNow,K3
        rcall   Blind_LoadSwitchTimerCh2
        retlw   0x01 
;----------------
Blind_SetRelaysCh2UP
        ;check if busy
        tstfsz  Ch2SwDe
        retlw   0x00
        ;check request
        btfss   RelStateReq,K3
        retlw   0x00
        btfss   RelStateReq,K4
        retlw   0x00
        ;check current state and operate
        btfss   RelStateNow,K3
        bra     Up_Ch2IsOff
        bra     Up_Ch2IsOn
Up_Ch2IsOff
        btfsc   RelStateNow,K4
        bra     $ + .6
        bsf     RelStateNow,K4
        retlw   0x01        
        bsf     RelStateNow,K3
        bsf     BlindLastDir,K4
        rcall   Blind_LoadRunTimerCh2
        retlw   0x01
Up_Ch2IsOn
        btfsc   RelStateNow,K4
        retlw   0x00
        bcf     RelStateNow,K3
        rcall   Blind_LoadSwitchTimerCh2
        retlw   0x01

;-------------------------------
;channel3
Blind_SetRelaysCh3OFF
        ;check request
        btfsc   RelStateReq,K5
        retlw   0x00
        ;check current state and operate
        btfss   RelStateNow,K5
        bra     $ + .8
        bcf     RelStateNow,K5
        rcall   Blind_LoadSwitchTimerCh3
        retlw   0x01
        btfss   RelStateNow,K6
        retlw   0x00
        bcf     RelStateNow,K6
        retlw   0x01
;----------------
Blind_SetRelaysCh3DOWN
        ;check if busy
        tstfsz  Ch3SwDe
        retlw   0x00
        ;check request
        btfss   RelStateReq,K5
        retlw   0x00
        btfsc   RelStateReq,K6
        retlw   0x00
        ;check current state and operate
        btfss   RelStateNow,K5
        bra     Down_Ch3IsOff 
        bra     Down_Ch3IsOn
Down_Ch3IsOff
        btfss   RelStateNow,K6
        bra     $ + .6
        bcf     RelStateNow,K6
        retlw   0x01     
        bsf     RelStateNow,K5
        bcf     BlindLastDir,K6
        rcall   Blind_LoadRunTimerCh3
        retlw   0x01
Down_Ch3IsOn
        btfss   RelStateNow,K6
        retlw   0x00
        bcf     RelStateNow,K5
        rcall   Blind_LoadSwitchTimerCh3
        retlw   0x01 
;----------------
Blind_SetRelaysCh3UP
        ;check if busy
        tstfsz  Ch3SwDe
        retlw   0x00
        ;check request
        btfss   RelStateReq,K5
        retlw   0x00
        btfss   RelStateReq,K6
        retlw   0x00
        ;check current state and operate
        btfss   RelStateNow,K5
        bra     Up_Ch3IsOff
        bra     Up_Ch3IsOn
Up_Ch3IsOff
        btfsc   RelStateNow,K6
        bra     $ + .6
        bsf     RelStateNow,K6
        retlw   0x01        
        bsf     RelStateNow,K5
        bsf     BlindLastDir,K6
        rcall   Blind_LoadRunTimerCh3
        retlw   0x01
Up_Ch3IsOn
        btfsc   RelStateNow,K6
        retlw   0x00
        bcf     RelStateNow,K5
        rcall   Blind_LoadSwitchTimerCh3
        retlw   0x01

;------------------------------------------------------------------------------
; Routine:          SEND BLIND STATES
;------------------------------------------------------------------------------
; Overview:         Sends blind new state when blind starts running or stops
;------------------------------------------------------------------------------
Blind_SendStates
        ;channel 1
        movf    RelStateNow,W               ;compare current and previous state
        xorwf   RelStatePre,W
        btfsc   WREG,K1                     ;K1 changed?
        rcall   Blind_SendStatesCh1         ;yes, so send new state
        ;channel 2
        movf    RelStateNow,W
        xorwf   RelStatePre,W
        btfsc   WREG,K3
        rcall   Blind_SendStatesCh2
        ;channel 3        
        movf    RelStateNow,W
        xorwf   RelStatePre,W
        btfsc   WREG,K5
        rcall   Blind_SendStatesCh3
        ;all channels
        movff   RelStateNow,RelStatePre     ;save current states as previous after sending
    return

;-------------------------------
Blind_SendStatesCh1
        banksel TXFIFOIN0
        movlw   0x01                        ;this is Blind 1
        movwf   TXFIFOIN6
        movff   Ch1PosU,TXFIFOIN7           ;blind position
        movlw   0x01                        ;set pre value 0x01 (means blind going DOWN)
        movwf   TXFIFOIN8 
        btfsc   RelStateNow,K2              ;is it going DOWN        
        incf    TXFIFOIN8                   ;no, so set 0x02 (means blind going UP)    
        btfss   RelStateNow,K1              ;is channel on
        clrf    TXFIFOIN8                   ;no, so set 0x00 (means blind is stoped)
        movff   Instr1Ch1,TXFIFOIN9         ;info what instruction is waiting for execution
        movlw   0x01
        movwf   TXFIFOIN10                  ;info what instruction is waiting for execution (channel)
        movff   TimerCh1,TXFIFOIN11         ;value of channel timer
        rcall   Blind_SendStatesNow
    return
;----------------
Blind_SendStatesCh2
        banksel TXFIFOIN0
        movlw   0x02                        ;this is Blind 2
        movwf   TXFIFOIN6
        movff   Ch2PosU,TXFIFOIN7
        movlw   0x01
        movwf   TXFIFOIN8 
        btfsc   RelStateNow,K4   
        incf    TXFIFOIN8   
        btfss   RelStateNow,K3
        clrf    TXFIFOIN8
        movff   Instr1Ch2,TXFIFOIN9
        movlw   0x02
        movwf   TXFIFOIN10
        movff   TimerCh2,TXFIFOIN11
        rcall   Blind_SendStatesNow
    return
;----------------
Blind_SendStatesCh3
        banksel TXFIFOIN0
        movlw   0x03                        ;this is Blind 3
        movwf   TXFIFOIN6
        movff   Ch3PosU,TXFIFOIN7
        movlw   0x01
        movwf   TXFIFOIN8 
        btfsc   RelStateNow,K6   
        incf    TXFIFOIN8   
        btfss   RelStateNow,K5
        clrf    TXFIFOIN8
        movff   Instr1Ch3,TXFIFOIN9
        movlw   0x04
        movwf   TXFIFOIN10
        movff   TimerCh3,TXFIFOIN11
        rcall   Blind_SendStatesNow
    return
;----------------
Blind_SendStatesNow
        movlw   0x30                        ;set relay frame
        movwf   TXFIFOIN0
        movlw   0x70
        movwf   TXFIFOIN1
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5                   ;unused
        call    WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
    return

;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END