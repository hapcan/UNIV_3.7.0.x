    NOLIST
;==============================================================================
;   MIT License
;
;   Copyright (c) 2022 HAPCAN Home Automation Project (http://hapcan.com)
;
;   Permission is hereby granted, free of charge, to any person obtaining a copy
;   of this software and associated documentation files (the "Software"), to deal
;   in the Software without restriction, including without limitation the rights
;   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;   copies of the Software, and to permit persons to whom the Software is
;   furnished to do so, subject to the following conditions:
;
;   The above copyright notice and this permission notice shall be included in all
;   copies or substantial portions of the Software.
;
;   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;   SOFTWARE.
;==============================================================================

;==============================================================================
;                         DEFAULT CONFIG DATA
;==============================================================================
EEPROM      code    0xF00008                                    ;default config
    DE      0x09,0x09,0x09,0xFF
;            |    |    |    
;            |    |    |________________________ 10s running time for channel 3
;            |    |_____________________________ 10s running time for channel 2
;            |__________________________________ 10s running time for channel 1

;==============================================================================
;                         FIRMWARE REGISTER DEFINITIONS
;==============================================================================
;definitions
CHANNELSNR          SET 3           ;number of channels

;eeprom config
Ch1RunningTime  SET     CONFIG0     ;CONFIG0 - channel 1 runninh time (0-255 -> 1s-256s)
Ch2RunningTime  SET     CONFIG1     ;CONFIG1 - channel 2 runninh time (0-255 -> 1s-256s)
Ch3RunningTime  SET     CONFIG2     ;CONFIG2 - channel 3 runninh time (0-255 -> 1s-256s)

;firmware registers
RelStateNow     EQU     H'0000'     ;curent blind relay states
RelStatePre     EQU     H'0001'     ;previous blind relay states
RelStateReq     EQU     H'0002'     ;requested blind relay states
K1              EQU     0           ;bit<0> - K1 channel 1 is  ON if '1' or is OFF if '0'
K2              EQU     1           ;bit<1> - K2 channel 1 is going UP if '1' or is going DOWN if '0'
K3              EQU     2           ;bit<2> - K3 channel 2
K4              EQU     3           ;bit<3> - K4 channel 2
K5              EQU     4           ;bit<4> - K5 channel 3
K6              EQU     5           ;bit<5> - K6 channel 3
BlindLastDir    EQU     H'0003'     ;last blind direction (UP if '1', DOWN if '0')

Ch1UpTiH        EQU     H'0004'     ;channel 1 running UP timer high byte
Ch1UpTiL        EQU     H'0005'     ;channel 1 running UP timer low byte
Ch2UpTiH        EQU     H'0006'     ;channel 2 running UP timer high byte
Ch2UpTiL        EQU     H'0007'     ;channel 2 running UP timer low byte
Ch3UpTiH        EQU     H'0008'     ;channel 3 running UP timer high byte
Ch3UpTiL        EQU     H'0009'     ;channel 3 running UP timer low byte
Ch1SwDe         EQU     H'000A'     ;channel 1 switch delay timer register
Ch2SwDe         EQU     H'000B'     ;channel 2 switch delay timer register
Ch3SwDe         EQU     H'000C'     ;channel 3 switch delay timer register

Ch1PosU         EQU     H'000D'     ;position of blind 1 upper byte
Ch1PosH         EQU     H'000E'     ;position of blind 1 high byte
Ch1PosL         EQU     H'000F'     ;position of blind 1 low byte 
Ch2PosU         EQU     H'0010'     ;position of blind 2 upper byte
Ch2PosH         EQU     H'0011'     ;position of blind 2 high byte
Ch2PosL         EQU     H'0012'     ;position of blind 2 low byte 
Ch3PosU         EQU     H'0013'     ;position of blind 3 upper byte
Ch3PosH         EQU     H'0014'     ;position of blind 3 high byte
Ch3PosL         EQU     H'0015'     ;position of blind 3 low byte 
Ch1PosStepU     EQU     H'0016'     ;step per 20ms to get 100% value for blind 1 upper byte
Ch1PosStepH     EQU     H'0017'     ;step per 20ms to get 100% value for blind 1 high byte
Ch1PosStepL     EQU     H'0018'     ;step per 20ms to get 100% value for blind 1 low byte
Ch2PosStepU     EQU     H'0019'     ;step per 20ms to get 100% value for blind 2 upper byte
Ch2PosStepH     EQU     H'001A'     ;step per 20ms to get 100% value for blind 2 high byte
Ch2PosStepL     EQU     H'001B'     ;step per 20ms to get 100% value for blind 2 low byte
Ch3PosStepU     EQU     H'001C'     ;step per 20ms to get 100% value for blind 3 upper byte  
Ch3PosStepH     EQU     H'001D'     ;step per 20ms to get 100% value for blind 3 high byte
Ch3PosStepL     EQU     H'001E'     ;step per 20ms to get 100% value for blind 3 low byte

DIVIDEND_U      EQU     H'0020'     ;working regs for Blind_CalcStep routine
DIVIDEND_H      EQU     H'0021'
DIVIDEND_L      EQU     H'0022'
DIVISOR_H       EQU     H'0023'
DIVISOR_L       EQU     H'0024'
REMAINDER_H     EQU     H'0025'
REMAINDER_L     EQU     H'0026'
BIT_COUNTER     EQU     H'0027'

;==============================================================================
;                             ROUTINES REGISTERS
;==============================================================================

;------------------------------
;   WORKING INSTRUCTION REGS
;------------------------------
INSTR1      EQU     H'0050'
INSTR2      EQU     H'0051'
INSTR3      EQU     H'0052'
INSTR4      EQU     H'0053'
INSTR5      EQU     H'0054'
INSTR6      EQU     H'0055'
INSTR7      EQU     H'0056'
INSTR8      EQU     H'0057'
TIMER       EQU     H'0058'
SUBTIMER1   EQU     H'0059'
SUBTIMER2   EQU     H'005A'

;------------------------------
;   INTERRUPT SHADOW REGS
;------------------------------
STATUS_HIGH EQU     H'0060'
WREG_HIGH   EQU     H'0061'
BSR_HIGH    EQU     H'0062'
FSR0L_HIGH  EQU     H'0063'
FSR0H_HIGH  EQU     H'0064'
FSR1L_HIGH  EQU     H'0065'
FSR1H_HIGH  EQU     H'0066'
STATUS_LOW  EQU     H'0067'
WREG_LOW    EQU     H'0068'
BSR_LOW     EQU     H'0069'
FSR0L_LOW   EQU     H'006A'
FSR0H_LOW   EQU     H'006B'
FSR1L_LOW   EQU     H'006C'
FSR1H_LOW   EQU     H'006D'
INTCON_SHD  EQU     H'006E'

;------------------------------
;   FIFO REGISTERS
;------------------------------
RXFIFOSIZE  EQU     .42                   ;size of CAN RX FIFO buffer
TXFIFOSIZE  EQU     .42                   ;size of CAN TX FIFO buffer
RXFIFORAM   EQU     0x300                 ;begining of RX FIFO in RAM
TXFIFORAM   EQU     0x500                 ;begining of TX FIFO in RAM
;CAN RX FIFO input buffer
RXFIFOIN0   EQU     H'0070'     
RXFIFOIN1   EQU     H'0071'
RXFIFOIN2   EQU     H'0072'
RXFIFOIN3   EQU     H'0073'
RXFIFOIN4   EQU     H'0074'
RXFIFOIN5   EQU     H'0075'
RXFIFOIN6   EQU     H'0076'
RXFIFOIN7   EQU     H'0077'
RXFIFOIN8   EQU     H'0078'
RXFIFOIN9   EQU     H'0079'
RXFIFOIN10  EQU     H'007A'
RXFIFOIN11  EQU     H'007B'
;CAN RX FIFO output buffer
RXFIFO0     EQU     H'007C'     
RXFIFO1     EQU     H'007D'
RXFIFO2     EQU     H'007E'
RXFIFO3     EQU     H'007F'
RXFIFO4     EQU     H'0080'
RXFIFO5     EQU     H'0081'
RXFIFO6     EQU     H'0082'
RXFIFO7     EQU     H'0083'
RXFIFO8     EQU     H'0084'
RXFIFO9     EQU     H'0085'
RXFIFO10    EQU     H'0086'
RXFIFO11    EQU     H'0087'
;CAN TX FIFO input buffer
TXFIFOIN0   EQU     H'0088'     
TXFIFOIN1   EQU     H'0089'
TXFIFOIN2   EQU     H'008A'
TXFIFOIN3   EQU     H'008B'
TXFIFOIN4   EQU     H'008C'
TXFIFOIN5   EQU     H'008D'
TXFIFOIN6   EQU     H'008E'
TXFIFOIN7   EQU     H'008F'
TXFIFOIN8   EQU     H'0090'
TXFIFOIN9   EQU     H'0091'
TXFIFOIN10  EQU     H'0092'
TXFIFOIN11  EQU     H'0093'
;CAN TX FIFO output buffer
TXFIFO0     EQU     H'0094'     
TXFIFO1     EQU     H'0095'
TXFIFO2     EQU     H'0096'
TXFIFO3     EQU     H'0097'
TXFIFO4     EQU     H'0098'
TXFIFO5     EQU     H'0099'
TXFIFO6     EQU     H'009A'
TXFIFO7     EQU     H'009B'
TXFIFO8     EQU     H'009C'
TXFIFO9     EQU     H'009D'
TXFIFO10    EQU     H'009E'
TXFIFO11    EQU     H'009F'
;CAN FIFO RX registers
RXTOP       EQU     H'00A0'     ;FIFO RX stack top
RXCNT       EQU     H'00A1'     ;FIFO RX current stack depth
RXCNTMX     EQU     H'00A2'     ;FIFO RX maximum reached stack depth
RXFIFOR0    EQU     H'00A3'     ;working register
RXFIFOR1    EQU     H'00A4'     ;working register
;CAN FIFO TX registers
TXTOP       EQU     H'00A5'     ;FIFO RX stack top
TXCNT       EQU     H'00A6'     ;FIFO RX current stack depth
TXCNTMX     EQU     H'00A7'     ;FIFO RX maximum reached stack depth
TXFIFOR0    EQU     H'00A8'     ;working register
TXFIFOR1    EQU     H'00A9'     ;working register

;------------------------------
;   HEALTH CHECK REGISTERS
;------------------------------
CANINTCNT   EQU     H'00AA'     ;counts can reinitialization events
TRSMTTIMER  EQU     H'00AB'     ;counts seconds when transmit buffers are busy

;------------------------------
;   UPTIME REGISTERS
;------------------------------
UPTIME0     EQU     H'00AC'
UPTIME1     EQU     H'00AD'
UPTIME2     EQU     H'00AE'
UPTIME3     EQU     H'00AF'

;------------------------------
;   WORKING REGISTERS
;------------------------------
ROUT0       EQU     H'00B0'     ;CanTransmit routine
ROUT1       EQU     H'00B1'     ;ProcessSystemMessage routine (HealthAskClear)
ROUT2       EQU     H'00B2'     ;CheckAgainstBoxes routine
ROUT3       EQU     H'00B3'     ;CheckAgainstBoxes routine
ROUT4       EQU     H'00B4'     ;ProcessSystemMessage routine (ChannelName)
ROUT5       EQU     H'00B5'     ;ProcessSystemMessage routine (ChannelName)
;Flag register
TIMER0_1000ms EQU   H'00BD'     ;Timer 0 interrupt occurred flag
TIMER2_20ms EQU     H'00BE'     ;Timer 2 interrupt occurred flag
EEPROMTIMER EQU     H'00BF'     ;Save node states to EEPROM timer (6s)


;------------------------------
;   BOX REGISTERS
;------------------------------
BOXADR      EQU     H'00C0'
BOXEN       EQU     H'00C1'
SAVEENBITS  EQU     H'00C2'     ;Box enable bits need to be saved into EEPROM flag

BOXMASK1    EQU     H'00D0'     ;enable/disable/toggle box mask
BOXMASK2    EQU     H'00D1'
BOXMASK3    EQU     H'00D2'
BOXMASK4    EQU     H'00D3'
BOXMASK5    EQU     H'00D4'
BOXMASK6    EQU     H'00D5'
BOXMASK7    EQU     H'00D6'
BOXMASK8    EQU     H'00D7'
BOXMASK9    EQU     H'00D8'
BOXMASK10   EQU     H'00D9'
BOXMASK11   EQU     H'00DA'
BOXMASK12   EQU     H'00DB'
BOXMASK13   EQU     H'00DC'
BOXMASK14   EQU     H'00DD'
BOXMASK15   EQU     H'00DE'
BOXMASK16   EQU     H'00DF'

BOXFIL1     EQU     H'00E0'     ;message filter
BOXFIL2     EQU     H'00E1'
BOXFIL3     EQU     H'00E2'
BOXFIL4     EQU     H'00E3'
BOXFIL5     EQU     H'00E4'
BOXFIL6     EQU     H'00E5'
BOXFIL7     EQU     H'00E6'
BOXFIL8     EQU     H'00E7'
BOXFIL9     EQU     H'00E8'
BOXFIL10    EQU     H'00E9'
BOXFIL11    EQU     H'00EA'
BOXFIL12    EQU     H'00EB'
BOXFIL1C    EQU     H'00EC'     ;message filter condition
BOXFIL2C    EQU     H'00ED'
BOXFIL3C    EQU     H'00EE'
BOXFIL4C    EQU     H'00EF'
BOXFIL5C    EQU     H'00F0'
BOXFIL6C    EQU     H'00F1'
BOXFIL7C    EQU     H'00F2'
BOXFIL8C    EQU     H'00F3'
BOXFIL9C    EQU     H'00F4'
BOXFIL10C   EQU     H'00F5'
BOXFIL11C   EQU     H'00F6'
BOXFIL12C   EQU     H'00F7'
BOXINSTR1   EQU     H'00F8'     ;instruction in box
BOXINSTR2   EQU     H'00F9'
BOXINSTR3   EQU     H'00FA'
BOXINSTR4   EQU     H'00FB'
BOXINSTR5   EQU     H'00FC'
BOXINSTR6   EQU     H'00FD'
BOXINSTR7   EQU     H'00FE'
BOXINSTR8   EQU     H'00FF'

;------------------------------
;   CHANNEL REGISTERS
;------------------------------
Instr1Ch1   EQU     H'0700'
Instr2Ch1   EQU     H'0701'
Instr3Ch1   EQU     H'0702'
Instr4Ch1   EQU     H'0703'
Instr5Ch1   EQU     H'0704'
Instr6Ch1   EQU     H'0705'
Instr7Ch1   EQU     H'0706'
Instr8Ch1   EQU     H'0707'
TimerCh1    EQU     H'0708'
SubTmr1Ch1  EQU     H'0709'
SubTmr2Ch1  EQU     H'070A'
Instr1Ch2   EQU     H'070B'
Instr2Ch2   EQU     H'070C'
Instr3Ch2   EQU     H'070D'
Instr4Ch2   EQU     H'070E'
Instr5Ch2   EQU     H'070F'
Instr6Ch2   EQU     H'0710'
Instr7Ch2   EQU     H'0711'
Instr8Ch2   EQU     H'0712'
TimerCh2    EQU     H'0713'
SubTmr1Ch2  EQU     H'0714'
SubTmr2Ch2  EQU     H'0715'
Instr1Ch3   EQU     H'0716'
Instr2Ch3   EQU     H'0717'
Instr3Ch3   EQU     H'0718'
Instr4Ch3   EQU     H'0719'
Instr5Ch3   EQU     H'071A'
Instr6Ch3   EQU     H'071B'
Instr7Ch3   EQU     H'071C'
Instr8Ch3   EQU     H'071D'
TimerCh3    EQU     H'071E'
SubTmr1Ch3  EQU     H'071F'
SubTmr2Ch3  EQU     H'0720'
Instr1Ch4   EQU     H'0721'
Instr2Ch4   EQU     H'0722'
Instr3Ch4   EQU     H'0723'
Instr4Ch4   EQU     H'0724'
Instr5Ch4   EQU     H'0725'
Instr6Ch4   EQU     H'0726'
Instr7Ch4   EQU     H'0727'
Instr8Ch4   EQU     H'0728'
TimerCh4    EQU     H'0729'
SubTmr1Ch4  EQU     H'072A'
SubTmr2Ch4  EQU     H'072B'
Instr1Ch5   EQU     H'072C'
Instr2Ch5   EQU     H'072D'
Instr3Ch5   EQU     H'072E'
Instr4Ch5   EQU     H'072F'
Instr5Ch5   EQU     H'0730'
Instr6Ch5   EQU     H'0731'
Instr7Ch5   EQU     H'0732'
Instr8Ch5   EQU     H'0733'
TimerCh5    EQU     H'0734'
SubTmr1Ch5  EQU     H'0735'
SubTmr2Ch5  EQU     H'0736'
Instr1Ch6   EQU     H'0737'
Instr2Ch6   EQU     H'0738'
Instr3Ch6   EQU     H'0739'
Instr4Ch6   EQU     H'073A'
Instr5Ch6   EQU     H'073B'
Instr6Ch6   EQU     H'073C'
Instr7Ch6   EQU     H'073D'
Instr8Ch6   EQU     H'073E'
TimerCh6    EQU     H'073F'
SubTmr1Ch6  EQU     H'0740'
SubTmr2Ch6  EQU     H'0741'
Instr1Ch7   EQU     H'0742'
Instr2Ch7   EQU     H'0743'
Instr3Ch7   EQU     H'0744'
Instr4Ch7   EQU     H'0745'
Instr5Ch7   EQU     H'0746'
Instr6Ch7   EQU     H'0747'
Instr7Ch7   EQU     H'0748'
Instr8Ch7   EQU     H'0749'
TimerCh7    EQU     H'074A'
SubTmr1Ch7  EQU     H'074B'
SubTmr2Ch7  EQU     H'074C'
Instr1Ch8   EQU     H'074D'
Instr2Ch8   EQU     H'074E'
Instr3Ch8   EQU     H'074F'
Instr4Ch8   EQU     H'0750'
Instr5Ch8   EQU     H'0751'
Instr6Ch8   EQU     H'0752'
Instr7Ch8   EQU     H'0753'
Instr8Ch8   EQU     H'0754'
TimerCh8    EQU     H'0755'
SubTmr1Ch8  EQU     H'0756'
SubTmr2Ch8  EQU     H'0757'
Instr1Ch9   EQU     H'0758'
Instr2Ch9   EQU     H'0759'
Instr3Ch9   EQU     H'075A'
Instr4Ch9   EQU     H'075B'
Instr5Ch9   EQU     H'075C'
Instr6Ch9   EQU     H'075D'
Instr7Ch9   EQU     H'075E'
Instr8Ch9   EQU     H'075F'
TimerCh9    EQU     H'0760'
SubTmr1Ch9  EQU     H'0761'
SubTmr2Ch9  EQU     H'0762'
Instr1Ch10  EQU     H'0763'
Instr2Ch10  EQU     H'0764'
Instr3Ch10  EQU     H'0765'
Instr4Ch10  EQU     H'0766'
Instr5Ch10  EQU     H'0767'
Instr6Ch10  EQU     H'0768'
Instr7Ch10  EQU     H'0769'
Instr8Ch10  EQU     H'076A'
TimerCh10   EQU     H'076B'
SubTmr1Ch10 EQU     H'076C'
SubTmr2Ch10 EQU     H'076D'

;==============================================================================
;                             COPPIED EEPROM
;==============================================================================
;eeprom last used address
EEPROM_U    EQU     H'0202'
EEPROM_H    EQU     H'0203'
EEPROM_L    EQU     H'0204'
;flash last used address
FLASH_U     EQU     H'0205'
FLASH_H     EQU     H'0206'
FLASH_L     EQU     H'0207'
;eeprom module config
CONFIG0     EQU     H'0208'
CONFIG1     EQU     H'0209'
CONFIG2     EQU     H'020A'
CONFIG3     EQU     H'020B'
CONFIG4     EQU     H'020C'
CONFIG5     EQU     H'020D'
CONFIG6     EQU     H'020E'
CONFIG7     EQU     H'020F'
CONFIG8     EQU     H'0210'
CONFIG9     EQU     H'0211'
CONFIG10    EQU     H'0212'
CONFIG11    EQU     H'0213'
CONFIG12    EQU     H'0214'
CONFIG13    EQU     H'0215'
CONFIG14    EQU     H'0216'
CONFIG15    EQU     H'0217'
CONFIG16    EQU     H'0218'
CONFIG17    EQU     H'0219'
CONFIG18    EQU     H'021A'
CONFIG19    EQU     H'021B'
CONFIG20    EQU     H'021C'
CONFIG21    EQU     H'021D'
CONFIG22    EQU     H'021E'
CONFIG23    EQU     H'021F'
;module id
NODENR      EQU     H'0226'     ;node number
GROUPNR     EQU     H'0227'     ;node group number
;health check regs
;res        EQU     H'0228'
;res        EQU     H'0229'
;res        EQU     H'022A'
RXCNTMXEE   EQU     H'022B'     ;maximum achieved CAN RX FIFO depth
TXCNTMXEE   EQU     H'022C'     ;maximum achieved CAN TX FIFO depth
CANINTCNTEE EQU     H'022D'     ;can engine reinitialization number
RXERRCNTEE  EQU     H'022E'     ;CAN receive errors
TXERRCNTEE  EQU     H'022F'     ;CAN transmit errors
;enable box registers
ENBOX08     EQU     H'0240'     ;boxes 1-8
ENBOX16     EQU     H'0241'
ENBOX24     EQU     H'0242'
ENBOX32     EQU     H'0243'
ENBOX40     EQU     H'0244'
ENBOX48     EQU     H'0245'
ENBOX56     EQU     H'0246'
ENBOX64     EQU     H'0247'
ENBOX72     EQU     H'0248'
ENBOX80     EQU     H'0249'
ENBOX88     EQU     H'024A'
ENBOX96     EQU     H'024B'
ENBOX104    EQU     H'024C'
ENBOX112    EQU     H'024D'
ENBOX120    EQU     H'024E'
ENBOX128    EQU     H'024F'     ;boxes 121-128


;==============================================================================
;                   BOOTLOADER REGISTERS - DO NOT CHANGE
;==============================================================================
;CAN receive buffer
RxBCON      EQU     H'0100'
CANFRAME1   EQU     H'0101'
CANFRAME2   EQU     H'0102'
CANNODE     EQU     H'0103'
CANGROUP    EQU     H'0104'
CANDLC      EQU     H'0105'
CAND0       EQU     H'0106'
CAND1       EQU     H'0107'
CAND2       EQU     H'0108'
CAND3       EQU     H'0109'
CAND4       EQU     H'010A'
CAND5       EQU     H'010B'
CAND6       EQU     H'010C'
CAND7       EQU     H'010D'
CANFULL     EQU     H'010E'     ;CAN buffer full flag
FIRMREADY   EQU     H'010F'     ;firmware started completely flag
;------------------------------
;UART receive buffer
UART0       EQU     H'0110'
UART1       EQU     H'0111'
UART2       EQU     H'0112'
UART3       EQU     H'0113'
UART4       EQU     H'0114'
UART5       EQU     H'0115'
UART6       EQU     H'0116'
UART7       EQU     H'0117'
UART8       EQU     H'0118'
UART9       EQU     H'0119'
UART10      EQU     H'011A'
UART11      EQU     H'011B'
UART12      EQU     H'011C'
UART13      EQU     H'011D'
UART14      EQU     H'011E'
UART15      EQU     H'011F'
UART16      EQU     H'0120'
UART17      EQU     H'0121'
UART18      EQU     H'0122'
UART19      EQU     H'0123'
UART20      EQU     H'0124'
UART21      EQU     H'0125'
UART22      EQU     H'0126'
UART23      EQU     H'0127'
UART24      EQU     H'0128'
UART25      EQU     H'0129'
UART26      EQU     H'012A'
UART27      EQU     H'012B'
UART28      EQU     H'012C'
UART29      EQU     H'012D'
UARTOVF     EQU     H'012E'     ;overflow indicator
UARTCNT     EQU     H'012F'     ;uart buffer counter
;------------------------------
;bootloader other regs
UARTON      EQU     H'016F'     ;indicates that UART bootloader is on when 0xFF
;------------------------------
;bootloader shadow registers for high interrupt
STATUS_H    EQU     H'0170'
WREG_H      EQU     H'0171'
BSR_H       EQU     H'0172'
PIR1_H      EQU     H'0173'
PIR4_H      EQU     H'0174'
PIR5_H      EQU     H'0175'
CANSTAT_H   EQU     H'0176'
CANCON_H    EQU     H'0177'
FSR0L_H     EQU     H'0178'
FSR0H_H     EQU     H'0179'
FSR1L_H     EQU     H'017A'
FSR1H_H     EQU     H'017B'
TABLAT_H    EQU     H'017C'
TBLPTRL_H   EQU     H'017D'
TBLPTRH_H   EQU     H'017E'
TBLPTRU_H   EQU     H'017F'
INTCON_H    EQU     H'0180'
EECON1_H    EQU     H'0181'
EEDATA_H    EQU     H'0182'
EEADRH_H    EQU     H'0183'
EEADR_H     EQU     H'0184'
TRISA_H     EQU     H'0185'
ADCON2_H    EQU     H'0186'
ADCON1_H    EQU     H'0187'
ADCON0_H    EQU     H'0188'
ANCON0_H    EQU     H'0189'
ADRESL_H    EQU     H'018A'
ADRESH_H    EQU     H'018B'
RCSTA1_H    EQU     H'018C'
TXSTA1_H    EQU     H'018D'
PMD1_H      EQU     H'018E'
COMSTAT_H   EQU     H'018F'
;------------------------------
;bootloader shadow registers for low interrupt
STATUS_L    EQU     H'0190'
WREG_L      EQU     H'0191'
BSR_L       EQU     H'0192'
PIR1_L      EQU     H'0193'
PIR4_L      EQU     H'0194'
PIR5_L      EQU     H'0195'
CANSTAT_L   EQU     H'0196'
CANCON_L    EQU     H'0197'
FSR0L_L     EQU     H'0198'
FSR0H_L     EQU     H'0199'
FSR1L_L     EQU     H'019A'
FSR1H_L     EQU     H'019B'
TABLAT_L    EQU     H'019C'
TBLPTRL_L   EQU     H'019D'
TBLPTRH_L   EQU     H'019E'
TBLPTRU_L   EQU     H'019F'
INTCON_L    EQU     H'01A0'
EECON1_L    EQU     H'01A1'
EEDATA_L    EQU     H'01A2'
EEADRH_L    EQU     H'01A3'
EEADR_L     EQU     H'01A4'
TRISA_L     EQU     H'01A5'
ADCON2_L    EQU     H'01A6'
ADCON1_L    EQU     H'01A7'
ADCON0_L    EQU     H'01A8'
ANCON0_L    EQU     H'01A9'
ADRESL_L    EQU     H'01AA'
ADRESH_L    EQU     H'01AB'
RCSTA1_L    EQU     H'01AC'
TXSTA1_L    EQU     H'01AD'
PMD1_L      EQU     H'01AE'
COMSTAT_L   EQU     H'01AF'
;==============================================================================
;==============================================================================
;==============================================================================
    LIST
