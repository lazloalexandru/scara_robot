
/*-------------------------------------------------------------------------
 *
 * MPLAB-C30  dsPIC30F2010 processor header
 *
 * (c) Copyright 2001 Microchip Technology, All rights reserved
 *
 * File Description / Notes:
 * =========================
 * 1] This header file defines special function registers (SFR), and useful  
 *    macros for the dsPIC30Fxxxx Family of Digital Signal
 *    Controllers (also referred to as the dsPIC).  
 * 2] The register and bit names used in this file match the  
 *    dsPIC30Fxxxx data sheets as closely as possible.  
 * 3] The memory locations of the registers defined in this header file are
 *    specified in the respective linker scripts.
 * 4] SFR definitions are listed in the ascending order of memory addresses
 *    and are grouped based on the module they belong to. For e.g., WREG10
 *    is listed before ACCAL, and the Core SFRs are grouped separately
 *    from the Interrupt Controller SFRs or the General Purpose Timer SFRs.
 * 
 * Revision History:
 * =================
 * --------------------------------------------------------------------------
 * Rev:   Date:        Details:                                     Who:
 * --------------------------------------------------------------------------
 * 2.0    23 Apr 2003  New file                                     P Sinha
 * 2.0a   24 Apr 2003  Added CEID bit                               P Sinha
 * 2.0b   30 Apr 2003  Removed suffix 'B' from device number        P Sinha
 * 2.2    17 Jun 2003  Renamed CONV bit to DONE                     P Sinha
 * 2.3    20 Jun 2003  Corrections in Config Fuses comments         P Sinha
 * 2.3b   5  Aug 2003  Added TUN1-4 bits in OSCCON                  P Sinha
 * 2.3c   18 Sep 2003  DFLTCON changes                              P Sinha
 * 2.4    10 Oct 2003  Added macros for data allocation and ISRs    P Sinha
 * 2.5    25 Nov 2003  Renamed TUN1-4 bits to TUNF0-3               P Sinha
 * --------------------------------------------------------------------------
 *
 * ------------------------------------------------------------------------*/

#ifndef __30F2010_H
#define __30F2010_H

/* ------------------------- */ 
/* Core Register Definitions */
/* ------------------------- */

/* W registers W0-W15 */
extern volatile unsigned int WREG0 __attribute__((__near__));
extern volatile unsigned int WREG1 __attribute__((__near__));
extern volatile unsigned int WREG2 __attribute__((__near__));
extern volatile unsigned int WREG3 __attribute__((__near__));
extern volatile unsigned int WREG4 __attribute__((__near__));
extern volatile unsigned int WREG5 __attribute__((__near__));
extern volatile unsigned int WREG6 __attribute__((__near__));
extern volatile unsigned int WREG7 __attribute__((__near__));
extern volatile unsigned int WREG8 __attribute__((__near__));
extern volatile unsigned int WREG9 __attribute__((__near__));
extern volatile unsigned int WREG10 __attribute__((__near__));
extern volatile unsigned int WREG11 __attribute__((__near__));
extern volatile unsigned int WREG12 __attribute__((__near__));
extern volatile unsigned int WREG13 __attribute__((__near__));
extern volatile unsigned int WREG14 __attribute__((__near__));
extern volatile unsigned int WREG15 __attribute__((__near__));

/* SPLIM: Stack Pointer Limit */
extern volatile unsigned int SPLIM __attribute__((__near__));

/* Alternative access structure for the 40-bit accumulators */
typedef struct tagACC {
    unsigned int L;
    unsigned int H;
    unsigned char U;
} ACC;

/* Acc A<15:0> */
extern volatile unsigned int ACCAL __attribute__((__near__));

/* Acc A<31:16> */
extern volatile unsigned int ACCAH __attribute__((__near__));

/* Acc A<39:32> */
extern volatile unsigned char ACCAU __attribute__((__near__));

/* Acc A defined as a structure consisting of the 3 parts */
extern volatile ACC ACCA __attribute__((__near__));

/* Acc B<15:0> */
extern volatile unsigned int ACCBL __attribute__((__near__));

/* Acc B<31:16> */
extern volatile unsigned int ACCBH __attribute__((__near__));

/* Acc B<39:32> */
extern volatile unsigned char ACCBU __attribute__((__near__));

/* Acc B defined as a structure consisting of the 3 parts */
extern volatile ACC ACCB __attribute__((__near__));

/* PCL: Program Counter low word */ 
extern volatile unsigned int PCL __attribute__((__near__));

/* PCH: Program Counter high byte */
extern volatile unsigned char PCH __attribute__((__near__));

/* TBLPAG: Table Page Register */
extern volatile unsigned char TBLPAG __attribute__((__near__));

/* PSVPAG: Program Space Visibility Page Register */
extern volatile unsigned char PSVPAG __attribute__((__near__));

/* RCOUNT: REPEAT loop count */
extern volatile unsigned int RCOUNT __attribute__((__near__));

/* DCOUNT: DO loop count */
extern volatile unsigned int DCOUNT __attribute__((__near__));

/* DOSTARTL: DO loop start address bits <15:0> */
extern volatile unsigned int DOSTARTL __attribute__((__near__));

/* DOSTARTH: DO loop start address bits <23:16> */
extern volatile unsigned int DOSTARTH __attribute__((__near__));

/* DOENDL: DO loop end address bits <15:0> */
extern volatile unsigned int DOENDL __attribute__((__near__));

/* DOENDH: DO loop end address bits <23:16> */
extern volatile unsigned int DOENDH __attribute__((__near__));

/* SR: Status Register */
extern volatile unsigned int SR __attribute__((__near__));
typedef struct tagSRBITS {
        unsigned C      :1;     /* Carry flag                   */
        unsigned Z      :1;     /* Sticky Zero flag             */
        unsigned OV     :1;     /* Overflow flag                */    
        unsigned N      :1;     /* Negative flag                */
        unsigned RA     :1;     /* REPEAT loop active flag      */
        unsigned IPL    :3;     /* CPU Interrupt Priority Level */
        unsigned DC     :1;     /* Digit Carry flag             */
        unsigned DA     :1;     /* DO loop active flag          */
        unsigned SAB    :1;     /* Combined A/B saturation flag */
        unsigned OAB    :1;     /* Combined A/B overflow flag   */    
        unsigned SB     :1;     /* Acc B saturation flag        */
        unsigned SA     :1;     /* Acc A saturation flag        */
        unsigned OB     :1;     /* Acc B overflow flag          */
        unsigned OA     :1;     /* Acc A overflow flag          */
} SRBITS;
extern volatile SRBITS SRbits __attribute__((__near__));

/* CORCON: CPU Mode control Register */
extern volatile unsigned int CORCON __attribute__((__near__));
typedef struct tagCORCONBITS {
        unsigned IF     :1;     /* Integer/Fractional mode              */    
        unsigned RND    :1;     /* Rounding mode                        */
        unsigned PSV    :1;     /* Program Space Visibility enable      */
        unsigned IPL3   :1;     /* CPU Interrupt Priority Level bit 3   */
        unsigned ACCSAT :1;     /* Acc saturation mode                  */
        unsigned SATDW  :1;     /* Data space write saturation enable   */
        unsigned SATB   :1;     /* Acc B saturation enable              */
        unsigned SATA   :1;     /* Acc A saturation enable              */
        unsigned DL     :3;     /* DO loop nesting level status         */
        unsigned EDT    :1;     /* Early DO loop termination control    */
        unsigned US     :1;     /* Signed/Unsigned mode                 */
        unsigned        :3;
} CORCONBITS;
extern volatile CORCONBITS CORCONbits __attribute__((__near__));

/* MODCON: Modulo Addressing Control Register */
extern volatile unsigned int MODCON __attribute__((__near__));
typedef struct tagMODCONBITS {
        unsigned XWM    :4;     /* X-RAGU/X-WAGU modulo addressing register select  */
        unsigned YWM    :4;     /* Y-RAGU modulo addressing register select         */
        unsigned BWM    :4;     /* Bit-reversed addressing register select          */
        unsigned        :2;
        unsigned YMODEN :1;     /* Y-RAGU modulo addressing enable                  */
        unsigned XMODEN :1;     /* X-RAGU/X-WAGU modulo addressing enable           */
} MODCONBITS;
extern volatile MODCONBITS MODCONbits __attribute__((__near__));

/* XMODSRT: X-RAGU/X-WAGU modulo buffer start address */
extern volatile unsigned int XMODSRT __attribute__((__near__));

/* XMODEND: X-RAGU/X-WAGU modulo buffer end address */
extern volatile unsigned int XMODEND __attribute__((__near__));

/* YMODSRT: Y-RAGU modulo buffer start address */
extern volatile unsigned int YMODSRT __attribute__((__near__));

/* YMODEND: Y-RAGU modulo buffer end address */
extern volatile unsigned int YMODEND __attribute__((__near__));

/* XBREV: X-WAGU Bit-reversed Addressing Control Register */
extern volatile unsigned int XBREV __attribute__((__near__));
typedef struct tagXBREVBITS {
        unsigned XB     :15;    /* Bit-reversed addressing register select  */
        unsigned BREN   :1;     /* Bit-reversed addressing enable           */
} XBREVBITS;
extern volatile XBREVBITS XBREVbits __attribute__((__near__));

/* DISICNT: Disable Interrupt Cycle Count */
extern volatile unsigned int DISICNT __attribute__((__near__));
typedef struct tagDISICNTBITS {
        unsigned DISICNT:14;
        unsigned        :2;
} DISICNTBITS;
extern volatile DISICNTBITS DISICNTbits __attribute__((__near__));

/* ----------------------------------------- */
/* Interrupt Controller register definitions */
/* ----------------------------------------- */

/* INTCON1: Interrupt Control Register 1 */
extern volatile unsigned int INTCON1 __attribute__((__near__));
typedef struct tagINTCON1BITS {
        unsigned        :1;
        unsigned OSCFAIL:1;
        unsigned STKERR :1;
        unsigned ADDRERR:1;
        unsigned MATHERR:1;
        unsigned        :3;
        unsigned COVTE  :1;
        unsigned OVBTE  :1;
        unsigned OVATE  :1;
        unsigned        :4;
        unsigned NSTDIS :1;
} INTCON1BITS;
extern volatile INTCON1BITS INTCON1bits __attribute__((__near__));

/* INTCON2: Interrupt Control Register 2 */
extern volatile unsigned int INTCON2 __attribute__((__near__));
typedef struct tagINTCON2BITS {
        unsigned INT0EP :1;
        unsigned INT1EP :1;
        unsigned INT2EP :1;
        unsigned        :11;
        unsigned DISI   :1;  
        unsigned ALTIVT :1;
} INTCON2BITS;
extern volatile INTCON2BITS INTCON2bits __attribute__((__near__));

/* IFS0: Interrupt Flag Status Register 0 */
extern volatile unsigned int IFS0 __attribute__((__near__));
typedef struct tagIFS0BITS {
        unsigned INT0IF :1;
        unsigned IC1IF  :1;
        unsigned OC1IF  :1;
        unsigned T1IF   :1;
        unsigned IC2IF  :1;
        unsigned OC2IF  :1;
        unsigned T2IF   :1;
        unsigned T3IF   :1;
        unsigned SPI1IF :1;
        unsigned U1RXIF :1;
        unsigned U1TXIF :1;
        unsigned ADIF   :1;
        unsigned NVMIF  :1;
        unsigned SI2CIF  :1;
        unsigned MI2CIF  :1;
        unsigned CNIF   :1;
} IFS0BITS;
extern volatile IFS0BITS IFS0bits __attribute__((__near__));

/* IFS1: Interrupt Flag Status Register 1 */
extern volatile unsigned int IFS1 __attribute__((__near__));
typedef struct tagIFS1BITS {
        unsigned INT1IF :1;
        unsigned IC7IF  :1;
        unsigned IC8IF  :1;
        unsigned        :4;
        unsigned INT2IF :1;
        unsigned        :8;
} IFS1BITS;
extern volatile IFS1BITS IFS1bits __attribute__((__near__));

/* IFS2: Interrupt Flag Status Register 2 */
extern volatile unsigned int IFS2 __attribute__((__near__));
typedef struct tagIFS2BITS {
        unsigned        :7;
        unsigned PWMIF  :1;
        unsigned QEIIF  :1;
        unsigned        :1;
        unsigned LVDIF  :1;
        unsigned FLTAIF :1;
        unsigned FLTBIF :1;
        unsigned        :3;
} IFS2BITS;
extern volatile IFS2BITS IFS2bits __attribute__((__near__));

/* IEC0: Interrupt Enable Control Register 0 */
extern volatile unsigned int IEC0 __attribute__((__near__));
typedef struct tagIEC0BITS {
        unsigned INT0IE :1;
        unsigned IC1IE  :1;
        unsigned OC1IE  :1;
        unsigned T1IE   :1;
        unsigned IC2IE  :1;
        unsigned OC2IE  :1;
        unsigned T2IE   :1;
        unsigned T3IE   :1;
        unsigned SPI1IE :1;
        unsigned U1RXIE :1;
        unsigned U1TXIE :1;
        unsigned ADIE   :1;
        unsigned NVMIE  :1;
        unsigned SI2CIE  :1;
        unsigned MI2CIE  :1;
        unsigned CNIE   :1;
} IEC0BITS;
extern volatile IEC0BITS IEC0bits __attribute__((__near__));

/* IEC1: Interrupt Enable Control Register 1 */
extern volatile unsigned int IEC1 __attribute__((__near__));
typedef struct tagIEC1BITS {
        unsigned INT1IE :1;
        unsigned IC7IE  :1;
        unsigned IC8IE  :1;
        unsigned        :4;
        unsigned INT2IE :1;
        unsigned        :8;
} IEC1BITS;
extern volatile IEC1BITS IEC1bits __attribute__((__near__));

/* IEC2: Interrupt Enable Control Register 2 */
extern volatile unsigned int IEC2 __attribute__((__near__));
typedef struct tagIEC2BITS {
        unsigned        :7;
        unsigned PWMIE  :1;
        unsigned QEIIE  :1;
        unsigned        :1;
        unsigned LVDIE  :1;
        unsigned FLTAIE :1;
        unsigned FLTBIE :1;
        unsigned        :3;
} IEC2BITS;
extern volatile IEC2BITS IEC2bits __attribute__((__near__));

/* IPC0: Interrupt Priority Control Register 0 */
extern volatile unsigned int IPC0 __attribute__((__near__));
typedef struct tagIPC0BITS {
        unsigned INT0IP :3;
        unsigned        :1;
        unsigned IC1IP  :3;
        unsigned        :1;
        unsigned OC1IP  :3;
        unsigned        :1;
        unsigned T1IP   :3;
        unsigned        :1;
} IPC0BITS;
extern volatile IPC0BITS IPC0bits __attribute__((__near__));

/* IPC1: Interrupt Priority Control Register 1 */
extern volatile unsigned int IPC1 __attribute__((__near__));
typedef struct tagIPC1BITS {
        unsigned IC2IP  :3;
        unsigned        :1;
        unsigned OC2IP  :3;
        unsigned        :1;
        unsigned T2IP   :3;
        unsigned        :1;
        unsigned T3IP   :3;
        unsigned        :1;
} IPC1BITS;
extern volatile IPC1BITS IPC1bits __attribute__((__near__));

/* IPC2: Interrupt Priority Control Register 2 */
extern volatile unsigned int IPC2 __attribute__((__near__));
typedef struct tagIPC2BITS {
        unsigned SPI1IP :3;
        unsigned        :1;
        unsigned U1RXIP :3;
        unsigned        :1;
        unsigned U1TXIP :3;
        unsigned        :1;
        unsigned ADIP   :3;
        unsigned        :1;
} IPC2BITS;
extern volatile IPC2BITS IPC2bits __attribute__((__near__));

/* IPC3: Interrupt Priority Control Register 3 */
extern volatile unsigned int IPC3 __attribute__((__near__));
typedef struct tagIPC3BITS {
        unsigned NVMIP  :3;
        unsigned        :1;
        unsigned SI2CIP  :3;
        unsigned        :1;
        unsigned MI2CIP  :3;
        unsigned        :1;
        unsigned CNIP   :3;
        unsigned        :1;
} IPC3BITS;
extern volatile IPC3BITS IPC3bits __attribute__((__near__));

/* IPC4: Interrupt Priority Control Register 4 */
extern volatile unsigned int IPC4 __attribute__((__near__));
typedef struct tagIPC4BITS {
        unsigned INT1IP :3;
        unsigned        :1;
        unsigned IC7IP  :3;
        unsigned        :1;
        unsigned IC8IP  :3;
        unsigned        :5;
} IPC4BITS;
extern volatile IPC4BITS IPC4bits __attribute__((__near__));

/* IPC5: Interrupt Priority Control Register 5 */
extern volatile unsigned int IPC5 __attribute__((__near__));
typedef struct tagIPC5BITS {
        unsigned        :12;
        unsigned INT2IP :3;
        unsigned        :1;
} IPC5BITS;
extern volatile IPC5BITS IPC5bits __attribute__((__near__));

/* IPC9: Interrupt Priority Control Register 9 */
extern volatile unsigned int IPC9 __attribute__((__near__));
typedef struct tagIPC9BITS {
        unsigned        :12;
        unsigned PWMIP  :3;
        unsigned        :1;
} IPC9BITS;
extern volatile IPC9BITS IPC9bits __attribute__((__near__));

/* IPC10: Interrupt Priority Control Register 10 */
extern volatile unsigned int IPC10 __attribute__((__near__));
typedef struct tagIPC10BITS {
        unsigned QEIIP  :3;
        unsigned        :5;
        unsigned LVDIP  :3;
        unsigned        :1;
        unsigned FLTAIP :3;
        unsigned        :1;
} IPC10BITS;
extern volatile IPC10BITS IPC10bits __attribute__((__near__));

/* IPC11: Interrupt Priority Control Register 11 */
extern volatile unsigned int IPC11 __attribute__((__near__));
typedef struct tagIPC11BITS {
        unsigned FLTBIP :3;
        unsigned        :13;
} IPC11BITS;
extern volatile IPC11BITS IPC11bits __attribute__((__near__));

/* ---------------------------------------------- */
/* Input Change Notification register definitions */
/* ---------------------------------------------- */

/* CNEN1: Input Change Notification Interrupt Enable Register 1 */
extern volatile unsigned int CNEN1 __attribute__((__near__));
typedef struct tagCNEN1BITS {
        unsigned CN0IE  :1;
        unsigned CN1IE  :1;
        unsigned CN2IE  :1;
        unsigned CN3IE  :1;
        unsigned CN4IE  :1;
        unsigned CN5IE  :1;
        unsigned CN6IE  :1;
        unsigned CN7IE  :1;
        unsigned        :8;
} CNEN1BITS;
extern volatile CNEN1BITS CNEN1bits __attribute__((__near__));

/* CNPU1: Input Change Notification Pullup Enable Register 1 */
extern volatile unsigned int CNPU1 __attribute__((__near__));
typedef struct tagCNPU1BITS {
        unsigned CN0PUE :1;
        unsigned CN1PUE :1;
        unsigned CN2PUE :1;
        unsigned CN3PUE :1;
        unsigned CN4PUE :1;
        unsigned CN5PUE :1;
        unsigned CN6PUE :1;
        unsigned CN7PUE :1;
        unsigned        :8;
} CNPU1BITS;
extern volatile CNPU1BITS CNPU1bits __attribute__((__near__));


/* --------------------------- */
/* Timer1 register definitions */
/* --------------------------- */

/* Generic structure for Timer 1 Control Register */
typedef struct tagTCON_16BIT {
        unsigned        :1;
        unsigned TCS    :1;
        unsigned TSYNC  :1;
        unsigned        :1;
        unsigned TCKPS  :2;
        unsigned TGATE  :1;
        unsigned        :6;
        unsigned TSIDL  :1;
        unsigned        :1;
        unsigned TON    :1;
} TCON_16BIT;

/* TMR1: Timer 1 Count Register */
extern volatile unsigned int TMR1 __attribute__((__near__));

/* PR1: Timer 1 Period Register */
extern volatile unsigned int PR1 __attribute__((__near__));

/* T1CON: Timer 1 Control Register */
extern volatile unsigned int T1CON __attribute__((__near__));
extern volatile TCON_16BIT T1CONbits __attribute__((__near__));

/* ----------------------------- */
/* Timer2/3 register definitions */
/* ----------------------------- */

/* Generic structure for Timer 2 and Timer 4 Control Registers */
typedef struct tagTCON_EVEN {
        unsigned        :1;
        unsigned TCS    :1;
        unsigned        :1;
        unsigned T32    :1;
        unsigned TCKPS  :2;
        unsigned TGATE  :1;
        unsigned        :6;
        unsigned TSIDL  :1;
        unsigned        :1;
        unsigned TON    :1;
} TCON_EVEN;

/* Generic structure for Timer 3 and Timer 5 Control Registers */
typedef struct tagTCON_ODD {
        unsigned        :1;
        unsigned TCS    :1;
        unsigned        :2;
        unsigned TCKPS  :2;
        unsigned TGATE  :1;
        unsigned        :6;
        unsigned TSIDL  :1;
        unsigned        :1;
        unsigned TON    :1;
} TCON_ODD;

/* TMR2: Timer 2 Count Register */
extern volatile unsigned int TMR2 __attribute__((__near__));

/* TMR3HLD: Timer 3 Holding Register */
extern volatile unsigned int TMR3HLD __attribute__((__near__));

/* TMR3: Timer 3 Count Register */
extern volatile unsigned int TMR3 __attribute__((__near__));

/* PR2: Timer 2 Period Register */
extern volatile unsigned int PR2 __attribute__((__near__));

/* PR3: Timer 3 Period Register */
extern volatile unsigned int PR3 __attribute__((__near__));

/* T2CON: Timer 2 Control Register */
extern volatile unsigned int T2CON __attribute__((__near__));
extern volatile TCON_EVEN T2CONbits __attribute__((__near__));

/* T3CON: Timer 3 Control Register */
extern volatile unsigned int T3CON __attribute__((__near__));
extern volatile TCON_ODD T3CONbits __attribute__((__near__));


/* ------------------------------------------------- */
/* Quadrature Encoder Interface register definitions */
/* ------------------------------------------------- */

/* QEICON: QEI Control Register */
extern volatile unsigned int QEICON __attribute__((__near__));
typedef struct tagQEICONBITS {
        unsigned UPDN_SRC:1;
        unsigned TQCS   :1;
        unsigned POSRES :1;
        unsigned TQCKPS :2;
        unsigned TQGATE :1;
        unsigned PCDOUT :1;
        unsigned SWPAB  :1;
        unsigned QEIM   :3;
        unsigned UPDN   :1;
        unsigned INDX   :1;
        unsigned QEISIDL:1;
        unsigned        :1;
        unsigned CNTERR :1;
} QEICONBITS;
extern volatile QEICONBITS QEICONbits __attribute__((__near__));

/* DFLTCON: Digital Filter Control Register */
extern volatile unsigned int DFLTCON __attribute__((__near__));
typedef struct tagDFLTCONBITS {
        unsigned        :4;
        unsigned QECK   :3;
        unsigned QEOUT  :1;
        unsigned CEID   :1;
        unsigned IMV    :2;
        unsigned        :5;
} DFLTCONBITS;
extern volatile DFLTCONBITS DFLTCONbits __attribute__((__near__));

/* POSCNT: Position Counter Register */
extern volatile unsigned int POSCNT __attribute__((__near__));

/* MAXCNT: Maximum Count Register */
extern volatile unsigned int MAXCNT __attribute__((__near__));

/* ---------------------------------- */
/* Input Capture register definitions */
/* ---------------------------------- */

/* Generic structure of entire SFR area for each Input Capture module */
typedef struct tagIC {
        unsigned int icxbuf;
        unsigned int icxcon;
} IC, *PIC;

/* SFR blocks for each Input Capture module */
extern volatile IC IC1 __attribute__((__near__));
extern volatile IC IC2 __attribute__((__near__));
extern volatile IC IC7 __attribute__((__near__));
extern volatile IC IC8 __attribute__((__near__));

/* Generic structure for Input Capture Control Registers */
typedef struct tagICxCONBITS {
        unsigned ICM    :3;
        unsigned ICBNE  :1;
        unsigned ICOV   :1;
        unsigned ICI    :2;
        unsigned ICTMR  :1;
        unsigned        :5;
        unsigned ICSIDL :1;
        unsigned        :2;
} ICxCONBITS;

/* IC1BUF: Input Capture 1 Buffer */
extern volatile unsigned int IC1BUF __attribute__((__near__));

/* IC1CON: Input Capture 1 Control Register */
extern volatile unsigned int IC1CON __attribute__((__near__));
extern volatile ICxCONBITS IC1CONbits __attribute__((__near__));

/* IC2BUF: Input Capture 2 Buffer */
extern volatile unsigned int IC2BUF __attribute__((__near__));

/* IC2CON: Input Capture 2 Control Register */
extern volatile unsigned int IC2CON __attribute__((__near__));
extern volatile ICxCONBITS IC2CONbits __attribute__((__near__));

/* IC7BUF: Input Capture 7 Buffer */
extern volatile unsigned int IC7BUF __attribute__((__near__));

/* IC7CON: Input Capture 7 Control Register */
extern volatile unsigned int IC7CON __attribute__((__near__));
extern volatile ICxCONBITS IC7CONbits __attribute__((__near__));

/* IC8BUF: Input Capture 8 Buffer */
extern volatile unsigned int IC8BUF __attribute__((__near__));

/* IC8CON: Input Capture 8 Control Register */
extern volatile unsigned int IC8CON __attribute__((__near__));
extern volatile ICxCONBITS IC8CONbits __attribute__((__near__));


/* --------------------------------------- */
/* Output Compare/PWM register definitions */
/* --------------------------------------- */

/* Generic structure of entire SFR area for each Output Compare module */
typedef struct tagOC {
        unsigned int ocxrs;
        unsigned int ocxr;
        unsigned int ocxcon;
} OC, *POC;

/* SFR blocks for each Output Compare module */
extern volatile OC OC1 __attribute__((__near__));
extern volatile OC OC2 __attribute__((__near__));

/* Generic structure for Output Compare Control Registers */
typedef struct tagOCxCONBITS {
        unsigned OCM    :3;
        unsigned OCTSEL :1;
        unsigned OCFLT  :1;
        unsigned        :8;
        unsigned OCSIDL :1;
        unsigned        :2;
} OCxCONBITS;

/* OC1RS: Output Compare 1 Secondary Register */
extern volatile unsigned int OC1RS __attribute__((__near__));

/* OC1R: Output Compare 1 Main Register */
extern volatile unsigned int OC1R __attribute__((__near__));

/* OC1CON: Output Compare 1 Control Register */
extern volatile unsigned int OC1CON __attribute__((__near__));
extern volatile OCxCONBITS OC1CONbits __attribute__((__near__));

/* OC2RS: Output Compare 2 Secondary Register */
extern volatile unsigned int OC2RS __attribute__((__near__));

/* OC2R: Output Compare 2 Main Register */
extern volatile unsigned int OC2R __attribute__((__near__));

/* OC2CON: Output Compare 2 Control Register */
extern volatile unsigned int OC2CON __attribute__((__near__));
extern volatile OCxCONBITS OC2CONbits __attribute__((__near__));


/* -------------------------------------- */
/* Motor Control PWM register definitions */
/* -------------------------------------- */

/* PTCON: PWM Timerbase Control Register */
extern volatile unsigned int PTCON __attribute__((__near__));
typedef struct tagPTCONBITS {
        unsigned PTMOD  :2;
        unsigned PTCKPS :2;
        unsigned PTOPS  :4;
        unsigned        :5;
        unsigned PTSIDL :1;
        unsigned        :1;
        unsigned PTEN   :1;
} PTCONBITS;
extern volatile PTCONBITS PTCONbits __attribute__((__near__));

/* PTMR: PWM Timebase Count Register */
extern volatile unsigned int PTMR __attribute__((__near__));
typedef struct tagPTMRBITS {
        unsigned PTMR   :15;
        unsigned PTDIR  :1;
} PTMRBITS;
extern volatile PTMRBITS PTMRbits __attribute__((__near__));

/* PTPER: PWM Timebase Period Register */
extern volatile unsigned int PTPER __attribute__((__near__));
typedef struct tagPTPERBITS {
        unsigned PTPER  :15;
        unsigned        :1;
} PTPERBITS;
extern volatile PTPERBITS PTPERbits __attribute__((__near__));

/* SEVTCMP: Special Event Compare Count Register */
extern volatile unsigned int SEVTCMP __attribute__((__near__));
typedef struct tagSEVTCMPBITS {
        unsigned SEVTCMP:15;
        unsigned SEVTDIR:1;
} SEVTCMPBITS;
extern volatile SEVTCMPBITS SEVTCMPbits __attribute__((__near__));

/* PWMCON1: PWM Control Register 1 */
extern volatile unsigned int PWMCON1 __attribute__((__near__));
typedef struct tagPWMCON1BITS {
        unsigned PEN1L  :1;
        unsigned PEN2L  :1;
        unsigned PEN3L  :1;
        unsigned        :1;
        unsigned PEN1H  :1;
        unsigned PEN2H  :1;
        unsigned PEN3H  :1;
        unsigned        :1;
        unsigned PMOD1  :1;
        unsigned PMOD2  :1;
        unsigned PMOD3  :1;
        unsigned        :5;
} PWMCON1BITS;
extern volatile PWMCON1BITS PWMCON1bits __attribute__((__near__));

/* PWMCON2: PWM Control Register 2 */
extern volatile unsigned int PWMCON2 __attribute__((__near__));
typedef struct tagPWMCON2BITS {
        unsigned UDIS   :1;
        unsigned OSYNC  :1;
        unsigned        :6;
        unsigned SEVOPS :4;
        unsigned        :4;
} PWMCON2BITS;
extern volatile PWMCON2BITS PWMCON2bits __attribute__((__near__));

/* DTCON1: PWM Dead Time Control Register 1 */
extern volatile unsigned int DTCON1 __attribute__((__near__));
typedef struct tagDTCON1BITS {
        unsigned DTA    :6;
        unsigned DTAPS  :2;
        unsigned        :8;
} DTCON1BITS;
extern volatile DTCON1BITS DTCON1bits __attribute__((__near__));

/* FLTACON: PWM Fault Input A Control Register */ 
extern volatile unsigned int FLTACON __attribute__((__near__));
typedef struct tagFLTACONBITS {
        unsigned FAEN1  :1;
        unsigned FAEN2  :1;
        unsigned FAEN3  :1;
        unsigned        :4;
        unsigned FLTAM  :1;
        unsigned FAOV1L :1;
        unsigned FAOV1H :1;
        unsigned FAOV2L :1;
        unsigned FAOV2H :1;
        unsigned FAOV3L :1;
        unsigned FAOV3H :1;
        unsigned        :2;
} FLTACONBITS;
extern volatile FLTACONBITS FLTACONbits __attribute__((__near__));

/* OVDCON: PWM Override Control Register */
extern volatile unsigned int OVDCON __attribute__((__near__));
typedef struct tagOVDCONBITS {
        unsigned POUT1L :1;
        unsigned POUT1H :1;
        unsigned POUT2L :1;
        unsigned POUT2H :1;
        unsigned POUT3L :1;
        unsigned POUT3H :1;
        unsigned        :2;
        unsigned POVD1L :1;
        unsigned POVD1H :1;
        unsigned POVD2L :1;
        unsigned POVD2H :1;
        unsigned POVD3L :1;
        unsigned POVD3H :1;
        unsigned        :2;
} OVDCONBITS;
extern volatile OVDCONBITS OVDCONbits __attribute__((__near__));

/* PDC1: PWM Duty Cycle Register 1 */
extern volatile unsigned int PDC1 __attribute__((__near__));

/* PDC2: PWM Duty Cycle Register 2 */
extern volatile unsigned int PDC2 __attribute__((__near__));

/* PDC3: PWM Duty Cycle Register 3 */
extern volatile unsigned int PDC3 __attribute__((__near__));


/* ------------------------ */
/* I2C register definitions */
/* ------------------------ */

/* I2CRCV: I2C Receive Register */
extern volatile unsigned char I2CRCV __attribute__((__near__));
typedef struct tagI2CRCVBITS {
        unsigned I2CRCV0:1;
        unsigned I2CRCV1:1;
        unsigned I2CRCV2:1;
        unsigned I2CRCV3:1;
        unsigned I2CRCV4:1;
        unsigned I2CRCV5:1;
        unsigned I2CRCV6:1;
        unsigned I2CRCV7:1;
} I2CRCVBITS;
extern volatile I2CRCVBITS I2CRCVbits __attribute__((__near__));

/* I2CTRN: I2C Transmit Register */
extern volatile unsigned char I2CTRN __attribute__((__near__));
typedef struct tagI2CTRNBITS {
        unsigned I2CTRN0:1;
        unsigned I2CTRN1:1;
        unsigned I2CTRN2:1;
        unsigned I2CTRN3:1;
        unsigned I2CTRN4:1;
        unsigned I2CTRN5:1;
        unsigned I2CTRN6:1;
        unsigned I2CTRN7:1;
} I2CTRNBITS;
extern volatile I2CTRNBITS I2CTRNbits __attribute__((__near__));

/* I2CBRG: I2C Baud Rate Generator Register */
extern volatile unsigned int I2CBRG __attribute__((__near__));
typedef struct tagI2CBRGBITS {
        unsigned I2CBRG :9;
        unsigned        :7;
} I2CBRGBITS;
extern volatile I2CBRGBITS I2CBRGbits __attribute__((__near__));

/* I2CCON: I2C Control Register */
extern volatile unsigned int I2CCON __attribute__((__near__));
typedef struct tagI2CCONBITS {
        unsigned SEN    :1;
        unsigned RSEN   :1;
        unsigned PEN    :1;
        unsigned RCEN   :1;
        unsigned ACKEN  :1;
        unsigned ACKDT  :1;
        unsigned STREN  :1;
        unsigned GCEN   :1;
        unsigned SMEN   :1;
        unsigned DISSLW :1;
        unsigned A10M  :1;
        unsigned IPMIEN :1;
        unsigned SCLREL :1;
        unsigned I2CSIDL:1;
        unsigned        :1;
        unsigned I2CEN  :1;
} I2CCONBITS;
extern volatile I2CCONBITS I2CCONbits __attribute__((__near__));

/* I2CSTAT: I2C Status Register */
extern volatile unsigned int I2CSTAT __attribute__((__near__));
typedef struct tagI2CSTATBITS {
        unsigned TBF    :1;
        unsigned RBF    :1;
        unsigned R_W    :1;
        unsigned S      :1;
        unsigned P      :1;
        unsigned D_A    :1;
        unsigned I2COV  :1;
        unsigned IWCOL  :1;
        unsigned ADD10  :1;
        unsigned GCSTAT :1;
        unsigned BCL    :1;
        unsigned        :3;
        unsigned TRSTAT :1;
        unsigned ACKSTAT:1;
} I2CSTATBITS;
extern volatile I2CSTATBITS I2CSTATbits __attribute__((__near__));

/* I2CADD: I2C Address Register */
extern volatile unsigned int I2CADD __attribute__((__near__));
typedef struct tagI2CADDBITS {
        unsigned I2CADD :10;
        unsigned        :6;
} I2CADDBITS;
extern volatile I2CADDBITS I2CADDbits __attribute__((__near__));

/* -------------------------- */
/* UART1 register definitions */
/* -------------------------- */

/* Generic structure of entire SFR area for each UART module */
typedef struct tagUART {
        unsigned int uxmode;
        unsigned int uxsta;
        unsigned int uxtxreg;
        unsigned int uxrxreg;
        unsigned int uxbrg;
} UART, *PUART;

/* SFR blocks for each UART module */
extern volatile UART UART1 __attribute__((__near__));

/* Generic structure for UART Mode Registers */
typedef struct tagUxMODEBITS {
        unsigned STSEL  :1;
        unsigned PDSEL  :2;
        unsigned        :2;
        unsigned ABAUD  :1;
        unsigned LPBACK :1;
        unsigned WAKE   :1;
        unsigned        :2;
        unsigned ALTIO  :1;
        unsigned        :2;
        unsigned USIDL  :1;
        unsigned        :1;
        unsigned UARTEN :1;
} UxMODEBITS;

/* Generic structure for UART Status and Control Registers */
typedef struct tagUxSTABITS {
        unsigned URXDA  :1;
        unsigned OERR   :1;
        unsigned FERR   :1;
        unsigned PERR   :1;
        unsigned RIDLE  :1;
        unsigned ADDEN  :1;
        unsigned URXISEL:2;
        unsigned TRMT   :1;
        unsigned UTXBF  :1;
        unsigned UTXEN  :1;
        unsigned UTXBRK :1;
        unsigned        :3;
        unsigned UTXISEL:1;
} UxSTABITS; 

/* Generic structure for UART Transmit Registers */
typedef struct tagUxTXREGBITS {
        unsigned UTXREG0:1;
        unsigned UTXREG1:1;
        unsigned UTXREG2:1;
        unsigned UTXREG3:1;
        unsigned UTXREG4:1;
        unsigned UTXREG5:1;
        unsigned UTXREG6:1;
        unsigned UTXREG7:1;
        unsigned UTX8   :1;
        unsigned        :7;
} UxTXREGBITS;

/* Generic structure for UART Receive Registers */
typedef struct tagUxRXREGBITS {
        unsigned URXREG0:1;
        unsigned URXREG1:1;
        unsigned URXREG2:1;
        unsigned URXREG3:1;
        unsigned URXREG4:1;
        unsigned URXREG5:1;
        unsigned URXREG6:1;
        unsigned URXREG7:1;
        unsigned URX8   :1;
        unsigned        :7;
} UxRXREGBITS;

/* U1MODE: UART1 Mode Regsiter */
extern volatile unsigned int U1MODE __attribute__((__near__));
extern volatile UxMODEBITS U1MODEbits __attribute__((__near__));

/* U1STA: UART1 Status and Control Register */
extern volatile unsigned int U1STA __attribute__((__near__));
extern volatile UxSTABITS U1STAbits __attribute__((__near__));

/* U1TXREG: UART1 Transmit Register */
extern volatile unsigned int U1TXREG __attribute__((__near__));
extern volatile UxTXREGBITS U1TXREGbits __attribute__((__near__));

/* U1RXREG: UART1 Receive Register */
extern volatile unsigned int U1RXREG __attribute__((__near__));
extern volatile UxRXREGBITS U1RXREGbits __attribute__((__near__));   

/* U1BRG: UART1 Baud Rate Generator Register */
extern volatile unsigned int U1BRG __attribute__((__near__));


/* ------------------------- */
/* SPI1 register definitions */
/* ------------------------- */

/* Generic structure of entire SFR area for each SPI module */
typedef struct tagSPI {
        unsigned int spixstat;
        unsigned int spixcon;
        unsigned int spixbuf;
} SPI, *PSPI;

/* SFR blocks for each SPI module */
extern volatile SPI SPI1 __attribute__((__near__));

/* Generic structure for SPI Status Registers */
typedef struct tagSPIxSTATBITS {
        unsigned SPIRBF :1;
        unsigned SPITBF :1;
        unsigned        :4;
        unsigned SPIROV :1;
        unsigned        :6;
        unsigned SPISIDL:1;
        unsigned        :1;
        unsigned SPIEN  :1;
} SPIxSTATBITS;

/* Generic structure for SPI Control Registers */
typedef struct tagSPIxCONBITS {
        unsigned PPRE   :2;
        unsigned SPRE   :3;
        unsigned MSTEN  :1;
        unsigned CKP    :1;
        unsigned SSEN   :1;
        unsigned CKE    :1;
        unsigned SMP    :1;
        unsigned MODE16 :1;
        unsigned DISSDO :1;
        unsigned        :1;
        unsigned SPIFSD:1;
        unsigned FRMEN  :1;
        unsigned        :1;
} SPIxCONBITS;

/* SPI1STAT: SPI1 Status Register */
extern volatile unsigned int SPI1STAT __attribute__((__near__));
extern volatile SPIxSTATBITS SPI1STATbits __attribute__((__near__));

/* SPI1CON: SPI1 Control Register */
extern volatile unsigned int SPI1CON __attribute__((__near__));
extern volatile SPIxCONBITS SPI1CONbits __attribute__((__near__));

/* SPI1BUF: SPI1 Buffer */
extern volatile unsigned int SPI1BUF __attribute__((__near__));


/* ------------------------------------------------------------------ */
/* 10-bit (500 ksps) Analog-to-Digital Converter register definitions */
/* ------------------------------------------------------------------ */

/* ADC Buffers 0-F */
extern volatile unsigned int ADCBUF0 __attribute__((__near__));
extern volatile unsigned int ADCBUF1 __attribute__((__near__));
extern volatile unsigned int ADCBUF2 __attribute__((__near__));
extern volatile unsigned int ADCBUF3 __attribute__((__near__));
extern volatile unsigned int ADCBUF4 __attribute__((__near__));
extern volatile unsigned int ADCBUF5 __attribute__((__near__));
extern volatile unsigned int ADCBUF6 __attribute__((__near__));
extern volatile unsigned int ADCBUF7 __attribute__((__near__));
extern volatile unsigned int ADCBUF8 __attribute__((__near__));
extern volatile unsigned int ADCBUF9 __attribute__((__near__));
extern volatile unsigned int ADCBUFA __attribute__((__near__));
extern volatile unsigned int ADCBUFB __attribute__((__near__));
extern volatile unsigned int ADCBUFC __attribute__((__near__));
extern volatile unsigned int ADCBUFD __attribute__((__near__));
extern volatile unsigned int ADCBUFE __attribute__((__near__));
extern volatile unsigned int ADCBUFF __attribute__((__near__));

/* ADCON1: ADC Control Register 1 */
extern volatile unsigned int ADCON1 __attribute__((__near__));
typedef struct tagADCON1BITS {
        unsigned DONE   :1;
        unsigned SAMP   :1;
        unsigned ASAM   :1;
        unsigned SIMSAM :1;
        unsigned        :1;
        unsigned SSRC   :3;
        unsigned FORM   :2;
        unsigned        :3;
        unsigned ADSIDL :1;
        unsigned        :1;
        unsigned ADON   :1;
} ADCON1BITS;
extern volatile ADCON1BITS ADCON1bits __attribute__((__near__));

/* ADCON2: ADC Control Register 2 */
extern volatile unsigned int ADCON2 __attribute__((__near__));
typedef struct tagADCON2BITS {
        unsigned ALTS   :1;
        unsigned BUFM   :1;
        unsigned SMPI   :4;
        unsigned        :1;
        unsigned BUFS   :1;
        unsigned CHPS   :2;
        unsigned CSCNA  :1;
        unsigned        :2;
        unsigned VCFG   :3;
} ADCON2BITS;
extern volatile ADCON2BITS ADCON2bits __attribute__((__near__));

/* ADCON3: ADC Control Register 3 */
extern volatile unsigned int ADCON3 __attribute__((__near__));
typedef struct tagADCON3BITS {
        unsigned ADCS   :6;
        unsigned        :1;
        unsigned ADRC   :1;
        unsigned SAMC   :5;
        unsigned        :3;
} ADCON3BITS;
extern volatile ADCON3BITS ADCON3bits __attribute__((__near__));

/* ADCHS: ADC Input Channel Select Register */
extern volatile unsigned int ADCHS __attribute__((__near__));
typedef struct tagADCHSBITS {
        unsigned CH0SA  :4;
        unsigned CH0NA  :1;
        unsigned CH123SA:1;
        unsigned CH123NA:2;
        unsigned CH0SB  :4;
        unsigned CH0NB  :1;
        unsigned CH123SB:1;
        unsigned CH123NB:2;
} ADCHSBITS;
extern volatile ADCHSBITS ADCHSbits __attribute__((__near__));

/* ADPCFG: ADC Port Configuration Register */
extern volatile unsigned int ADPCFG __attribute__((__near__));
typedef struct tagADPCFGBITS {
        unsigned PCFG0  :1;
        unsigned PCFG1  :1;
        unsigned PCFG2  :1;
        unsigned PCFG3  :1;
        unsigned PCFG4  :1;
        unsigned PCFG5  :1;
        unsigned        :10;
} ADPCFGBITS;
extern volatile ADPCFGBITS ADPCFGbits __attribute__((__near__));

/* ADCSSL: ADC Input Scan Select Register */
extern volatile unsigned int ADCSSL __attribute__((__near__));
typedef struct tagADCSSLBITS {
        unsigned CSSL0  :1;
        unsigned CSSL1  :1;
        unsigned CSSL2  :1;
        unsigned CSSL3  :1;
        unsigned CSSL4  :1;
        unsigned CSSL5  :1;
        unsigned        :10;
} ADCSSLBITS;
extern volatile ADCSSLBITS ADCSSLbits __attribute__((__near__));


/* ------------------------------ */
/* I/O Ports register definitions */
/* ------------------------------ */

/* TRISB: Port B Direction Control Register */
extern volatile unsigned int TRISB __attribute__((__near__));
typedef struct tagTRISBBITS {
        unsigned TRISB0 :1;
        unsigned TRISB1 :1;
        unsigned TRISB2 :1;
        unsigned TRISB3 :1;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned        :10;
} TRISBBITS;
extern volatile TRISBBITS TRISBbits __attribute__((__near__));

/* PORTB: Port B Pin Register */
extern volatile unsigned int PORTB __attribute__((__near__));
typedef struct tagPORTBBITS {
        unsigned RB0    :1;
        unsigned RB1    :1;
        unsigned RB2    :1;
        unsigned RB3    :1;
        unsigned RB4    :1;
        unsigned RB5    :1;
        unsigned        :10;
} PORTBBITS;
extern volatile PORTBBITS PORTBbits __attribute__((__near__));

/* LATB: Port B Latch Register */
extern volatile unsigned int LATB __attribute__((__near__));
typedef struct tagLATBBITS {
        unsigned LATB0  :1;
        unsigned LATB1  :1;
        unsigned LATB2  :1;
        unsigned LATB3  :1;
        unsigned LATB4  :1;
        unsigned LATB5  :1;
        unsigned        :10;
} LATBBITS;
extern volatile LATBBITS LATBbits __attribute__((__near__));

/* TRISC: Port C Direction Control Register */
extern volatile unsigned int TRISC __attribute__((__near__));
typedef struct tagTRISCBITS {
        unsigned        :13;
        unsigned TRISC13:1;
        unsigned TRISC14:1;
        unsigned TRISC15:1;
} TRISCBITS;
extern volatile TRISCBITS TRISCbits __attribute__((__near__));

/* PORTC: Port C Pin Register */
extern volatile unsigned int PORTC __attribute__((__near__));
typedef struct tagPORTCBITS {
        unsigned        :13;
        unsigned RC13   :1;
        unsigned RC14   :1;
        unsigned RC15   :1;
} PORTCBITS;
extern volatile PORTCBITS PORTCbits __attribute__((__near__));

/* LATC: Port C Latch Register */
extern volatile unsigned int LATC __attribute__((__near__));
typedef struct tagLATCBITS {
        unsigned        :13;
        unsigned LATC13 :1;
        unsigned LATC14 :1;
        unsigned LATC15 :1;
} LATCBITS;
extern volatile LATCBITS LATCbits __attribute__((__near__));

/* TRISD: Port D Direction Control Register */
extern volatile unsigned int TRISD __attribute__((__near__));
typedef struct tagTRISDBITS {
        unsigned TRISD0 :1;
        unsigned TRISD1 :1;
        unsigned        :14;
} TRISDBITS;
extern volatile TRISDBITS TRISDbits __attribute__((__near__));

/* PORTD: Port D Pin Register */
extern volatile unsigned int PORTD __attribute__((__near__));
typedef struct tagPORTDBITS {
        unsigned RD0    :1;
        unsigned RD1    :1;
        unsigned        :14;
} PORTDBITS;
extern volatile PORTDBITS PORTDbits __attribute__((__near__));

/* LATD: Port D Latch Register */
extern volatile unsigned int LATD __attribute__((__near__));
typedef struct tagLATDBITS {
        unsigned LATD0  :1;
        unsigned LATD1  :1;
        unsigned        :14;
} LATDBITS;
extern volatile LATDBITS LATDbits __attribute__((__near__));

/* TRISE: Port E Direction Control Register */
extern volatile unsigned int TRISE __attribute__((__near__));
typedef struct tagTRISEBITS {
        unsigned TRISE0 :1;
        unsigned TRISE1 :1;
        unsigned TRISE2 :1;
        unsigned TRISE3 :1;
        unsigned TRISE4 :1;
        unsigned TRISE5 :1;
        unsigned        :2;
        unsigned TRISE8 :1;
        unsigned        :7;
} TRISEBITS;
extern volatile TRISEBITS TRISEbits __attribute__((__near__));

/* PORTE: Port E Pin Register */
extern volatile unsigned int PORTE __attribute__((__near__));
typedef struct tagPORTEBITS {
        unsigned RE0    :1;
        unsigned RE1    :1;
        unsigned RE2    :1;
        unsigned RE3    :1;
        unsigned RE4    :1;
        unsigned RE5    :1;
        unsigned        :2;
        unsigned RE8    :1;
        unsigned        :7;
} PORTEBITS;
extern volatile PORTEBITS PORTEbits __attribute__((__near__));

/* LATE: Port E Latch Register */
extern volatile unsigned int LATE __attribute__((__near__));
typedef struct tagLATEBITS {
        unsigned LATE0  :1;
        unsigned LATE1  :1;
        unsigned LATE2  :1;
        unsigned LATE3  :1;
        unsigned LATE4  :1;
        unsigned LATE5  :1;
        unsigned        :2;
        unsigned LATE8  :1;
        unsigned        :7;
} LATEBITS;
extern volatile LATEBITS LATEbits __attribute__((__near__));

/* TRISF: Port F Direction Control Register */
extern volatile unsigned int TRISF __attribute__((__near__));
typedef struct tagTRISFBITS {
        unsigned        :2;
        unsigned TRISF2 :1;
        unsigned TRISF3 :1;
        unsigned        :12;
} TRISFBITS;
extern volatile TRISFBITS TRISFbits __attribute__((__near__));

/* PORTF: Port F Pin Register */
extern volatile unsigned int PORTF __attribute__((__near__));
typedef struct tagPORTFBITS {
        unsigned        :2;
        unsigned RF2    :1;
        unsigned RF3    :1;
        unsigned        :12;
} PORTFBITS;
extern volatile PORTFBITS PORTFbits __attribute__((__near__));

/* LATF: Port F Latch Register */
extern volatile unsigned int LATF __attribute__((__near__));
typedef struct tagLATFBITS {
        unsigned        :2;
        unsigned LATF2  :1;
        unsigned LATF3  :1;
        unsigned        :12;
} LATFBITS;
extern volatile LATFBITS LATFbits __attribute__((__near__));


/* --------------------------------------- */
/* System Integration register definitions */
/* --------------------------------------- */

/* RCON: Reset Control Register */
extern volatile unsigned int RCON __attribute__((__near__));
typedef struct tagRCONBITS {
        unsigned POR    :1;
        unsigned BOR    :1;
        unsigned IDLE   :1;
        unsigned SLEEP  :1;
        unsigned WDTO   :1;
        unsigned SWDTEN :1;
        unsigned SWR    :1;
        unsigned EXTR   :1;
        unsigned LVDL   :4;
        unsigned LVDEN  :1;
        unsigned BGST   :1;
        unsigned IOPUWR :1;
        unsigned TRAPR  :1;
} RCONBITS;
extern volatile RCONBITS RCONbits __attribute__((__near__));

/* OSCCON: Oscillator Control Register */
extern volatile unsigned int OSCCON __attribute__((__near__));
typedef struct tagOSCCONBITS {
        unsigned OSWEN  :1;
        unsigned LPOSCEN:1;
        unsigned        :1;
        unsigned CF     :1;
        unsigned        :1;
        unsigned LOCK   :1;
        unsigned POST   :2;
        unsigned NOSC   :2;
        unsigned TUN0   :1;
        unsigned TUN1   :1;
        unsigned COSC   :2;
        unsigned TUN2   :1;
        unsigned TUN3   :1;
} OSCCONBITS;
extern volatile OSCCONBITS OSCCONbits __attribute__((__near__));

/* ---------------------------------------- */
/* Non-Volatile Memory register definitions */
/* ---------------------------------------- */

/* NVMCON: Non-Volatile Memory Control Register */
extern volatile unsigned int NVMCON __attribute__((__near__));
typedef struct tagNVMCONBITS {
        unsigned PROGOP :7;
        unsigned        :1;
        unsigned TWRI   :1;
        unsigned        :4;
        unsigned WRERR  :1;
        unsigned WREN   :1;
        unsigned WR     :1;
} NVMCONBITS;
extern volatile NVMCONBITS NVMCONbits __attribute__((__near__));

/* NVM Address bits <15:0> */
extern volatile unsigned int NVMADR __attribute__((__near__));

/* NVM Address bits <23:16> */
extern volatile unsigned char NVMADRU __attribute__((__near__));

/* NVM Key */
extern volatile unsigned char NVMKEY __attribute__((__near__));


/* ---------------------------------------------- */
/* Peripheral Module Disable register definitions */
/* ---------------------------------------------- */

/* PMD1: Peripheral Module Disable Register 1 */
extern volatile unsigned int PMD1 __attribute__((__near__));
typedef struct tagPMD1BITS {
        unsigned ADCMD  :1;
        unsigned        :2;
        unsigned SPI1MD :1;
        unsigned        :1;
        unsigned U1MD   :1;
        unsigned        :1;
        unsigned I2CMD  :1;
        unsigned        :1;
        unsigned PWMMD  :1;
        unsigned QEIMD  :1;
        unsigned T1MD   :1;
        unsigned T2MD   :1;
        unsigned T3MD   :1;
        unsigned        :2;
} PMD1BITS;
extern volatile PMD1BITS PMD1bits __attribute__((__near__));

/* PMD2: Peripheral Module Disable Register 3 */
extern volatile unsigned int PMD2 __attribute__((__near__));
typedef struct tagPMD2BITS {
        unsigned OC1MD  :1;
        unsigned OC2MD  :1;
        unsigned        :6;
        unsigned IC1MD  :1;
        unsigned IC2MD  :1;
        unsigned        :4;
        unsigned IC7MD  :1;
        unsigned IC8MD  :1;
} PMD2BITS;
extern volatile PMD2BITS PMD2bits __attribute__((__near__));


/* -------------------------------------------- */ 
/* Macros for Device Configuration Registers    */
/* -------------------------------------------- */

/* FOSC */
#define _FOSC(x) __attribute__((section("__FOSC.sec,x"))) int _FOSC = (x);

#define CSW_FSCM_OFF    0xFFFF
#define CSW_ON_FSCM_OFF 0x7FFF
#define CSW_FSCM_ON     0x3FFF
#define EC              0xFFFB
#define EC_IO           0xFFFC
#define EC_PLL4         0xFFFD
#define EC_PLL8         0xFFFE
#define EC_PLL16        0xFFFF
#define ERC             0xFFF9
#define ERC_IO          0xFFF8
#define XT              0xFFF4
#define XT_PLL4         0xFFF5
#define XT_PLL8         0xFFF6
#define XT_PLL16        0xFFF7
#define XTL             0xFFF0
#define HS              0xFFF2
#define LP              0xFCFF
#define FRC             0xFDFF
#define LPRC            0xFEFF

/* FWDT */
#define _FWDT(x) __attribute__((section("__FWDT.sec,x"))) int _FWDT = (x);

#define WDT_ON         0xFFFF
#define WDT_OFF        0x7FFF
#define WDTPSA_1       0xFFCF
#define WDTPSA_8       0xFFDF
#define WDTPSA_64      0xFFEF
#define WDTPSA_512     0xFFFF
#define WDTPSB_1       0xFFF0
#define WDTPSB_2       0xFFF1
#define WDTPSB_3       0xFFF2
#define WDTPSB_4       0xFFF3
#define WDTPSB_5       0xFFF4
#define WDTPSB_6       0xFFF5
#define WDTPSB_7       0xFFF6
#define WDTPSB_8       0xFFF7
#define WDTPSB_9       0xFFF8
#define WDTPSB_10      0xFFF9
#define WDTPSB_11      0xFFFA
#define WDTPSB_12      0xFFFB
#define WDTPSB_13      0xFFFC
#define WDTPSB_14      0xFFFD
#define WDTPSB_15      0xFFFE
#define WDTPSB_16      0xFFFF

/* FBORPOR */
#define _FBORPOR(x) __attribute__((section("__FBORPOR.sec,x"))) int _FBORPOR = (x);

#define MCLR_EN        0xFFFF
#define MCLR_DIS       0x7FFF
#define RST_IOPIN      0xFFFF
#define RST_PWMPIN     0xFBFF
#define PWMxH_ACT_HI   0xFFFF
#define PWMxH_ACT_LO   0xFDFF
#define PWMxL_ACT_HI   0xFFFF
#define PWMxL_ACT_LO   0xFEFF
#define PBOR_ON        0xFFFF
#define PBOR_OFF       0xFF7F
#define BORV_20        0xFFFF
#define BORV_27        0xFFEF
#define BORV_42        0xFFDF
#define BORV_45        0xFFCF
#define PWRT_OFF       0xFFFC
#define PWRT_4         0xFFFD
#define PWRT_16        0xFFFE
#define PWRT_64        0xFFFF

/* FGS */
#define _FGS(x) __attribute__((section("__FGS.sec,x"))) int _FGS = (x);

#define CODE_PROT_OFF  0xFFFF
#define CODE_PROT_ON   0xFFFD


/* --------------------------------------------------------------------------- 
 Setting configuration fuses using macros:
 ==========================================
 Macros are provided which can be used to set configuration fuses:
 For e.g., to set the FOSC fuse using a macro above, the following line of
 code can be pasted before the beginning of the C source code.

        _FOSC(CSW_FSCM_ON & EC_PLL16);

 This would enable the external clock with the PLL set to 16x and further,
 enable clock switching and failsafe clock monitoring.

 Similarly, to set the FBORPOR fuse, paste the following :

        _FBORPOR(PBOR_ON & BORV_27 & PWRT_ON_64 & MCLR_DIS);

 This would enable Brown-out Reset at 2.7 Volts and initialize the Power-up
 timer to 64 milliseconds and configure the use of the MCLR pin for I/O.
 Given below, is a complete list of settings valid to each of the fuses:
 (Paste the ones relevant to your application before the beginning of C
 source code.)

               FOSC:
               ======
               _FOSC(CSW_FSCM_OFF & EC);
               _FOSC(CSW_FSCM_OFF & EC_IO);
               _FOSC(CSW_FSCM_OFF & EC_PLL4);
               _FOSC(CSW_FSCM_OFF & EC_PLL8);
               _FOSC(CSW_FSCM_OFF & EC_PLL16);
               _FOSC(CSW_FSCM_OFF & ERC);
               _FOSC(CSW_FSCM_OFF & ERC_IO);
               _FOSC(CSW_FSCM_OFF & XT);
               _FOSC(CSW_FSCM_OFF & XT_PLL4);
               _FOSC(CSW_FSCM_OFF & XT_PLL8);
               _FOSC(CSW_FSCM_OFF & XT_PLL16);
               _FOSC(CSW_FSCM_OFF & XTL);
               _FOSC(CSW_FSCM_OFF & HS);
               _FOSC(CSW_FSCM_OFF & LP);
               _FOSC(CSW_FSCM_OFF & FRC);
               _FOSC(CSW_FSCM_OFF & LPRC);
               _FOSC(CSW_ON_FSCM_OFF & EC);
               _FOSC(CSW_ON_FSCM_OFF & EC_IO);
               _FOSC(CSW_ON_FSCM_OFF & EC_PLL4);
               _FOSC(CSW_ON_FSCM_OFF & EC_PLL8);
               _FOSC(CSW_ON_FSCM_OFF & EC_PLL16);
               _FOSC(CSW_ON_FSCM_OFF & ERC);
               _FOSC(CSW_ON_FSCM_OFF & ERC_IO);
               _FOSC(CSW_ON_FSCM_OFF & XT);
               _FOSC(CSW_ON_FSCM_OFF & XT_PLL4);
               _FOSC(CSW_ON_FSCM_OFF & XT_PLL8);
               _FOSC(CSW_ON_FSCM_OFF & XT_PLL16);
               _FOSC(CSW_ON_FSCM_OFF & XTL);
               _FOSC(CSW_ON_FSCM_OFF & HS);
               _FOSC(CSW_ON_FSCM_OFF & LP);
               _FOSC(CSW_ON_FSCM_OFF & FRC);
               _FOSC(CSW_ON_FSCM_OFF & LPRC);
               _FOSC(CSW_FSCM_ON & EC);
               _FOSC(CSW_FSCM_ON & EC_IO);
               _FOSC(CSW_FSCM_ON & EC_PLL4);
               _FOSC(CSW_FSCM_ON & EC_PLL8);
               _FOSC(CSW_FSCM_ON & EC_PLL16);
               _FOSC(CSW_FSCM_ON & ERC);
               _FOSC(CSW_FSCM_ON & ERC_IO);
               _FOSC(CSW_FSCM_ON & XT);
               _FOSC(CSW_FSCM_ON & XT_PLL4);
               _FOSC(CSW_FSCM_ON & XT_PLL8);
               _FOSC(CSW_FSCM_ON & XT_PLL16);
               _FOSC(CSW_FSCM_ON & XTL);
               _FOSC(CSW_FSCM_ON & HS);
               _FOSC(CSW_FSCM_ON & LP);
               _FOSC(CSW_FSCM_ON & FRC);
               _FOSC(CSW_FSCM_ON & LPRC);

               FWDT
               =====
               _FWDT(WDT_OFF);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_1);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_2);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_3);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_4);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_5);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_6);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_7);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_8);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_9);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_10);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_11);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_12);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_13);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_14);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_15);
               _FWDT(WDT_ON & WDTPSA_1 & WDTPSB_16);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_1);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_2);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_3);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_4);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_5);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_6);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_7);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_8);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_9);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_10);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_11);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_12);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_13);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_14);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_15);
               _FWDT(WDT_ON & WDTPSA_8 & WDTPSB_16);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_1);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_2);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_3);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_4);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_5);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_6);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_7);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_9);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_10);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_11);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_12);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_13);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_14);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_15);
               _FWDT(WDT_ON & WDTPSA_64 & WDTPSB_16);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_1);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_2);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_3);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_4);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_5);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_6);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_7);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_9);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_10);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_11);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_12);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_13);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_14);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_15);
               _FWDT(WDT_ON & WDTPSA_512 & WDTPSB_16);

               FBORPOR
               ========
               _FBORPOR(PBOR_OFF & MCLR_DIS);
               _FBORPOR(PBOR_OFF & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_OFF & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_OFF & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_OFF & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_OFF & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_OFF & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_OFF & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_OFF & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_OFF & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_4 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_4 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_4 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_4 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_4 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_4 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_4 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_4 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_16 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_16 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_16 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_16 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_16 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_16 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_16 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_16 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_64 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_64 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_64 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_64 & MCLR_DIS);
               _FBORPOR(PBOR_ON & BORV_20 & PWRT_64 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_27 & PWRT_64 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_42 & PWRT_64 & MCLR_EN);
               _FBORPOR(PBOR_ON & BORV_45 & PWRT_64 & MCLR_EN);

               FGS
               ====
               _FGS(CODE_PROT_OFF);
               _FGS(CODE_PROT_ON);

 ---------------------------------------------------------------------------- */


/* -------------------------------------------- */ 
/* Some useful macros for inline assembly stuff */
/* -------------------------------------------- */

#define Nop()    {__asm__ volatile ("nop");}
#define ClrWdt() {__asm__ volatile ("clrwdt");}
#define Sleep()  {__asm__ volatile ("pwrsav #0");}
#define Idle()   {__asm__ volatile ("pwrsav #1");}

/* ---------------------------------------------------------- */ 
/* Some useful macros for allocating data memory              */
/* ---------------------------------------------------------- */

/* The following macros require an argument N that specifies  */
/* alignment. N must a power of two, minimum value = 2.       */
/* For example, to declare an uninitialized array in X memory */
/* that is aligned to a 32 byte address:                      */
/*                                                            */
/* int _XBSS(32) xbuf[16];                                    */
/*                                                            */
/* To declare an initialized array in data EEPROM without     */
/* special alignment:                                         */
/*                                                            */
/* int _EEDATA(2) table1[] = {0, 1, 1, 2, 3, 5, 8, 13, 21};   */
/*                                                            */
#define _XBSS(N)    __attribute__((section(".xbss,b"),aligned(N)))
#define _XDATA(N)   __attribute__((section(".xdata,d"),aligned(N)))
#define _YBSS(N)    __attribute__((section(".ybss,b"),aligned(N)))
#define _YDATA(N)   __attribute__((section(".ydata,d"),aligned(N)))
#define _EEDATA(N)  __attribute__((section(".eedata,r"),aligned(N)))

/* The following macros do not require an argument. They can  */
/* be used to locate a variable in persistent data memory or  */
/* in near data memory. For example, to declare two variables */
/* that retain their values across a device reset:            */
/*                                                            */
/* int _PERSISTENT var1,var2;                                 */
/*                                                            */
#define _PERSISTENT __attribute__((section(".pbss,b")))
#define _NEAR       __attribute__((near))

/* ---------------------------------------------------------- */ 
/* Some useful macros for declaring functions                 */
/* ---------------------------------------------------------- */

/* The following macros can be used to declare interrupt      */
/* service routines (ISRs). For example, to declare an ISR    */
/* for the timer0 interrupt:                                  */
/*                                                            */
/* void _ISR _INT0Interrupt(void);                            */
/*                                                            */
/* To declare an ISR for the SPI1 interrupt with fast         */
/* context save:                                              */
/*                                                            */
/* void _ISRFAST _SPI1Interrupt(void);                        */
/*                                                            */
/* Note: ISRs will be installed into the interrupt vector     */
/* tables automatically if the reserved names listed in the   */
/* MPLAB C30 Compiler User's Guide (DS51284) are used.        */
/*                                                            */
#define _ISR __attribute__((interrupt))
#define _ISRFAST __attribute__((interrupt, shadow))

#endif

