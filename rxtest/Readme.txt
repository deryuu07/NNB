-------- PROJECT GENERATOR --------
PROJECT NAME :	rxtest
PROJECT DIRECTORY :	C:\WorkSpace\rxtest\rxtest
CPU SERIES :	RX600
CPU TYPE :	RX62T
TOOLCHAIN NAME :	Renesas RX Standard Toolchain
TOOLCHAIN VERSION :	1.2.1.0
GENERATION FILES :
    C:\WorkSpace\rxtest\rxtest\dbsct.c
        Setting of B,R Section
    C:\WorkSpace\rxtest\rxtest\typedefine.h
        Aliases of Integer Type
    C:\WorkSpace\rxtest\rxtest\sbrk.c
        Program of sbrk
    C:\WorkSpace\rxtest\rxtest\iodefine.h
        Definition of I/O Register
    C:\WorkSpace\rxtest\rxtest\intprg.c
        Interrupt Program
    C:\WorkSpace\rxtest\rxtest\vecttbl.c
        Initialize of Vector Table
    C:\WorkSpace\rxtest\rxtest\vect.h
        Definition of Vector
    C:\WorkSpace\rxtest\rxtest\resetprg.c
        Reset Program
    C:\WorkSpace\rxtest\rxtest\rxtest.c
        Main Program
    C:\WorkSpace\rxtest\rxtest\sbrk.h
        Header file of sbrk file
    C:\WorkSpace\rxtest\rxtest\stacksct.h
        Setting of Stack area
START ADDRESS OF SECTION :
 H'1000	B_1,R_1,B_2,R_2,B,R,SU,SI
 H'FFFF8000	PResetPRG
 H'FFFF8100	C_1,C_2,C,C$*,D_1,D_2,D,P,PIntPRG,W*,L
 H'FFFFFFD0	FIXEDVECT

* When the user program is executed,
* the interrupt mask has been masked.
* 
* Program start 0xFFFF8000.
* RAM start 0x1000.

DATE & TIME : 2014/04/17 22:04:43
