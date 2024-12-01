MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K

    SRAM4 : ORIGIN = 0x20040000, LENGTH = 4k
    SRAM5 : ORIGIN = 0x20041000, LENGTH = 4k
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;


SECTIONS {
    .bbi2c : ALIGN(4)
    {
        . = ALIGN(4);
        __sbbi2c = .;
        *(.bbi2c_run .bbi2c_run.*);
        . = ALIGN(4);
        __ebbi2c = .;
    } > SRAM4 AT>FLASH
    __sibbi2c = LOADADDR(.bbi2c);

    .sram5 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.bbi2c_stack .bbi2c_stack.*);
        . = ALIGN(4);
    } > SRAM5
} INSERT BEFORE .got;
