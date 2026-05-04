/* Entry point */
ENTRY(Reset_Handler)

/* Memory definition */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Top of stack */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* Sections */
SECTIONS
{
    /* ===== TEXT ===== */
    .text :
    {
        . = ALIGN(4);

        KEEP(*(.isr_vector))   /* Vector table */
        *(.text*)
        *(.rodata*)

        . = ALIGN(4);
        _etext = .;
    } > FLASH


    /* ===== DATA ===== */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;

        *(.data*)

        . = ALIGN(4);
        _edata = .;
    } > SRAM AT > FLASH

    /* Dirección de carga en FLASH */
    _sidata = LOADADDR(.data);


    /* ===== BSS ===== */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;

        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;
    } > SRAM
}