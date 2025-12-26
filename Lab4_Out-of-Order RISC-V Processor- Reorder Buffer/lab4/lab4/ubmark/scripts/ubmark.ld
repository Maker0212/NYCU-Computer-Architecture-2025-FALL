OUTPUT_ARCH( "riscv" )
ENTRY(_start)

PHDRS {
    text PT_LOAD FLAGS(5);   /* R + X = 0x1 | 0x4 */
    data PT_LOAD FLAGS(6);   /* R + W = 0x2 | 0x4 */
}

SECTIONS {
    . = 0x80000;

    .text : {
        *(.text) *(.text.*)
    } :text

    .rodata : {
        *(.rodata) *(.rodata.*)
    } :text

    .data : {
        *(.data) *(.data.*)
        . = . + 4;
    } :data

    .bss : {
        *(.bss) *(.bss.*)
        . = . + 4;
    } :data
}