/* This linker script generated from xt-genldscripts.tpp for LSP . */
/* Linker Script for ld -N */
MEMORY
{
  dram1_0_seg :                         org = 0x3FFFE000, len = 0x00800
  iram1_0_seg :                         org = 0x40102000, len = 0x0A000
  irom0_0_seg :                         org = 0x40258000, len = 0x8C000
}

PHDRS
{
  dram1_0_phdr PT_LOAD;
  irom0_0_phdr PT_LOAD;
  iram1_0_phdr PT_LOAD;
}


/*  Default entry point:  */
ENTRY(main_start)

SECTIONS
{
  .dram1.text : ALIGN(4)
  {
    _dram1_text_start = ABSOLUTE(.);

    *(.dram1.literal .dram.literal .dram.text.literal .dram1.text .dram.text)
    _dram1_text_end = ABSOLUTE(.);
  } >dram1_0_seg :dram1_0_phdr

  .text : ALIGN(4)
  {
    _iram1_text_start = ABSOLUTE(.);

    *(.UserEnter.text)
    . = ALIGN (16);
    *(.entry.text)
    *(.init.literal)
    *(.init)
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.fini.literal)
    *(.fini)
    *libm.a:(.literal .text .literal.* .text.*)

    *(.iram1.literal .iram.literal .iram.text.literal .iram1.text .iram.text)
    *(.rodata)
    *(.gnu.version)
    . = ALIGN (4);
    _iram1_text_end = ABSOLUTE(.);
  } >iram1_0_seg :iram1_0_phdr

  .irom0.text : ALIGN(4)
  {
    _stext = .;
    _text_start = ABSOLUTE(.);
    KEEP(*(.irom2.literal .irom2.text))
    *libc.a:(.literal .text .literal.* .text.*)
    *libgcc.a:(.literal .text .literal.* .text.*)

    *(.irom0.literal .irom.literal .irom.text.literal .irom0.text .irom.text)
    *(.rodata.*)
    . = ALIGN (4);
    _text_end = ABSOLUTE(.);
    _etext = .;
  } >irom0_0_seg :irom0_0_phdr
}

/* get ROM code address */
INCLUDE "../ld/eagle.rom.addr.v6.ld"
