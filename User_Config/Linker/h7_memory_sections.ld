/*
 * User-owned STM32H7 RAM sections.
 * Kept separate from the STM32CubeMX-generated linker script.
 */

SECTIONS
{
  /* DMA buffers in non-cacheable AXI SRAM. */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    _sdma_buffer = .;
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
    _edma_buffer = .;
  } >RAM_D1

  /* Non-DMA application data and the FreeRTOS heap extension in AXI SRAM. */
  .ram_d1_data (NOLOAD) :
  {
    . = ALIGN(8);
    _sram_d1_data = .;
    *(.ram_d1_data)
    *(.ram_d1_data*)
    . = ALIGN(8);
    _eram_d1_data = .;
  } >RAM_D1
}

