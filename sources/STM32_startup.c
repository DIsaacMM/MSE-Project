#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/* ================== LINKER SYMBOLS ================== */
extern uint32_t _estack; 
extern uint32_t _sidata;   // FIX IMPORTANTE
extern uint32_t _sdata; 
extern uint32_t _edata; 
extern uint32_t _sbss; 
extern uint32_t _ebss; 

/* ================== PROTOTYPES ================== */
void Reset_Handler(void);
int main(void);  
void Default_Handler(void);

/* Weak aliases */
void NMI_Handler(void)                      __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void)                __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler(void)                __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler(void)                 __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler(void)               __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void)                      __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler(void)                 __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void)                   __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void)                  __attribute__ ((weak, alias("Default_Handler")));

/* ================== VECTOR TABLE ================== */
/* FIX: nombre correcto de sección */
uint32_t vector_tbl[] __attribute__((section(".isr_vector"))) = 
{
  (uint32_t)&_estack,
  (uint32_t)&Reset_Handler,
  (uint32_t)&NMI_Handler,
  (uint32_t)&HardFault_Handler,
  (uint32_t)&MemManage_Handler,
  (uint32_t)&BusFault_Handler,
  (uint32_t)&UsageFault_Handler,
  0, 0, 0, 0,
  (uint32_t)&SVC_Handler,
  (uint32_t)&DebugMon_Handler,
  0,
  (uint32_t)&PendSV_Handler,
  (uint32_t)&SysTick_Handler,
};

/* ================== DEFAULT HANDLER ================== */
void Default_Handler()
{
    while(1);
}

/* ================== RESET HANDLER ================== */
void Reset_Handler()
{
    
    /* Call the clock system initialization function. */
    SystemInit(); 

    /* 1. Tamaños */
    uint32_t data_mem_size = (uint32_t)&_edata - (uint32_t)&_sdata; 
    uint32_t bss_mem_size  = (uint32_t)&_ebss - (uint32_t)&_sbss; 

    data_mem_size /= 4; 
    bss_mem_size  /= 4; 

    /* FIX CLAVE: usar _sidata */
    uint32_t *p_src_mem  = (uint32_t *)&_sidata; 
    uint32_t *p_dest_mem = (uint32_t *)&_sdata; 

    /* 2. Copiar .data */
    for(uint32_t i = 0; i < data_mem_size; i++)
    {
        *p_dest_mem++ = *p_src_mem++; 
    }

    /* 3. Limpiar .bss */
    p_dest_mem = (uint32_t *)&_sbss; 

    for(uint32_t i = 0; i < bss_mem_size; i++)
    {
        *p_dest_mem++ = 0; 
    }

    /* 4. Ir a main */
    main(); 

    /* seguridad */
    while(1);
}