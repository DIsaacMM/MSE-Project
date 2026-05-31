#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/* ================== LINKER SYMBOLS ================== */
extern uint32_t _estack; 
extern uint32_t _sidata;
extern uint32_t _sdata; 
extern uint32_t _edata; 
extern uint32_t _sbss; 
extern uint32_t _ebss; 

/* ================== PROTOTYPES ================== */
void Reset_Handler(void);
int main(void);  
void Default_Handler(void);

/* Weak aliases — core */
void NMI_Handler(void)                      __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void)                __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler(void)                __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler(void)                 __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler(void)               __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void)                      __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler(void)                 __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void)                   __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void)                  __attribute__ ((weak, alias("Default_Handler")));

/* Weak alias — periférico que usamos */
void TIM5_IRQHandler(void)                  __attribute__ ((weak, alias("Default_Handler")));

/* ================== VECTOR TABLE ================== */
uint32_t vector_tbl[] __attribute__((section(".isr_vector"))) = 
{
  /* Cortex-M4 core (posiciones 0-15) */
  (uint32_t)&_estack,            /*  0: Stack pointer inicial */
  (uint32_t)&Reset_Handler,      /*  1: Reset                 */
  (uint32_t)&NMI_Handler,        /*  2: NMI                   */
  (uint32_t)&HardFault_Handler,  /*  3: HardFault             */
  (uint32_t)&MemManage_Handler,  /*  4: MemManage             */
  (uint32_t)&BusFault_Handler,   /*  5: BusFault              */
  (uint32_t)&UsageFault_Handler, /*  6: UsageFault            */
  0, 0, 0, 0,                    /*  7-10: reservados         */
  (uint32_t)&SVC_Handler,        /* 11: SVCall                */
  (uint32_t)&DebugMon_Handler,   /* 12: DebugMon              */
  0,                             /* 13: reservado             */
  (uint32_t)&PendSV_Handler,     /* 14: PendSV                */
  (uint32_t)&SysTick_Handler,    /* 15: SysTick               */

  /* STM32F411 periféricos — IRQ 0 a 49 (posiciones 16-65) */
  0,0,0,0,0,0,0,0,0,0,           /* IRQ  0- 9                 */
  0,0,0,0,0,0,0,0,0,0,           /* IRQ 10-19                 */
  0,0,0,0,0,0,0,0,0,0,           /* IRQ 20-29                 */
  0,0,0,0,0,0,0,0,0,0,           /* IRQ 30-39                 */
  0,0,0,0,0,0,0,0,0,0,           /* IRQ 40-49                 */

  /* IRQ 50: TIM5 (posición 66) */
  (uint32_t)&TIM5_IRQHandler,    /* IRQ 50: TIM5              */
};

/* ================== DEFAULT HANDLER ================== */
void Default_Handler()
{
    while(1);
}

/* ================== RESET HANDLER ================== */
void Reset_Handler()
{
    SystemInit(); 

    uint32_t data_mem_size = (uint32_t)&_edata - (uint32_t)&_sdata; 
    uint32_t bss_mem_size  = (uint32_t)&_ebss - (uint32_t)&_sbss; 

    data_mem_size /= 4; 
    bss_mem_size  /= 4; 

    uint32_t *p_src_mem  = (uint32_t *)&_sidata; 
    uint32_t *p_dest_mem = (uint32_t *)&_sdata; 

    for(uint32_t i = 0; i < data_mem_size; i++)
    {
        *p_dest_mem++ = *p_src_mem++; 
    }

    p_dest_mem = (uint32_t *)&_sbss; 

    for(uint32_t i = 0; i < bss_mem_size; i++)
    {
        *p_dest_mem++ = 0; 
    }

    main(); 

    while(1);
}