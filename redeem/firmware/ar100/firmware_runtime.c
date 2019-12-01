
#include <stdint.h>

#define PIO_BASE 0x01C20800
#define PIOE_BASE PIO_BASE + 4*0x24
#define PE_CFG0 PIOE_BASE + 0x0
#define PE_CFG1 PIOE_BASE + 0x4
#define PE_DATA PIOE_BASE + 0x10

#define SHARED_MEM_BASE 0x13000
#define SHARED_MEM_LENGTH 1000
volatile unsigned int *sram = (volatile unsigned int *)SHARED_MEM_BASE;

void inline write_reg(uint32_t addr, uint32_t val){
  *((volatile unsigned long *)(addr)) = val;
}

void delay_cycles(uint32_t cycles){
  if(cycles <= 2)
    return;
  while(cycles--)
  ;
}

void gpio_init(){
  write_reg(PE_CFG0, 1 << 0 | 1 << 4 | 1 << 8 | 1 << 12 | 1 << 16 | 1 << 20 | 1 << 24 | 1 << 28);
  write_reg(PE_CFG1, 1 << 0 | 1 << 4 | 1 << 8 | 1 << 12 | 1 << 16 | 1 << 20 | 1 << 24 | 1 << 28);
}

int main(void){
  gpio_init();
  uint32_t p_shared_mem = SHARED_MEM_BASE;

  while(1){
    write_reg(PE_DATA, sram[p_shared_mem++]);
    delay_cycles(sram[p_shared_mem++]);
    if(p_shared_mem == SHARED_MEM_BASE + SHARED_MEM_LENGTH)
      p_shared_mem = SHARED_MEM_BASE;
  }
}