#include <stdint.h>
#include <stm32f10x.h>
#include <stdbool.h>

int main(void) {

    RCC->APB2ENR |= 0b10000; // 0b10000
    GPIOC->CRH &= ~ //'~' - обратная маска
    GPIOC->CRH &= ~(CPIO_CRH_CNF13 | CGIP_CHR_MODE13); //GPIOC->CRH[23:20]=0000
    GPIOC->CRH |= GPIO_CRH_MODE13_0; // GPIOC->CRH[23:20]=001
    GPIO->ODR = GPIO_ODR_ODR13
return 0;
}
