#include <stdint.h>
#include <stm32f10x.h>
#include <stdbool.h>

void delay_us(uint32_t us){
    __asm volatile(
        "push {r0}\r\n"
        "mov R0, %0\r\n"      //val = (9 * us) for 72Mhz
        "_loop:\r\n"    //approx. 8ticks/iteration
	        "cmp R0, #0\r\n" //1
	        "beq _exit\r\n"      		//1 or 1+P (when condition is True)
	        "sub R0, R0, #1\r\n" 	//1
	        "nop\r\n" 				//1 allignment
	        "b _loop\r\n" 			//1+P (pipeline refill) ~4 cycle
        "_exit:\r\n"
	        "pop {r0}\r\n"
        :: "r"(9*us) // For 72 Mhz
    );
}



/* Interrupt handler */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        if(GPIOC->ODR & GPIO_ODR_ODR13){
            GPIOC->ODR &= ~GPIO_ODR_ODR13;
        } else {
            GPIOC->ODR |= GPIO_ODR_ODR13;
        }
    //Clear Interrupt flag
    TIM2->SR &= ~TIM_SR_UIF;
    }
}

void cmd(uint8_t data){//отправка данных
    GPIOA->ODR &= ~GPIO_ODR_ODR12; // A0=0 --указание на то, что оправляем команду
    GPIOA->ODR &- ~GPIO_ODR_ODR4; // CS=0 - указание дисплею , что данные адресованы ему
    delay(1000);
    SPI1_Write(data);
    GPIOA->ODR |= GPIO_ODR_ODR4; //CS=1 - окончание передачи данных
}

void dat(uint8_t data){// Отправка данных
    GPIOA->ODR |= GPIO_ODR_ODR12; // A0=1 - указание на то, что отправляем данные
    GPIOA->ODR &= ~GPIO_ODR_ODR4; //CS=0 - указание дисплею, что данные адресованны ему
    delay(1000);
    SPI1_Write(data);
    GPIOA->ODR |= GPIO_ODR_ODR4; //CS=1 - окончание передачи данных
}

int main(void) {
    // int i = 0;
    // int mask = 8; // 8 = 0b10000 = 0x8 = (1 << 4)
    // i = i | mask; // i |= mask;

    /* IO PORTS Configuration */
    //RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // 0b10000=0x10
    //GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); // GPIOC->CRH[23:20]=0000
    //GPIOC->CRH |= GPIO_CRH_MODE13_0; // GPIOC->CRH[23:20]=0001

    /* TIM2 Configuration */
    /*RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
    TIM2->PSC = 4000;
    TIM2->ARR = 22000;
    TIM2->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ in NVIC
    TIM2->CR1 |= TIM_CR1_CEN; // Start timer
    while (1) {
        __asm volatile ("nop");
    }

    while(1){
        GPIOC->ODR &= ~GPIO_ODR_ODR13;
        delay_us(1000000);
        GPIOC->ODR |= GPIO_ODR_ODR13;
        delay_us(1000000);
    }*/

    //Включаем такотовый сигнал порта A
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    //A4
    GPIOA -> CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4); //GPIO -> CRL[19:16]=0000
    GPIOA->CRL |= GPIO_CRL_MODE4_0; //GPIO -> CRL[19:16]=0001
    //A11
    GPIOA->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11); //GPIOA->CRH[15:12]=0000
    GPIOA->CRH |= GPIO_CRH_MODE11_0; // GPIOC->CRH[15:12]=0001
    //A12
    GPIOA->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12); //GPIOA->CRH[19:16]=0000
    GPIOA->CRH |= GPIO_CRH_MODE12_0; // GPIOC->CRH[19:16]=0001
    //A15
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5); //GPIOA->CRL[23:20]=0000
    GPIOA->CRL |= GPIO_CRL_MODE5_0; // GPIOC->CRL[23:20]=0001
    GPIOA->CRL |= GPIO_CRL_CNF5_1; // GPIOC->CRL[23:20]=1001
    //A7
    GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7); //GPIOA->CRL[31:28]=0000
    GPIOA->CRL |= GPIO_CRL_MODE7_0; //GPIOA->CRL[31:28]=0001
    GPIOA->CRL |= GPIO_CRL_CNF7_1; //GPIOA->CRL[31:28]=1001

    SPI1->CR1 &= ~SPI_CR1_DFF;
    SPI1->CR1 &= ~SPI_CR1_CRCEN;
    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;
    SPI1->CR1 &= ~SPI_CR1_BR; //BR[2:0]=000
    SPI1->CR1 |= SPI_CR1_BR; //BR[2:0]=100
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 &= ~SPI_CR1_CPOL;
    SPI1->CR1 &= ~SPI_CR1_CPHA;
    SPI1->CR1 |= SPI_CR1_SPE;

return 0;
}
