#include "com_debug.h"

int fputc(int ch, FILE *f) {
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
    return ch;
}

