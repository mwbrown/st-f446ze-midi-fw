
#include <FreeRTOS.h>

void *malloc(size_t size)
{
    return pvPortMalloc(size);
}

void free(void *ptr)
{
    vPortFree(ptr);
}
