#ifndef _UTILS_HEADER
#define _UTILS_HEADER

#include <stdint.h>

static inline void spin(volatile uint32_t count);
static inline void spin_one();
int strlen(const char *str);
void itoa(int num, char *buf, int buf_len);

#endif