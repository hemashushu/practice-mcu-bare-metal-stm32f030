#include <stdint.h>

static inline void spin(volatile uint32_t count)
{
    while (count--)
        asm("nop");
}

static inline void spin_one()
{
    spin(1);
}

int strlen(const char *str)
{
    const char *char_ptr;
    for (char_ptr = str; *char_ptr != 0; ++char_ptr)
    {
        //
    }
    return (char_ptr - str);
}

void itoa(int num, char *buf, int buf_len)
{
    int idx = 0;
    for (; idx < buf_len; idx++)
    {
        int mod = num % 10;
        buf[idx] = mod + '0';

        num = num / 10;
        if (num == 0)
        {
            break;
        }
    }

    int count = buf_len;

    if (idx < buf_len - 1) {
        count = idx + 1;
        buf[count] = '\0';
    }

    // reverse
    for (int i = 0; i < count / 2; i++)
    {
        int j = count - i - 1;
        char t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
    }
}
