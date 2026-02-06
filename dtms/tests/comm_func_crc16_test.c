#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "../common/comm_func_crc16.h"

int main(void)
{
    {
        const unsigned char data[] = {1, 2, 3, 4, 5};
        unsigned char sum = checkSum((unsigned char *)data, (int)sizeof(data));
        assert(sum == (unsigned char)(1 + 2 + 3 + 4 + 5));
    }

    {
        const unsigned char data[] = "123456789";
        unsigned short crc = do_crc_table((unsigned char *)data, (int)strlen((const char *)data));
        assert(crc == 0x31C3);
    }

    {
        const unsigned char data[] = "";
        unsigned short crc = do_crc_table((unsigned char *)data, 0);
        assert(crc == 0x0000);
    }

    return 0;
}
