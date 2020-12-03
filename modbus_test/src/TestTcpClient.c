#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>

#define LOOP            1
#define SERVER_ID       17
#define ADDRESS_START   0
#define ADDRESS_END     1

int main(void)
{
    modbus_t *ctx;
    int rc;
    int nb_fail;
    int nb_loop;
    int addr = 0;
    int nb;
    uint8_t *tab_rq_bits;
    uint8_t *tab_rp_bits;
    uint16_t *tab_rq_registers;
    uint16_t *tab_rw_rq_registers;
    uint16_t *tab_rp_registers;

    ctx = modbus_new_tcp("127.0.0.1", 1502);
    modbus_set_debug(ctx, TRUE);

    if(modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    nb = ADDRESS_END - ADDRESS_START;

    tab_rq_bits = (uint8_t *)malloc(nb * sizeof(uint8_t));
    memset(tab_rq_bits, 0, nb * sizeof(uint8_t));

    tab_rp_bits = (uint8_t *)malloc(nb * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb * sizeof(uint8_t));

    tab_rq_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rq_registers, 0, nb * sizeof(uint16_t));

    tab_rw_rq_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rw_rq_registers, 0, nb * sizeof(uint16_t));

    tab_rp_registers = (uint16_t *)malloc(nb * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb * sizeof(uint16_t));

    nb_loop = nb_fail = 0;
    while (nb_loop++ < LOOP)
    {
        for(addr = ADDRESS_START; addr < ADDRESS_END; ++addr)
        {
            int i;
            for(i = 0; i < nb; ++i)
            {
                tab_rq_registers[i] = (uint16_t)(65535.0 * rand() / (RAND_MAX + 1.0));
                tab_rw_rq_registers[i] = ~tab_rq_registers[i];
                tab_rq_bits[i] = tab_rq_registers[i] % 2;
            }
            nb = ADDRESS_END - addr;

            // 0x05, write single coil
            // [00][01][00][00][00][06][FF][05][00][00][FF][00]
            // Waiting for a confirmation...
            // <00><01><00><00><00><06><FF><05><00><00><FF><00>
            rc = modbus_write_bit(ctx, addr, tab_rq_bits[0]);
            if(rc != 1)
            {
                printf("ERR modbus_write_bit (%d) - %s \n", rc, modbus_strerror(errno));
                printf("Address = %d, value = %d\n", addr, tab_rq_bits[0]);
                ++nb_fail;
            }else
            {
                // 0x01, read some coils
                // [00][02][00][00][00][06][FF][01][00][00][00][01]
                // Waiting for a confirmation...
                // <00><02><00><00><00><04><FF><01><01><01>
                rc = modbus_read_bits(ctx, addr, 1, tab_rp_bits);
                if(rc != 1 || tab_rq_bits[0] != tab_rp_bits[0])
                {
                    printf("ERROR modbus_read_bits single (%d)\n", rc);
                    printf("Address = %d\n", addr);
                    ++ nb_fail;
                }
            }

            // 0x0F, write some coils
            // [00][03][00][00][00][08][FF][0F][00][00][00][01][01][01]
            // Waiting for a confirmation...
            // <00><03><00><00><00><06><FF><0F><00><00><00><01>
            rc = modbus_write_bits(ctx, addr, nb, tab_rq_bits);
            if(rc != nb)
            {
                printf("ERROR modbus_write_bits (%d)\n", rc);
                printf("Address = %d, nb = %d\n", addr, nb);
                ++nb_fail;
            }else
            {
                // 0x01, read some coils
                // [00][04][00][00][00][06][FF][01][00][00][00][01]
                // Waiting for a confirmation...
                // <00><04><00><00><00><04><FF><01><01><01>
                rc = modbus_read_bits(ctx, addr, nb, tab_rp_bits);
                if(rc != nb)
                {
                    printf("ERROR modbus_read_bits\n");
                    printf("Address = %d, nb = %d\n", addr, nb);
                    ++ nb_fail;
                }else
                {
                    for(i = 0; i < nb; ++i)
                    {
                        if(tab_rp_bits[i] != tab_rq_bits[i])
                        {
                            printf("ERROR modbus_read_bits\n");
                            printf("Addr = %d, %d (0x%X) != %d (0x%X)\n",
                            addr, tab_rq_bits[i], tab_rq_bits[i],
                            tab_rp_bits[i], tab_rp_bits[i]);
                            ++nb_fail;
                        }
                    }
                }
            }

            // 0x06, write single register
            // [00][05][00][00][00][06][FF][06][00][00][D7][15]
            // Waiting for a confirmation...
            // <00><05><00><00><00><06><FF><06><00><00><D7><15>
            rc = modbus_write_register(ctx, addr, tab_rq_registers[0]);
            if(rc != 1)
            {
                printf("ERROR modbus_write_register (%d)\n", rc);
                printf("Addr = %d, value = %d (0x%X)\n", addr, tab_rq_registers[0], tab_rq_registers[0]);
                ++nb_fail;
            }else
            {
                // 0x03, read holding registers
                // [00][06][00][00][00][06][FF][03][00][00][00][01]
                // Waiting for a confirmation...
                // <00><06><00><00><00><05><FF><03><02><D7><15>
                rc = modbus_read_registers(ctx, addr, 1, tab_rp_registers);
                if(rc != 1)
                {
                    printf("ERROR modbus_read_registers (%d)\n", rc);
                    printf("Address = %d\n", addr);
                    ++nb_fail;
                }else
                {
                    if(tab_rq_registers[0] != tab_rp_registers[0])
                    {
                        printf("ERROR modbus_read_registers single\n");
                        printf("Addr = %d, %d (0x%X) != %d (0x%X)\n", addr, tab_rq_registers[0], tab_rq_registers[0], tab_rp_registers[0], tab_rp_registers[0]);
                        ++nb_fail;
                    }
                }
                
            }
            
        }
    }

    return 0;
}