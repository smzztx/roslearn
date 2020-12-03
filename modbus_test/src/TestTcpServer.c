#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>

int main(void)
{
    int server_socket = -1;
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;

    ctx = modbus_new_tcp("127.0.0.1", 1502);
    modbus_set_debug(ctx, TRUE);

    mb_mapping = modbus_mapping_new(500, 500, 500, 500);
    if(mb_mapping == NULL)
    {
        fprintf(stderr, "Failed mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    server_socket = modbus_tcp_listen(ctx, 1);
    if(server_socket == -1)
    {
        fprintf(stderr, "Unable to listen TCP\n");
        modbus_free(ctx);
        return -1;
    }

    modbus_tcp_accept(ctx, &server_socket);

    for(;;)
    {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc;

        rc = modbus_receive(ctx, query);
        if(rc >= 0)
        {
            modbus_reply(ctx, query, rc, mb_mapping);
        }else
        {
            printf("Connection Closed\n");
            modbus_close(ctx);
            modbus_tcp_accept(ctx, &server_socket);
        }
    }

    printf("Quit the loop: %s\n", modbus_strerror(errno));
    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}