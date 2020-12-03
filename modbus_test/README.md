gcc TestTcpServer.c -o server `pkg-config --libs --cflags libmodbus`
gcc TestTcpClient.c -o client `pkg-config --libs --cflags libmodbus`