#include "tcp_server.hpp"


int main(int argc, char *argv[])
{
    TcpServer server("0.0.0.0", 8800);
    server.launch();
    
    return 0;
}