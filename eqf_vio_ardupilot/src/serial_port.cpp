#include "serial_port.h"


void Serial_Port::Serial_port(const char *uart_name_, int baudrate_)
{
    initialize_defaults();
    uart_name = uart_name_;
    baudrate = baudrate_;
}

