#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_Hh


// ------------------------------------
// Includes
// ------------------------------------

#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <signal.h>

#include <common/mavlink.h>


// ------------------------------------
// Defines
// ------------------------------------

// non standard baudrates, need to be defined.
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

// status flags
#define SERIAL_PORT_OPEN    1;
#define SERIAL_PORT_CLOSED  0;
#define SERIAL_PORT_ERROR   -1;


// -------------------------------------
// Prototypes
// -------------------------------------

class Serial_Port
{

    public:

        Serial_Port();
        Serial_Port(const char *uart_name_, int baudrate_);
        void initialize_defaults();
        ~Serial_Port();

        bool debug;
        const char *uart_name;
        int baudrate;
        int status;

        int read_message(mavlink_message_t &message);
        int write_message(const mavlink_message_t &message);

        void open_serial();
        void close_serial();

        void start();
        void stop();

        void handle_quit(int sig);


    private:

        int fd;
        mavlink_message_t lastStatus;
        pthread_mutex_t lock;

        int _open_port(const char* port);
        bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
        int _read_port(uint8_t &cp);
        int _write_port(char *buf, unsigned len);

};


#endif