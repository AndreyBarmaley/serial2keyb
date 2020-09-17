/***************************************************************************
 *   Copyright (C) 2020 by SWE team <public.irkutsk@gmail.com>             *
 *                                                                         *
 *   https://github.com/AndreyBarmaley/serial2keyb                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*
 *
 * Build: gcc -O2 serial2keyb.c -o serial2keyb
*/

#include <linux/input.h>
#include <linux/uinput.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>

#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <libgen.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#include "jsmn.h"

#define VERSION 20200522
#define BUFSIZE 128

extern char* optarg;
extern int optind, opterr, optopt;
int getopt(int argc, char * const argv[],  const char* optstring);

speed_t int_speed(int baudrate);

int  tty_open(const char* device, int baudrate);
void tty_close(int fd);
void tty_read_dump(const char* buf, int len);
int  tty_set_attribs(int fd, speed_t speed, int parity, int blocking);
int  tty_backup(int fd, struct termios*);
int  tty_restore(int fd, struct termios*);
int  json_parse(const char* file);

void key_event(int uinp_fd, struct input_event* event, int key);
void key_event_with_shift(int uinp_fd, struct input_event* event, int key);
void key_event_press(int uinp_fd, struct input_event* event, int key);
void key_event_release(int uinp_fd, struct input_event* event, int key);
void events_processing(struct input_event* event, int byte, int test_mode);
void events_processing_string(struct input_event* event, const char* str, int test_mode);
void term_handler(int);

int debug = 0;
int speed = 9600;
int wait = 100;
int end_byte = -1;
int uinp_fd = -1;
int ttys_fd = -1;
char* device2 = NULL;
char* prefix = NULL;
char* suffix = NULL;
const char* progname = NULL;
struct termios ttys_st;

int main(int argc, char** argv)
{
    const char* uinput = "/dev/uinput";
    const char* config = NULL;
    const char* device = "/dev/ttyUSB0";

    int opt;
    int test = 0;
    int beginpart = 1;
    progname = basename(argv[0]);

    while((opt = getopt(argc, argv, "htvs:w:i:o:c:")) != -1)
    switch(opt)
    {
        case 't': test = 1; break;
        case 'v': debug = 1; break;
        case 's': speed = optarg ? atoi(optarg) : 9600; break;
        case 'w': wait = optarg ? atoi(optarg) : 100; break;
        case 'i': device = optarg; break;
        case 'o': uinput = optarg; break;
        case 'c': config = optarg; break;
        default:
            fprintf(stdout, "%s, version %d, usage: [-v verbose] [-c config.json] [-t test mode (skip uinput)] [-i serial device (/dev/ttyUSB0)] [-s speed (9600)] [-w wait ms (100)] [-o uinput device (/dev/uinput)]\n", progname, VERSION);
            fprintf(stdout, "config json example: { \"verbose\": true, \"device\": \"/dev/ttyUSB0\", \"speed\": 9600, \"waitms\": 100, \"endl\": 0x0d, \"prefix\": \"\", \"suffix\": \"\" }\n");
            return 0;
    }

    if(debug) fprintf(stderr, "%s: version: %d\n", progname, VERSION);
    if(config) json_parse(config);
    if(0 > end_byte && (prefix || suffix)) fprintf(stderr, "param \"endl\" not set, suffix or prefix skipping...\n");

    uinp_fd = open(uinput, O_WRONLY | O_NDELAY);

    if(0 > uinp_fd && test == 0)
    {
        fprintf(stderr, "%s: unable to open %s, check permissions or load uinput module\n", progname, uinput);
        return -1;
    }

    struct uinput_user_dev uinp;
    memset(&uinp, 0, sizeof(struct uinput_user_dev));
    strncpy(uinp.name, argv[0], UINPUT_MAX_NAME_SIZE);

    if(test == 0)
    {
        ioctl(uinp_fd, UI_SET_EVBIT, EV_KEY);
        ioctl(uinp_fd, UI_SET_EVBIT, EV_REP);

        int i;
        for(i = 0; i < 256; i++) ioctl(uinp_fd, UI_SET_KEYBIT, i);
        write(uinp_fd, & uinp, sizeof(struct uinput_user_dev));

        if(ioctl(uinp_fd, UI_DEV_CREATE))
        {
            fprintf(stderr, "%s: unable to create uinput device", progname);
            return -1;
        }
    }

    struct sigaction sa;
    memset(& sa, 0, sizeof(struct sigaction));
    sigset_t newset;
    sigemptyset(& newset);
    sigaddset(& newset, SIGHUP);
    sigprocmask(SIG_BLOCK, & newset, 0);
    sa.sa_handler = term_handler;
    sigaction(SIGTERM, & sa, 0);

    char buf[BUFSIZE];
    struct input_event event;
    memset(& event, 0, sizeof(struct input_event));

    while(1)
    {
        if(0 < ttys_fd)
        {
            int len = read(ttys_fd, buf, BUFSIZE);
            if(len < 0)
            {
                fprintf(stderr, "%s error: %s, errno: %d\n", "tty_read", strerror(errno), errno);
                tty_close(ttys_fd);
                ttys_fd = -1;
            }
            else
            // read data
            if(0 < len)
            {
                if(debug)
                    tty_read_dump(buf, len);

                for(int pos = 0; pos < len; ++pos)
                {
                    if(beginpart)
                    {
                        if(prefix)
                            events_processing_string(& event, prefix, test);
                        beginpart = 0;
                    }

                    if(0 <= end_byte && end_byte == buf[pos])
                    {
                        if(suffix)
                            events_processing_string(& event, suffix, test);
                        beginpart = 1;
                    }

                    events_processing(& event, buf[pos], test);
                }
            }
        }
        else
        {
            ttys_fd = tty_open(device2 ? device2 : device, speed);
            if(0 > ttys_fd) usleep(900000);
        }

        usleep(wait * 1000);
    }

    term_handler(0);
    return 0;
}

void parse_standard()
{
}

void term_handler(int r)
{
    if(0 < ttys_fd)
    {
        tty_close(ttys_fd);
        ttys_fd = -1;
    }

    if(0 < uinp_fd)
    {
        ioctl(uinp_fd, UI_DEV_DESTROY);
        close(uinp_fd);
    }

    if(device2)
        free(device2);

    if(prefix)
        free(prefix);

    if(suffix)
        free(suffix);

    exit(EXIT_SUCCESS);
}

speed_t int_speed(int baudrate)
{
    switch(baudrate)
    {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: break;
    }

    return B0;
}

void events_processing_string(struct input_event* event, const char* str, int test_mode)
{
    while(str && *str)
        events_processing(event, *str++, test_mode);
}

void events_processing(struct input_event* event, int byte, int test_mode)
{
    if(test_mode)
        fprintf(stderr, "events_processing: %.2x\n", byte);
    else
    switch(byte)
    {
        case '`':  key_event(uinp_fd, event, KEY_GRAVE); break;
        case '0':  key_event(uinp_fd, event, KEY_0); break;
        case '1':  key_event(uinp_fd, event, KEY_1); break;
        case '2':  key_event(uinp_fd, event, KEY_2); break;
        case '3':  key_event(uinp_fd, event, KEY_3); break;
        case '4':  key_event(uinp_fd, event, KEY_4); break;
        case '5':  key_event(uinp_fd, event, KEY_5); break;
        case '6':  key_event(uinp_fd, event, KEY_6); break;
        case '7':  key_event(uinp_fd, event, KEY_7); break;
        case '8':  key_event(uinp_fd, event, KEY_8); break;
        case '9':  key_event(uinp_fd, event, KEY_9); break;
        case '-':  key_event(uinp_fd, event, KEY_MINUS); break;
        case '=':  key_event(uinp_fd, event, KEY_EQUAL); break;
        case '\\': key_event(uinp_fd, event, KEY_BACKSLASH); break;
        case '\b': key_event(uinp_fd, event, KEY_BACKSPACE); break;
        case '\t': key_event(uinp_fd, event, KEY_TAB); break;
        case 'a':  key_event(uinp_fd, event, KEY_A); break;
        case 'b':  key_event(uinp_fd, event, KEY_B); break;
        case 'c':  key_event(uinp_fd, event, KEY_C); break;
        case 'd':  key_event(uinp_fd, event, KEY_D); break;
        case 'e':  key_event(uinp_fd, event, KEY_E); break;
        case 'f':  key_event(uinp_fd, event, KEY_F); break;
        case 'g':  key_event(uinp_fd, event, KEY_G); break;
        case 'h':  key_event(uinp_fd, event, KEY_H); break;
        case 'i':  key_event(uinp_fd, event, KEY_I); break;
        case 'j':  key_event(uinp_fd, event, KEY_J); break;
        case 'k':  key_event(uinp_fd, event, KEY_K); break;
        case 'l':  key_event(uinp_fd, event, KEY_L); break;
        case 'm':  key_event(uinp_fd, event, KEY_M); break;
        case 'n':  key_event(uinp_fd, event, KEY_N); break;
        case 'o':  key_event(uinp_fd, event, KEY_O); break;
        case 'p':  key_event(uinp_fd, event, KEY_P); break;
        case 'q':  key_event(uinp_fd, event, KEY_Q); break;
        case 'r':  key_event(uinp_fd, event, KEY_R); break;
        case 's':  key_event(uinp_fd, event, KEY_S); break;
        case 't':  key_event(uinp_fd, event, KEY_T); break;
        case 'u':  key_event(uinp_fd, event, KEY_U); break;
        case 'v':  key_event(uinp_fd, event, KEY_V); break;
        case 'w':  key_event(uinp_fd, event, KEY_W); break;
        case 'x':  key_event(uinp_fd, event, KEY_X); break;
        case 'y':  key_event(uinp_fd, event, KEY_Y); break;
        case 'z':  key_event(uinp_fd, event, KEY_Z); break;
        case '[':  key_event(uinp_fd, event, KEY_LEFTBRACE); break;
        case ']':  key_event(uinp_fd, event, KEY_RIGHTBRACE); break;
        case '\r': key_event(uinp_fd, event, KEY_ENTER); break;
        case ';':  key_event(uinp_fd, event, KEY_SEMICOLON); break;
        case '\'': key_event(uinp_fd, event, KEY_APOSTROPHE); break;
        case ',':  key_event(uinp_fd, event, KEY_COMMA); break;
        case '.':  key_event(uinp_fd, event, KEY_DOT); break;
        case '/':  key_event(uinp_fd, event, KEY_SLASH); break;
        case 0x20: key_event(uinp_fd, event, KEY_SPACE); break;

        case '~': key_event_with_shift(uinp_fd, event, KEY_GRAVE); break;
        case '!': key_event_with_shift(uinp_fd, event, KEY_1); break;
        case '@': key_event_with_shift(uinp_fd, event, KEY_2); break;
        case '#': key_event_with_shift(uinp_fd, event, KEY_3); break;
        case '$': key_event_with_shift(uinp_fd, event, KEY_4); break;
        case '%': key_event_with_shift(uinp_fd, event, KEY_5); break;
        case '^': key_event_with_shift(uinp_fd, event, KEY_6); break;
        case '&': key_event_with_shift(uinp_fd, event, KEY_7); break;
        case '*': key_event_with_shift(uinp_fd, event, KEY_8); break;
        case '(': key_event_with_shift(uinp_fd, event, KEY_9); break;
        case ')': key_event_with_shift(uinp_fd, event, KEY_0); break;
        case '_': key_event_with_shift(uinp_fd, event, KEY_MINUS); break;
        case '+': key_event_with_shift(uinp_fd, event, KEY_EQUAL); break;
        case '|': key_event_with_shift(uinp_fd, event, KEY_BACKSLASH); break;
        case '{': key_event_with_shift(uinp_fd, event, KEY_LEFTBRACE); break;
        case '}': key_event_with_shift(uinp_fd, event, KEY_RIGHTBRACE); break;
        case '<': key_event_with_shift(uinp_fd, event, KEY_COMMA); break;
        case '>': key_event_with_shift(uinp_fd, event, KEY_DOT); break;
        case '?': key_event_with_shift(uinp_fd, event, KEY_SLASH); break;

        case 'A': key_event_with_shift(uinp_fd, event, KEY_A); break;
        case 'B': key_event_with_shift(uinp_fd, event, KEY_B); break;
        case 'C': key_event_with_shift(uinp_fd, event, KEY_C); break;
        case 'D': key_event_with_shift(uinp_fd, event, KEY_D); break;
        case 'E': key_event_with_shift(uinp_fd, event, KEY_E); break;
        case 'F': key_event_with_shift(uinp_fd, event, KEY_F); break;
        case 'G': key_event_with_shift(uinp_fd, event, KEY_G); break;
        case 'H': key_event_with_shift(uinp_fd, event, KEY_H); break;
        case 'I': key_event_with_shift(uinp_fd, event, KEY_I); break;
        case 'J': key_event_with_shift(uinp_fd, event, KEY_J); break;
        case 'K': key_event_with_shift(uinp_fd, event, KEY_K); break;
        case 'L': key_event_with_shift(uinp_fd, event, KEY_L); break;
        case 'M': key_event_with_shift(uinp_fd, event, KEY_M); break;
        case 'N': key_event_with_shift(uinp_fd, event, KEY_N); break;
        case 'O': key_event_with_shift(uinp_fd, event, KEY_O); break;
        case 'P': key_event_with_shift(uinp_fd, event, KEY_P); break;
        case 'Q': key_event_with_shift(uinp_fd, event, KEY_Q); break;
        case 'R': key_event_with_shift(uinp_fd, event, KEY_R); break;
        case 'S': key_event_with_shift(uinp_fd, event, KEY_S); break;
        case 'T': key_event_with_shift(uinp_fd, event, KEY_T); break;
        case 'U': key_event_with_shift(uinp_fd, event, KEY_U); break;
        case 'V': key_event_with_shift(uinp_fd, event, KEY_V); break;
        case 'W': key_event_with_shift(uinp_fd, event, KEY_W); break;
        case 'X': key_event_with_shift(uinp_fd, event, KEY_X); break;
        case 'Y': key_event_with_shift(uinp_fd, event, KEY_Y); break;
        case 'Z': key_event_with_shift(uinp_fd, event, KEY_Z); break;

        default: break;
    }
}

int tty_backup(int fd, struct termios* tty)
{
    memset(tty, 0, sizeof(struct termios));

    if(0 != tcgetattr(fd, tty))
    {
        fprintf(stderr, "%s error: %s, errno: %d\n", "tcgetattr", strerror(errno), errno);
        return -1;
    }

    return 0;
}

int tty_restore(int fd, struct termios* tty)
{
    if(0 > tcsetattr(fd, TCSANOW, tty))
    {
        fprintf(stderr, "%s error: %s, errno, %d\n", "tcsetattr", strerror(errno), errno);
        return -1;
    }

    return 0;
}

int tty_set_attribs(int fd, speed_t speed, int parity, int blocking)
{
    struct termios tty;

    if(0 > tty_backup(fd, &tty))
        return -1;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;                        // shut off parity
    tty.c_cflag &= ~CSTOPB;                        // set stop bits
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                            // 8 bit chars
    tty.c_cflag &= ~CRTSCTS;                       // no flow control
    tty.c_cflag |= (CLOCAL | CREAD);               // ignore modem ctrl
    tty.c_cflag |= parity;

    tty.c_iflag &= ~IGNBRK;                        // disable break
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);        // turn off xon/xoff ctrl

    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = blocking ? 1 : 0;             // read doesn't block
    tty.c_cc[VTIME] = 5;                           // 0.5 seconds read timeout

    return tty_restore(fd, &tty);
}

int tty_open(const char* device, int baudrate)
{
    int fd = open(device, O_RDONLY | O_NOCTTY | O_SYNC);
    if(0 > fd)
    {
        fprintf(stderr, "%s: unable open device: %s\n", progname, device);
        return -1;
    }

    if(0 > tty_backup(fd, &ttys_st))
        return -1;

    if(0 > tty_set_attribs(fd, int_speed(baudrate), 0, 0))
        return -1;

    if(debug) fprintf(stderr, "device open %s, fd: %d\n", device, fd);
    return fd;
}

void tty_read_dump(const char* buf, int len)
{
    fprintf(stderr, "tty_read data: %d byte, [ ", len);
    int pos;

    for(pos = 0; pos < len; ++pos)
        fprintf(stderr, "%.2x, ", buf[pos]);

    fprintf(stderr, "]\n");
}

void tty_close(int fd)
{
    tty_restore(fd, &ttys_st);
    if(debug) fprintf(stderr, "close fd: %d\n", fd);
    close(fd);
}

void key_event(int uinp_fd, struct input_event* event, int key)
{
    key_event_press(uinp_fd, event, key);
    key_event_release(uinp_fd, event, key);
}

void key_event_with_shift(int uinp_fd, struct input_event* event, int key)
{
    key_event_press(uinp_fd, event, KEY_LEFTSHIFT);
    key_event(uinp_fd, event, key);
    key_event_release(uinp_fd, event, KEY_LEFTSHIFT);
}

void key_event_press(int uinp_fd, struct input_event* event, int key)
{
    gettimeofday(&event->time, NULL);
    event->type = EV_KEY;
    event->code = key;
    event->value = 1;
    write(uinp_fd, event, sizeof(struct input_event));
    event->type = EV_SYN;
    event->code = SYN_REPORT;
    event->value = 0;
    write(uinp_fd, event, sizeof(struct input_event));
}

void key_event_release(int uinp_fd, struct input_event* event, int key)
{
    gettimeofday(&event->time, NULL);
    event->type = EV_KEY;
    event->code = key;
    event->value = 0;
    write(uinp_fd, event, sizeof(struct input_event));
    event->type = EV_SYN;
    event->code = SYN_REPORT;
    event->value = 0;
    write(uinp_fd, event, sizeof(struct input_event));
}

int json_eq(const char* json, jsmntok_t* tok, const char* s)
{
  if(tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0)
    {
        return 0;
    }
    return -1;
}

int json_parse(const char* file)
{
    char* buf = NULL;
    int fsize = 0;
    FILE* fd = fopen(file, "rb");

    if(!fd)
    {
        fprintf(stderr, "%s error: %s, errno: %d\n", "json_parse", strerror(errno), errno);
        return 0;
    }

    fseek(fd, 0, SEEK_END);
    fsize = ftell(fd);
    fseek(fd, 0, SEEK_SET);
    buf = calloc(1, fsize);
    fread(buf, fsize, 1, fd);
    fclose(fd);

    jsmntok_t t[32];
    jsmn_parser p;

    jsmn_init(& p);
    int i;
    int r = jsmn_parse(& p, buf, strlen(buf), t, sizeof(t) / sizeof(t[0]));
    if(r < 0)
    {
        fprintf(stderr, "json_parse error: failed to parse JSON: %d\n", r);
        return 0;
    }

    if(r < 1 || t[0].type != JSMN_OBJECT)
    {
        fprintf(stderr, "json_parse error: object expected\n");
        return 0;
    }

    for(i = 1; i < r; i++)
    {
        const char* strptr = buf + t[i + 1].start;
        int len = t[i + 1].end - t[i + 1].start;

        if(json_eq(buf, &t[i], "verbose") == 0)
        {
            debug = strncmp(strptr, "true", len) == 0 ? 1 : 0;
            i++;
        }
        else
        if(json_eq(buf, &t[i], "speed") == 0)
        {
            speed = atoi(strptr);
            i++;
        }
        else
        if(json_eq(buf, &t[i], "wait") == 0)
        {
            wait = atoi(strptr);
            i++;
        }
        else
        if(json_eq(buf, &t[i], "device") == 0)
        {
            if(device2) free(device2);
            device2 = 0 < len ? strndup(strptr, len) : NULL;
            i++;
        }
        else
        if(json_eq(buf, &t[i], "endl") == 0)
        {
            end_byte = strtol(strptr, NULL, 16);
        }
        else
        if(json_eq(buf, &t[i], "prefix") == 0)
        {
            if(prefix) free(prefix);
            prefix = 0 < len ? strndup(strptr, len) : NULL;
            i++;
        }
        else
        if(json_eq(buf, &t[i], "suffix") == 0)
        {
            if(suffix) free(suffix);
            suffix = 0 < len ? strndup(strptr, len) : NULL;
            i++;
        }
    }

    free(buf);
    return 1;
}
