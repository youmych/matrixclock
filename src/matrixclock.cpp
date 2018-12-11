#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/time.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define BM(n) (0x01 << (n))

enum {
    SEG_A = BM(6),
    SEG_B = BM(5),
    SEG_C = BM(4),
    SEG_D = BM(3),
    SEG_E = BM(2),
    SEG_F = BM(1),
    SEG_G = BM(0),
    DP    = BM(7),
    BLANK = 0x00,
    FONT_0 = (SEG_A | SEG_B | SEG_D | SEG_E | SEG_F),
    FONT_1 = (SEG_B | SEG_C ),
    FONT_2 = (SEG_A | SEG_B | SEG_G | SEG_E | SEG_D),
    FONT_3 = (SEG_A | SEG_B | SEG_C | SEG_D | SEG_G),
    FONT_4 = (SEG_B | SEG_B | SEG_C | SEG_F | SEG_G),
    FONT_5 = (SEG_A | SEG_C | SEG_D | SEG_F | SEG_G),
    FONT_6 = (SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),
    FONT_7 = (SEG_A | SEG_B | SEG_C),
    FONT_8 = (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),
    FONT_9 = (SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G),
    FONT_MINUS = SEG_G,
    FONT_E = 0x00,
    FONT_H = 0x00,
    FONT_L = 0x00,
    FONT_P = 0x00,
    FONT_SPACE = 0x00
};

static const uint8_t font[] = {
    FONT_0, FONT_1, FONT_2, FONT_3, FONT_4, FONT_5, FONT_6, FONT_7, FONT_8, FONT_9,
    FONT_MINUS, FONT_E, FONT_H, FONT_L, FONT_P, FONT_SPACE
};

static void pabort(const char *s)
{
    perror(s);
    abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8; //16 - for odroid; 8 - for rpi ((
static uint32_t speed = 5000000;
static uint16_t delay;

static void Send_7219(int fd, uint8_t reg, uint8_t val)
{
    int ret;
    uint8_t tx[] = {
        reg, val
    };
    uint8_t rx[ARRAY_SIZE(tx)] = {0, };
    struct spi_ioc_transfer tr = {
        (unsigned long)tx, // .tx_buf
        (unsigned long)rx, // .rx_buf
        ARRAY_SIZE(tx),       // .len
        delay,        // .delay_usecs
        speed,           // .speed_hz
        bits,       // .bits_per_word
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        pabort("can't send spi message");
#if 0
    for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
        if (!(ret % 6))
            puts("");
        printf("%.2X ", rx[ret]);
    }
    puts("");
#endif
}

static void Clear_7219(int fd)
{
    for(int i=0; i<8; i++) {
        Send_7219(fd, i+1, 0x0F);
    }
}


static void print_usage(const char *prog)
{
    printf("Usage: %s [-DsbdlHOLC3]\n", prog);
    puts("  -D --device   device to use (default /dev/spidev1.1)\n"
         "  -s --speed    max speed (Hz)\n"
         "  -d --delay    delay (usec)\n"
         "  -b --bpw      bits per word \n"
         "  -l --loop     loopback\n"
         "  -H --cpha     clock phase\n"
         "  -O --cpol     clock polarity\n"
         "  -L --lsb      least significant bit first\n"
         "  -C --cs-high  chip select active high\n"
         "  -3 --3wire    SI/SO signals shared\n");
    exit(1);
}

static void parse_opts(int argc, char *argv[])
{
    while (1) {
        static const struct option lopts[] = {
            { "device",  1, 0, 'D' },
            { "speed",   1, 0, 's' },
            { "delay",   1, 0, 'd' },
            { "bpw",     1, 0, 'b' },
            { "loop",    0, 0, 'l' },
            { "cpha",    0, 0, 'H' },
            { "cpol",    0, 0, 'O' },
            { "lsb",     0, 0, 'L' },
            { "cs-high", 0, 0, 'C' },
            { "3wire",   0, 0, '3' },
            { "no-cs",   0, 0, 'N' },
            { "ready",   0, 0, 'R' },
            { NULL, 0, 0, 0 },
        };
        int c;

        c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

        if (c == -1)
            break;

        switch (c) {
        case 'D':
            device = optarg;
            break;
        case 's':
            speed = atoi(optarg);
            break;
        case 'd':
            delay = atoi(optarg);
            break;
        case 'b':
            bits = atoi(optarg);
            break;
        case 'l':
            mode |= SPI_LOOP;
            break;
        case 'H':
            mode |= SPI_CPHA;
            break;
        case 'O':
            mode |= SPI_CPOL;
            break;
        case 'L':
            mode |= SPI_LSB_FIRST;
            break;
        case 'C':
            mode |= SPI_CS_HIGH;
            break;
        case '3':
            mode |= SPI_3WIRE;
            break;
        case 'N':
            mode |= SPI_NO_CS;
            break;
        case 'R':
            mode |= SPI_READY;
            break;
        default:
            print_usage(argv[0]);
            break;
        }
    }
}

static inline uint8_t to_font_7219(char ch) {
    switch(ch) {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            return (ch - '0');
            return (ch - 'a' + 0x0A);
        case ':':
        case '-':
            return 0x0A;
        case '.':
        case ',':
            return 0x80;
        default:
            return 0x0F;
    }
}

static const char* convert_to_7219(uint8_t* buf, size_t size, const char* sp)
{
    for(uint8_t* bp = buf; bp != (buf + size); ++bp) {
        if( *sp == '\0' )
            break;
        uint8_t ch = to_font_7219(*sp++);
        if(ch == 0x80) {
            if(!sp)
                break;
            if(bp != buf) {
                bp[-1] |= ch;
                ch = to_font_7219(*sp++);
            }
        }
        *bp = ch;
    }
    return sp;
}

static void initialize(int fd)
{
    Send_7219(fd, 0x09, 0xFF);//включим режим декодирования
    Send_7219(fd, 0x0B, 0x07);//кол-во используемых разрядов
    Send_7219(fd, 0x0A, 0x04);//интенсивность свечения
    Send_7219(fd, 0x0C, 0x01);//включим индикатор

    Clear_7219(fd);
}

static void demo(int fd);

int main(int argc, char *argv[])
{
    int ret = 0;
    int fd;

    parse_opts(argc, argv);

    fd = open(device, O_RDWR);
    if (fd < 0)
        pabort("can't open device");

    /*
     * spi mode
     */
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
        pabort("can't set spi mode");

    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
        pabort("can't get spi mode");

    /*
     * bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't set bits per word");

    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        pabort("can't get bits per word");

    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't set max speed hz");

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
        pabort("can't get max speed hz");

    printf("spi mode: %d\n", mode);
    printf("bits per word: %d\n", bits);
    printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

    initialize(fd);

    Send_7219(fd, 0x0F, 0x01);
    usleep(100000);
    Send_7219(fd, 0x0F, 0x00);
    usleep(100000);

    demo(fd);
    demo(fd);
    demo(fd);
    demo(fd);
    demo(fd);
    demo(fd);
    demo(fd);

    struct timeval tv;
    struct tm tm;
    uint8_t buf[8]= {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
    uint8_t buf2[8] = {0x00, };
    uint8_t* bufs[2] = { buf, buf2 };
    int m = 0;
    char str[30];

    for(int n = 0; ; n++) {
        gettimeofday(&tv, NULL);
        tv.tv_sec += 3*60*60;
        localtime_r(&tv.tv_sec, &tm);

        uint8_t* front = bufs[(n+0) % 2];
        uint8_t* back =  bufs[(n+1) % 2];

        if( m != tm.tm_min ) {
            initialize(fd);
            demo(fd);
            memset(back, 0x0F, 8);
            memset(front, 0x0F, 8);
            m = tm.tm_min;
        }

        if( tv.tv_usec < 500000 ) {
            strftime(str, sizeof(str), "%H %M-%S", &tm);
        }
        else {
            strftime(str, sizeof(str), "%H-%M %S", &tm);
        }
        convert_to_7219(front, 8, str);

        for(int i = 0; i<8; i++) {
            if( front[i] != back[i] ) {
                Send_7219(fd, 8 - i, front[i]);
            }
        }
        usleep(20000);
    }

    close(fd);

    return ret;
}

struct demo_prg_t {
    int pos;
    int val;
    int delay;
};

typedef struct demo_prg_t demo_prg_t;

static const demo_prg_t prg1[] = {
/*
    { 8, SEG_E, 1 },
    { 8, SEG_F + SEG_E, 1 },
    { 8, SEG_F + SEG_A, 1 },
    { 8, SEG_A, 0 }, { 7, SEG_A, 1 },
    { 8, BLANK, 0 }, { 6, SEG_A, 1 },
    { 7, BLANK, 0 }, { 5, SEG_A, 1 },
    { 6, BLANK, 0 }, { 4, SEG_A, 1 },
    { 5, BLANK, 0 }, { 3, SEG_A, 1 },
    { 4, BLANK, 0 }, { 2, SEG_A, 1 },
    { 3, BLANK, 0 }, { 1, SEG_A, 1 },
    { 2, BLANK, 0 }, { 1, SEG_A + SEG_B, 1 },
    { 1, SEG_B + SEG_C, 1 },
    { 1, SEG_C + SEG_D, 1 },
    { 1, SEG_D, 0 }, { 2, SEG_D, 1 },
    { 1, BLANK, 0 }, { 3, SEG_D, 1 },
    { 2, BLANK, 0 }, { 4, SEG_D, 1 },
    { 3, BLANK, 0 }, { 5, SEG_D, 1 },
    { 4, BLANK, 0 }, { 6, SEG_D, 1 },
    { 5, BLANK, 0 }, { 7, SEG_D, 1 },
    { 6, BLANK, 0 }, { 8, SEG_D, 1 },
    { 7, BLANK, 0 }, { 8, SEG_D + SEG_E, 1 },
    { 8, SEG_E + SEG_F, 1 },
 */
    { 8, SEG_E, 1 },
    { 8, SEG_E + SEG_F, 1 },
    { 8, SEG_E + SEG_F + SEG_A, 1 },

    { 8, SEG_E + SEG_F + SEG_A, 1 },
    { 7, SEG_A, 1 },
    { 6, SEG_A, 1 },
    { 5, SEG_A, 1 },
    { 4, SEG_A, 1 },
    { 3, SEG_A, 1 },
    { 2, SEG_A, 1 },
    { 1, SEG_A, 1 },
    { 1, SEG_A + SEG_B, 1 },
    { 1, SEG_A + SEG_B + SEG_C, 1 },
    { 1, SEG_A + SEG_B + SEG_C + SEG_D, 1 },
    { 2, SEG_A + SEG_D, 1 },
    { 3, SEG_A + SEG_D, 1 },
    { 4, SEG_A + SEG_D, 1 },
    { 5, SEG_A + SEG_D, 1 },
    { 6, SEG_A + SEG_D, 1 },
    { 7, SEG_A + SEG_D, 1 },
    { 8, SEG_A + SEG_D + SEG_E + SEG_F, 1 },

    { 8, SEG_D + SEG_F + SEG_A, 1 },
    { 8, SEG_D + SEG_A, 1 },
    { 8, SEG_D, 1 },
    { 7, SEG_D, 1 },
    { 6, SEG_D, 1 },
    { 5, SEG_D, 1 },
    { 4, SEG_D, 1 },
    { 3, SEG_D, 1 },
    { 2, SEG_D, 1 },
    { 1, SEG_B + SEG_C + SEG_D, 1 },
    { 1, SEG_C + SEG_D, 1 },
    { 1, SEG_D, 1 },
    { 1, BLANK, 1 },
    { 2, BLANK, 1 },
    { 3, BLANK, 1 },
    { 4, BLANK, 1 },
    { 5, BLANK, 1 },
    { 6, BLANK, 1 },
    { 7, BLANK, 1 },
    { 8, BLANK, 1 }
};

static void demo(int fd)
{
    Send_7219(fd, 0x09, 0x00);
    for(int i = 0; i < 8; i++)
        Send_7219(fd, i + 1, 0x00);

    for(int i = 0; i < ARRAY_SIZE(prg1); i++) {
        Send_7219(fd, prg1[i].pos, prg1[i].val);
        if( prg1[i].delay ) {
            usleep(20000);
        }
    }
    Send_7219(fd, 0x09, 0xFF);
}

