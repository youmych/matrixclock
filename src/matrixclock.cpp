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

#include <array>
#include <iostream>
#include <algorithm>
#include <system_error>
#include <cerrno>
#include <cassert>
#include <vector>
#include <iostream>
#include <iterator>
#include <bitset>
#include <chrono>
#include <thread>
#include <random>

namespace MAX7219 {

/// Имена регистров и их адреса
enum Reg : uint8_t {
    NOP         = 0x00,

    DIGIT_0     = 0x01,
    DIGIT_1     = 0x02,
    DIGIT_2     = 0x03,
    DIGIT_3     = 0x04,
    DIGIT_4     = 0x05,
    DIGIT_5     = 0x06,
    DIGIT_6     = 0x07,
    DIGIT_7     = 0x08,

    DECODE_MODE = 0x09,
    INTENSITY   = 0x0A,
    SCAN_LIMIT  = 0x0B,
    SHUTDOWN    = 0x0C,

    DISPLAY_TEST = 0x0F
};

/// Преобразование констант в строку
const char* to_string(Reg r) {
    #define TO_STRING( x ) case x: return #x;

    switch( r ) {
        TO_STRING(NOP);

        TO_STRING(DIGIT_0);
        TO_STRING(DIGIT_1);
        TO_STRING(DIGIT_2);
        TO_STRING(DIGIT_3);
        TO_STRING(DIGIT_4);
        TO_STRING(DIGIT_5);
        TO_STRING(DIGIT_6);
        TO_STRING(DIGIT_7);

        TO_STRING(DECODE_MODE);
        TO_STRING(INTENSITY);
        TO_STRING(SCAN_LIMIT);
        TO_STRING(SHUTDOWN);

        TO_STRING(DISPLAY_TEST);

        default: return "";
    }

    #undef TO_STRING
}

constexpr auto make_transformer(uint8_t regAddr)
{
    return [=](uint8_t data) -> uint16_t {
        union {
            uint8_t rd[2];
            uint16_t res;
        } t;
        t.rd[0] = regAddr;
        t.rd[1] = data;
        return t.res;
    };
}

constexpr auto make_cmd(uint8_t reg, uint8_t cmd)
{
    union T {
        uint8_t rd[2];
        uint16_t res;
    } t = { reg, cmd };
    return [=]() constexpr -> uint16_t {
        return t.res;
    };
}

} // namespace MAX7219

std::ostream& operator<<(std::ostream& os, MAX7219::Reg r)
{
    return os << MAX7219::to_string(r);
}

class SPI {
public:
    static constexpr uint8_t  DEFAULT_BITS_PER_WORD = 8;
    static constexpr uint32_t DEFAULT_SPEED = 5000000;
    static constexpr uint8_t  DEFAULT_DELAY = 0;

    explicit SPI() : m_Fd(-1) {}
    explicit SPI(const std::string& device,
                 uint8_t mode,
                 uint8_t bpw = DEFAULT_BITS_PER_WORD,
                 uint32_t speed = DEFAULT_SPEED,
                 uint8_t delay = DEFAULT_DELAY)
        : m_Device(device), m_Mode(mode), m_Bits(bpw), m_Speed(speed), m_Delay(delay)
    {
        Open();
    }

    ~SPI() {
        if( IsOpen() )
            close(m_Fd);
    }

    void Open(const std::string& device,
              uint8_t mode,
              uint8_t bpw = DEFAULT_BITS_PER_WORD,
              uint32_t speed = DEFAULT_SPEED,
              uint8_t delay = DEFAULT_DELAY)
    {
        if( IsOpen() && m_Device == device && m_Mode == mode && m_Bits == bpw &&
            m_Speed == speed && m_Delay == delay)
        {
            return;
        }
        if( IsOpen() ) { close(m_Fd); m_Fd = -1; }
        m_Device = device;
        m_Mode = mode;
        m_Bits = bpw;
        m_Speed = speed;
        m_Delay = delay;
        Open();
    }

    void Transfer(const void* buf, size_t len) {
        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));
        tr.tx_buf = (unsigned long)buf; // .tx_buf
        tr.rx_buf = 0;                  // .rx_buf
        tr.len = len;                   // .len
        tr.delay_usecs = m_Delay;       // .delay_usecs
        tr.speed_hz = m_Speed;          // .speed_hz
        tr.bits_per_word = m_Bits;      // .bits_per_word

        int ret = ioctl(m_Fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
            throw std::system_error(errno, std::system_category(), "can't send spi message");
    }

    bool IsOpen() const { return m_Fd != -1; }

private:
    void Open() {
        m_Fd = open(m_Device.c_str(), O_RDWR);
        if (m_Fd < 0)
            throw std::system_error(errno, std::system_category(),
                std::string("can't open device ") + m_Device);

        // spi mode
        int ret = ioctl(m_Fd, SPI_IOC_WR_MODE, &m_Mode);
        if (ret == -1)
            throw std::system_error(errno, std::system_category(),
                std::string("can't set spi mode ") + std::to_string(m_Mode));

        ret = ioctl(m_Fd, SPI_IOC_RD_MODE, &m_Mode);
        if (ret == -1)
            throw std::system_error(errno, std::system_category(), "can't get spi mode");

        // bits per word
        ret = ioctl(m_Fd, SPI_IOC_WR_BITS_PER_WORD, &m_Bits);
        if (ret == -1)
            throw std::system_error(errno, std::system_category(), "can't set bits per word");

        ret = ioctl(m_Fd, SPI_IOC_RD_BITS_PER_WORD, &m_Bits);
        if (ret == -1)
            throw std::system_error(errno, std::system_category(), "can't get bits per word");

        // max speed hz
        ret = ioctl(m_Fd, SPI_IOC_WR_MAX_SPEED_HZ, &m_Speed);
        if (ret == -1)
            throw std::system_error(errno, std::system_category(), "can't set max speed hz");

        ret = ioctl(m_Fd, SPI_IOC_RD_MAX_SPEED_HZ, &m_Speed);
        if (ret == -1)
            throw std::system_error(errno, std::system_category(), "can't get max speed hz");
    }

private:
    int m_Fd = -1;
    std::string m_Device;
    uint8_t m_Mode;
    uint8_t m_Bits = 8; //16 - for odroid; 8 - for rpi ((
    uint32_t m_Speed = 5000000;
    uint8_t m_Delay = 0;
};

class Display {
public:
    /// Кол-во точек по горизонтали в одном сегменте
    static constexpr size_t SEG_X_POINTS = 8;
    /// Кол-во точек по вертикали в одном сегменте
    static constexpr size_t SEG_Y_POINTS = 8;
    /// кол-во последовательно подключенных микросхем
    static constexpr size_t IC_LINE_SIZE = 4;

    explicit Display(SPI& spi) : m_SPI(spi) {}
    //explicit Display() { Initialize(); }

    static void SendCmd(SPI& spi, MAX7219::Reg reg, uint8_t cmd) {
        std::array<uint16_t, IC_LINE_SIZE> renderBuf;
        std::generate(renderBuf.begin(), renderBuf.end(), MAX7219::make_cmd(reg, cmd));
        spi.Transfer(renderBuf.data(), renderBuf.size()*sizeof(uint16_t));
    }

    static void Initialize(SPI& spi) {
        SendCmd(spi, MAX7219::Reg::DECODE_MODE, 0x00);//выключим режим декодирования
        SendCmd(spi, MAX7219::Reg::SCAN_LIMIT,  0x07);//кол-во используемых разрядов
        SendCmd(spi, MAX7219::Reg::INTENSITY,   0x01);//интенсивность свечения
        SendCmd(spi, MAX7219::Reg::SHUTDOWN,    0x01);//включим индикатор
    }

    void Initialize() {
        Initialize(m_SPI);
        Clear();
    }

    void Fill(uint8_t val = 0x00) {
        for(size_t i = 0; i <= SEG_Y_POINTS; ++i) {
            std::generate(m_RenderBuf.begin() + IC_LINE_SIZE * i,
                          m_RenderBuf.begin() + IC_LINE_SIZE * (i + 1),
                          MAX7219::make_cmd(MAX7219::DIGIT_0 + i, val));
        }
        Transfer();
    }

    void Clear() { Fill(); }

    void Transfer() {
        for(auto it = m_RenderBuf.begin(); it != m_RenderBuf.end(); std::advance(it, IC_LINE_SIZE)) {
            auto ofs = std::distance(m_RenderBuf.begin(), it);
            m_SPI.Transfer( &m_RenderBuf[ofs], IC_LINE_SIZE*sizeof(uint16_t) );
        }
    }

    template <class InputIt>
    void GrabLine(uint8_t displayLineNo, InputIt begin, InputIt end, uint8_t bitOfs) {
        using namespace MAX7219;

        bitOfs &= 0x07;
        displayLineNo &= 0x07;

        // максимальное смещение внутри буфера [begin, end)
        auto maxOfs = std::distance(begin, end);
        // всё, что выходит за границы входного буфера = 0
        auto paddedData = [=](decltype(maxOfs) ofs) -> uint8_t {
            if( ofs < maxOfs )
                return *(begin + ofs);
            return static_cast<uint8_t>(0);
        };
        // начало и конец буфера записи
        auto lbegin = std::next(m_RenderBuf.begin(), displayLineNo*IC_LINE_SIZE);
        auto lend = std::next(lbegin, IC_LINE_SIZE);
        // преобразование байта данных в команду MAX7219
        auto t = make_transformer(DIGIT_0 + displayLineNo);

        if( bitOfs == 0 ) {
            for( auto it = lbegin; it != lend; ++it ) {
                auto ofs = std::distance(lbegin, it);
                *it = t( paddedData(ofs) );
            }
        }
        else {
            for( auto it = lbegin; it != lend; ++it ) {
                auto ofs = std::distance(lbegin, it);
                *it = t( (paddedData(ofs) << bitOfs) + (paddedData(ofs+1) >> (8 - bitOfs)) );
            }
        }
    }

    void PrintBuf() {
        for(size_t i = 0; i < SEG_Y_POINTS; ++i ) {
            std::copy(m_RenderBuf.begin() + i*IC_LINE_SIZE,
                      m_RenderBuf.begin() + (i+1)*IC_LINE_SIZE,
                      std::ostream_iterator<std::bitset<16>>(std::cout, " "));
            std::cout << std::endl;
        }
    }

private:
    SPI& m_SPI;
    /// Буфер для рендеринга максимум 8 строк.
    /// MAX7219 соединены последовательно. Одна SPI-транзакция
    /// должна записать по одному значению в каждый из N регистров.
    /// Чтобы обновить все строки, нам нужно будет 8 транзакций.
    std::array<uint16_t, IC_LINE_SIZE*SEG_Y_POINTS> m_RenderBuf;
};

class VScreenRowIterator {
    using iterator = std::vector<uint8_t>::const_iterator;
    using difference_type = std::vector<uint8_t>::difference_type;
public:
    explicit VScreenRowIterator(iterator begin, difference_type line_size)
        : m_Begin(begin)
        , m_Length(line_size)
        {}

    auto operator*() const {
        return std::make_pair(m_Begin, std::next(m_Begin, m_Length));
    }

    VScreenRowIterator& operator++() {
        std::advance(m_Begin, m_Length);
        return *this;
    }

    bool operator != (const VScreenRowIterator& other) const {
        return m_Begin != other.m_Begin;
    }

private:
    iterator m_Begin;
    difference_type m_Length;
};


class VScreen {
public:
    using size_type = unsigned int;

    explicit VScreen(size_t w = 8*4, size_t h = 8)
        : m_Width(aligned(w))
        , m_Height(aligned(h))
        , m_Ddata(m_Width * m_Height / 8, 0)
    {}

    size_type width() const { return m_Width; }
    size_type height() const { return m_Height; }
    size_t sizeBytes() const { return m_Ddata.size(); }

    void putPixel(size_type x, size_type y, bool val = 1) {
        if( x >= width() || y >= height() )
            return;
        if( val )
            m_Ddata[ offset(x, y) ] |= bitMask(x % 8);
        else
            m_Ddata[ offset(x, y) ] &= ~bitMask(x % 8);
    }

    void clear() {
        std::fill(m_Ddata.begin(), m_Ddata.end(), 0);
    }

    void fill(uint8_t val) {
        std::fill(m_Ddata.begin(), m_Ddata.end(), val);
    }

    class Rows {
        friend class VScreen;

        const std::vector<uint8_t>& m_Data;
        size_type m_LineLenght;

        explicit Rows(const std::vector<uint8_t>& data, size_type lineLength)
            : m_Data(data), m_LineLenght(lineLength)
        {}
    public:

        Rows(const Rows&) = default;
        Rows(Rows&&) = default;

        VScreenRowIterator begin() const {
            return VScreenRowIterator(m_Data.cbegin(), m_LineLenght);
        }

        VScreenRowIterator end() const {
            return VScreenRowIterator(m_Data.cend(), 0);
        }
    };

    Rows rows() const {
        return Rows{this->m_Ddata, width() / 8};
    }

private:
    static constexpr uint8_t bitMask(uint8_t bit) {
        return (0x01 << (bit & 0x07));
    }

    auto rowBegin(size_type y) const {
        return m_Ddata.begin() + width()*y/8;
    }

    size_type offset(size_type x, size_type y) const {
        return (width() * y + x) / 8;
    }

    static size_type aligned(size_type sz) {
        return ((sz / 8) + (sz % 8 > 0 ? 1 : 0)) * 8;
    }

private:
    /// Ширина экрана в пикселях
    size_type m_Width;
    /// Высота экрана в пикселях
    size_type m_Height;
    /// Буфер экрана. Размер всегда кратен 8х8
    std::vector<uint8_t> m_Ddata;
};

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8; //16 - for odroid; 8 - for rpi ((
static uint32_t speed = 5000000;
static uint16_t delay;

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

static void print_screen(const VScreen& scr)
{
    for(auto [begin, end] : scr.rows()) {
        std::copy(begin, end, std::ostream_iterator<std::bitset<8>>(std::cout, " "));
        std::cout << std::endl;
    }
}

int main(int argc, char *argv[])
{
    VScreen scr/*(8*5)*/;

    std::random_device rd;
    std::uniform_int_distribution<int> distX(0, scr.width());   
    std::uniform_int_distribution<int> distY(0, scr.height());
    std::uniform_int_distribution<int> distB(0, 1);

    std::cout << "W = " << scr.width() << ", H = " << scr.height()
        << ", Bytes = " << scr.sizeBytes() << std::endl;
    print_screen(scr);
    //scr.fill(0x01);
    //print_screen(scr);

    std::cout << std::endl;
#if 0
    Display d;
    for( int t = 0; t < 8; ++t ) {
        auto it = scr.rows().begin();
        for(int i = 0; i < 8; ++i) {
            d.GrabLine(i, (*it).first, (*it).second, t);
            ++it;
        }
        d.PrintBuf();
        std::cout << std::endl;
    }

    return 0;
#endif
    parse_opts(argc, argv);

    printf("spi mode: %d\n", mode);
    printf("bits per word: %d\n", bits);
    printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

    SPI spi(device, mode, bits, speed, delay);
    Display d(spi);

    int n = 1000;

    while(--n) {
        scr.putPixel(distX(rd), distY(rd), distB(rd));
    //    for( int t = 0; t < 8; ++t ) {
            auto it = scr.rows().begin();
            for(int i = 0; i < 8; ++i) {
                d.GrabLine(i, (*it).first, (*it).second, 0);
                ++it;
            }
            d.PrintBuf();
            d.Transfer();
            std::cout << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        }
    }


    // for(auto [begin, end] : scr.rows()) {
    //     display.GrabLine(0, begin, end, 0);
    // }

    // initialize(fd);

    // Send_7219(fd, 0x0F, 0x01);
    // usleep(100000*10L);
    // Send_7219(fd, 0x0F, 0x00);
    // usleep(100000*10L);

    // std::array<uint16_t, 4> transform_buf;
    // std::array<uint8_t, 4> line1{ 0x55, 0x55, 0x55, 0x55 };
    // std::array<uint8_t, 4> line2{ 0xAA, 0xAA, 0xAA, 0xAA };

    // for(;;) {

    //     std::transform(line1.begin(), line1.end(), transform_buf.begin(), make_7219_transformer(0x01));
    //     for(auto& v: transform_buf) {
    //         std::cout << std::hex << v << ' ';
    //     }
    //     std::cout << std::endl;
    //     send_buf_7219(fd, transform_buf.data(), transform_buf.size()*sizeof(uint16_t));

    //     std::transform(line2.begin(), line2.end(), transform_buf.begin(), make_7219_transformer(0x02));
    //     for(auto& v: transform_buf) {
    //         std::cout << std::hex << v << ' ';
    //     }
    //     std::cout << std::endl;

    //     send_buf_7219(fd, transform_buf.data(), transform_buf.size()*sizeof(uint16_t));

    //     //Send_7219(fd, 0x01, 0x55);
    //     //Send_7219(fd, 0x02, 0xAA);
    //     Send_7219(fd, 0x03, 0x55);
    //     Send_7219(fd, 0x04, 0xAA);
    //     Send_7219(fd, 0x05, 0x55);
    //     Send_7219(fd, 0x06, 0xAA);
    //     Send_7219(fd, 0x07, 0x55);
    //     Send_7219(fd, 0x08, 0xAA);
    //     usleep(100000*10L);

    //     std::transform(line2.begin(), line2.end(), transform_buf.begin(), make_7219_transformer(0x01));
    //     send_buf_7219(fd, transform_buf.data(), transform_buf.size()*sizeof(uint16_t));

    //     std::transform(line1.begin(), line1.end(), transform_buf.begin(), make_7219_transformer(0x02));
    //     send_buf_7219(fd, transform_buf.data(), transform_buf.size()*sizeof(uint16_t));

    //     //Send_7219(fd, 0x01, 0xAA);
    //     //Send_7219(fd, 0x02, 0x55);
    //     Send_7219(fd, 0x03, 0xAA);
    //     Send_7219(fd, 0x04, 0x55);
    //     Send_7219(fd, 0x05, 0xAA);
    //     Send_7219(fd, 0x06, 0x55);
    //     Send_7219(fd, 0x07, 0xAA);
    //     Send_7219(fd, 0x08, 0x55);
    //     usleep(100000*10L);

    // }

    // demo(fd);
    // demo(fd);
    // demo(fd);
    // demo(fd);
    // demo(fd);
    // demo(fd);
    // demo(fd);

    // struct timeval tv;
    // struct tm tm;
    // uint8_t buf[8]= {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
    // uint8_t buf2[8] = {0x00, };
    // uint8_t* bufs[2] = { buf, buf2 };
    // int m = 0;
    // char str[30];

    // for(int n = 0; ; n++) {
    //     gettimeofday(&tv, NULL);
    //     tv.tv_sec += 3*60*60;
    //     localtime_r(&tv.tv_sec, &tm);

    //     uint8_t* front = bufs[(n+0) % 2];
    //     uint8_t* back =  bufs[(n+1) % 2];

    //     if( m != tm.tm_min ) {
    //         initialize(fd);
    //         demo(fd);
    //         memset(back, 0x0F, 8);
    //         memset(front, 0x0F, 8);
    //         m = tm.tm_min;
    //     }

    //     if( tv.tv_usec < 500000 ) {
    //         strftime(str, sizeof(str), "%H %M-%S", &tm);
    //     }
    //     else {
    //         strftime(str, sizeof(str), "%H-%M %S", &tm);
    //     }
    //     convert_to_7219(front, 8, str);

    //     for(int i = 0; i<8; i++) {
    //         if( front[i] != back[i] ) {
    //             Send_7219(fd, 8 - i, front[i]);
    //         }
    //     }
    //     usleep(20000);
    // }

    // close(fd);

    // return ret;
}

