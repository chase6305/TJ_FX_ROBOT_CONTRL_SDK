#include "MarvinSDK.h"
#include <unistd.h>

#include <cstdarg>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
int receive_count = 0, poll_waits = 0, releases = 0, frame = 0;
int stop_signal = SIGINT;
bool receiving = false, invalid_wait = false;
std::string printed;
const long lengths[] = {-1, 0, 1, 8, 170, 171, 256, 257};
}

int eef_test_usleep(useconds_t delay) {
    if (receiving) {
        invalid_wait |= delay != 200000;
        ++poll_waits;
    }
    return 0;
}
unsigned int eef_test_sleep(unsigned int) {
    invalid_wait |= receiving;
    return 0;
}
int eef_test_printf(const char* format, ...) {
    char buffer[2048];
    va_list args;
    va_start(args, format);
    const int length = std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0 || length >= static_cast<int>(sizeof(buffer)))
        throw std::runtime_error("Unexpectedly large console record");
    printed.append(buffer, static_cast<std::size_t>(length));
    return length;
}

#define usleep eef_test_usleep
#define sleep eef_test_sleep
#define printf eef_test_printf
#define main eef_example_main
#include EEF_SOURCE
#undef main
#undef printf
#undef sleep
#undef usleep

extern "C" {
bool OnLinkTo(FX_UCHAR, FX_UCHAR, FX_UCHAR, FX_UCHAR) { return true; }
bool OnClearSet() { return true; }
void OnClearErr_A() {}
void OnClearErr_B() {}
bool OnSetSend() { return true; }
bool OnGetBuf(DCSS* value) {
    std::memset(value, 0, sizeof(*value));
    value->m_Out[0].m_OutFrameSerial = ++frame;
    return true;
}
bool OnSetJointLmt_A(int, int) { return true; }
bool OnSetTargetState_A(int) { return true; }
bool OnClearChDataA() { return true; }
bool OnSetChDataA(unsigned char*, long, long) { return true; }
long OnGetChDataA(unsigned char* data, long* channel) {
    receiving = true;
    const int index = receive_count++;
    if (index == static_cast<int>(sizeof(lengths) / sizeof(lengths[0]))) {
        std::raise(stop_signal);
        return 0;
    }
    const long length = lengths[index];
    *channel = 1;
    for (int i = 0; i < length && i < 256; ++i)
        data[i] = static_cast<unsigned char>(i);
    return length;
}
bool OnRelease() { ++releases; return true; }
}

void verify(int signal) {
    receive_count = poll_waits = releases = frame = 0;
    receiving = invalid_wait = false;
    printed.clear();
    stop_requested = 0;
    stop_signal = signal;
    const int result = eef_example_main();
    if (result != 0 || receive_count != 9 || poll_waits != 9 ||
        invalid_wait || releases != 1)
        throw std::runtime_error("Incorrect polling interval or shutdown lifecycle");

    std::string expected;
    for (long length : lengths) {
        if (length < 1 || length > 256) continue;
        expected += "接收信号: " + std::to_string(length) + ", 接收的HEX数据: ";
        for (int byte = 0; byte < length; ++byte) {
            char hex[4];
            std::snprintf(hex, sizeof(hex), "%02X", byte);
            if (byte) expected += ' ';
            expected += hex;
        }
        expected += '\n';
    }
    const std::size_t first_record = printed.find("接收信号: ");
    if (first_record == std::string::npos || printed.substr(first_record) != expected)
        throw std::runtime_error("Received frames were truncated or invalid lengths were printed");
}

int main() {
    try {
        verify(SIGINT);
        verify(SIGTERM);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "PASS 8 receive lengths and both stop signals\n";
    return 0;
}
