#include "MarvinSDK.h"
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {
int failure_at = 0, operation_count = 0, release_count = 0, frame = 0;
long error_code = -1;
bool incorrect_value = false;

long operation() {
    return ++operation_count == failure_at ? error_code : 0;
}
} // namespace

int parameter_test_sleep(useconds_t) { return 0; }
#define usleep parameter_test_sleep
#define main parameter_example_main
#include "../c++_win/get_set_param_demo.cpp"
#undef main
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
long OnGetIntPara(char*, long* value) {
    const long result = operation();
    if (!result) *value = 42;
    return result;
}
long OnGetFloatPara(char*, double* value) {
    const long result = operation();
    if (!result) *value = 1.25;
    return result;
}
long OnSetIntPara(char*, long value) {
    incorrect_value |= value != 42;
    return operation();
}
long OnSetFloatPara(char*, double value) {
    incorrect_value |= value != 1.25;
    return operation();
}
long OnSavePara() { return operation(); }
bool OnRelease() { ++release_count; return true; }
}

void verify(int failed_step, long status) {
    failure_at = failed_step;
    error_code = status;
    operation_count = release_count = frame = 0;
    incorrect_value = false;
    const int result = parameter_example_main();
    const int expected_operations = failed_step ? failed_step : 5;
    if (operation_count != expected_operations || release_count != 1 || incorrect_value ||
        (result < 0) != (failed_step != 0))
        throw std::runtime_error("Parameter failure did not stop subsequent operations or release correctly");
}

int main() {
    try {
        verify(0, 0);
        const long errors[] = {-2, -1, 1};
        for (int step = 1; step <= 5; ++step)
            for (long error : errors) verify(step, error);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "PASS all 16 parameter-operation scenarios\n";
    return 0;
}
