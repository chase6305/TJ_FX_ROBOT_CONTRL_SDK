#include "MarvinSDK.h"
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
typedef std::pair<std::string, long> Command;
int failure_at = 0, failed_feedback = 0, reads = 0, releases = 0;
long failure_code = -1;
std::vector<Command> commands;
#ifdef TEST_BRAKE
const std::vector<Command> expected = {
    {"BRAK0", 2}, {"BRAK0", 1}, {"BRAK1", 2}, {"BRAK1", 1}
};
#else
const std::vector<Command> expected = {
    {"RESETMOTENC0", 0}, {"RESETMOTENC1", 0},
    {"RESETEXTENC0", 0}, {"RESETEXTENC1", 0},
    {"CLEARMOTENC0", 0}, {"CLEARMOTENC1", 0}
};
#endif
}

int maintenance_test_usleep(useconds_t) { return 0; }
unsigned int maintenance_test_sleep(unsigned int) { return 0; }
#define usleep maintenance_test_usleep
#define sleep maintenance_test_sleep
#define main maintenance_example_main
#ifdef TEST_BRAKE
#include "../c++_win/apply-brake_release-brake_demo.cpp"
#else
#include "../c++_win/motor_encoder_clear_demo.cpp"
#endif
#undef main
#undef sleep
#undef usleep

extern "C" {
bool OnLinkTo(FX_UCHAR, FX_UCHAR, FX_UCHAR, FX_UCHAR) { return true; }
bool OnClearSet() { return true; }
void OnClearErr_A() {}
void OnClearErr_B() {}
bool OnSetSend() { return true; }
bool OnSetTargetState_A(int) { return true; }
bool OnSetTargetState_B(int) { return true; }
bool OnGetBuf(DCSS* value) {
    ++reads;
    // A failed read deliberately leaves the previous disabled-arm state intact.
    if (failed_feedback && reads == 5 + failed_feedback) return false;
    std::memset(value, 0, sizeof(*value));
    value->m_Out[0].m_OutFrameSerial = reads;
    return true;
}
long OnSetIntPara(char* name, long value) {
    commands.push_back(Command(name, value));
    return static_cast<int>(commands.size()) == failure_at ? failure_code : 0;
}
bool OnRelease() { ++releases; return true; }
}

void verify(int failed_operation, long error_code, int failed_read_stage = 0) {
    failure_at = failed_operation;
    failure_code = error_code;
    failed_feedback = failed_read_stage;
    reads = releases = 0;
    commands.clear();
    const int result = maintenance_example_main();
    const std::size_t expected_count = failed_read_stage
        ? static_cast<std::size_t>((failed_read_stage - 1) * 2)
        : (failed_operation ? static_cast<std::size_t>(failed_operation) : expected.size());
    const bool failed = failed_operation || failed_read_stage;
    if (releases != 1 || commands.size() != expected_count || (result < 0) != failed)
        throw std::runtime_error("Maintenance did not stop or release at the failing operation");
    for (std::size_t i = 0; i < commands.size(); ++i)
        if (commands[i] != expected[i])
            throw std::runtime_error("Maintenance changed a parameter, value or operation order");
    if (failed_read_stage && reads != 5 + failed_read_stage)
        throw std::runtime_error("Maintenance continued reading after failed feedback");
}

int main() {
    try {
        verify(0, 0);
        const long errors[] = {-2, -1, 1};
        for (int step = 1; step <= static_cast<int>(expected.size()); ++step)
            for (long error : errors) verify(step, error);
#ifndef TEST_BRAKE
        for (int stage = 1; stage <= 3; ++stage) verify(0, 0, stage);
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "PASS maintenance failure and operation-order scenarios\n";
    return 0;
}
