#include "MarvinSDK.h"
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {
enum Scenario { Ready, Idle, Transition, WrongImpedance, ServoError, FailedRead };
Scenario scenario = Ready;
int mode = 0, impedance = 0, targets = 0, releases = 0, frame = 0;
}

int mode_test_usleep(useconds_t) { return 0; }
unsigned int mode_test_sleep(unsigned int) { return 0; }
#define usleep mode_test_usleep
#define sleep mode_test_sleep
#define main mode_example_main
#include MODE_SOURCE
#undef main
#undef sleep
#undef usleep

extern "C" {
bool OnLinkTo(FX_UCHAR, FX_UCHAR, FX_UCHAR, FX_UCHAR) { return true; }
bool OnClearSet() { return true; }
void OnClearErr_A() {}
void OnClearErr_B() {}
bool OnSetSend() { return true; }
bool OnSetTargetState_A(int value) { mode = value; return true; }
bool OnSetImpType_A(int value) { impedance = value; return true; }
bool OnGetBuf(DCSS* value) {
    std::memset(value, 0, sizeof(*value));
    value->m_Out[0].m_OutFrameSerial = ++frame;
    value->m_State[0].m_CurState = mode;
    value->m_In[0].m_ImpType = impedance;
    if (mode) {
        if (scenario == Idle) value->m_State[0].m_CurState = ARM_STATE_IDLE;
        if (scenario == Transition) value->m_State[0].m_CurState = ARM_STATE_TRANS_TO_TORQ;
        if (scenario == WrongImpedance) value->m_In[0].m_ImpType = impedance == 1 ? 2 : 1;
        if (scenario == ServoError) value->m_State[0].m_ERRCode = 42;
        // Even plausible output fields must not override a failed read status.
        if (scenario == FailedRead) return false;
    }
    return true;
}
bool OnSetJointLmt_A(int, int) { return true; }
bool OnSetJointKD_A(double*, double*) { return true; }
bool OnSetCartKD_A(double*, double*, int) { return true; }
bool OnSetForceCtrPara_A(int, double*, double*, double) { return true; }
bool OnSetForceCmd_A(double) { return true; }
bool OnSetJointCmdPos_A(double*) { ++targets; return true; }
bool OnRelease() { ++releases; return true; }
}

void verify(Scenario value) {
    scenario = value;
    mode = impedance = targets = releases = frame = 0;
    const int result = mode_example_main();
    const bool ready = value == Ready;
    if (releases != 1 || targets != (ready ? 1 : 0) || (ready ? result < 0 : result != -1))
        throw std::runtime_error("Joint target or release disagrees with feedback validity");
}

int main() {
    try {
        const Scenario cases[] = {Ready, Idle, Transition, ServoError, FailedRead};
        for (Scenario value : cases) verify(value);
#ifndef POSITION_EXAMPLE
        verify(WrongImpedance);
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "PASS joint-command mode guards\n";
    return 0;
}
