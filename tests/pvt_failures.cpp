#include "MarvinSDK.h"
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {
enum Stage { Success, Upload, Gather, Clear, SetTrajectory, Send, Save };
enum Feedback { Valid, Idle, Transition, ServoError, FailedRead };
Stage failure = Success;
Feedback feedback = Valid;
int uploaded_id = -1, executed_id = -1, frame = 0, mode = 0;
int gathered = 0, executed = 0, saved = 0, released = 0;
int stopped = 0, run_cleared = 0, run_sent = 0;
bool gathering = false, run_pending = false;
}

int pvt_test_usleep(useconds_t) { return 0; }
unsigned int pvt_test_sleep(unsigned int) { return 0; }
#define usleep pvt_test_usleep
#define sleep pvt_test_sleep
#define main pvt_example_main
#include "../c++_win/pvt_demo.cpp"
#undef main
#undef sleep
#undef usleep

extern "C" {
bool OnLinkTo(FX_UCHAR, FX_UCHAR, FX_UCHAR, FX_UCHAR) { return true; }
bool OnClearSet() {
    if (gathering) { ++run_cleared; return failure != Clear; }
    return true;
}
void OnClearErr_A() {}
void OnClearErr_B() {}
bool OnSetSend() {
    if (run_pending) { ++run_sent; return failure != Send; }
    return true;
}
bool OnSetTargetState_A(int value) { mode = value; return true; }
bool OnGetBuf(DCSS* value) {
    std::memset(value, 0, sizeof(*value));
    value->m_Out[0].m_OutFrameSerial = ++frame;
    value->m_State[0].m_CurState = mode;
    if (mode == ARM_STATE_PVT) {
        if (feedback == Idle) value->m_State[0].m_CurState = ARM_STATE_IDLE;
        if (feedback == Transition) value->m_State[0].m_CurState = ARM_STATE_TRANS_TO_PVT;
        if (feedback == ServoError) value->m_State[0].m_ERRCode = 42;
        if (feedback == FailedRead) return false;
    }
    return true;
}
bool OnSendPVT_A(char*, long id) {
    uploaded_id = static_cast<int>(id);
    return failure != Upload;
}
bool OnStartGather(long, long*, long) {
    ++gathered;
    gathering = failure != Gather;
    return gathering;
}
bool OnSetPVT_A(int id) {
    ++executed;
    executed_id = id;
    run_pending = failure != SetTrajectory;
    return run_pending;
}
bool OnStopGather() { ++stopped; gathering = false; return true; }
bool OnSaveGatherData(char*) { ++saved; return failure != Save; }
bool OnRelease() { ++released; return true; }
}

void verify(Stage stage, Feedback state = Valid) {
    failure = stage;
    feedback = state;
    uploaded_id = executed_id = -1;
    frame = mode = gathered = executed = saved = released = stopped = run_cleared = run_sent = 0;
    gathering = run_pending = false;
    const int result = pvt_example_main();
    if (state != Valid) {
        if (result != -1 || released != 1 || uploaded_id != -1 || gathered || executed || saved || stopped)
            throw std::runtime_error("PVT advanced despite invalid mode feedback");
        return;
    }
    bool valid = released == 1 && uploaded_id > 0;
    if (stage == Success) {
        valid &= result >= 0 && gathered == 1 && executed == 1 && run_sent == 1 &&
                 saved == 1 && executed_id == uploaded_id && stopped == 0;
    } else {
        valid &= result == -1 && gathered == (stage > Upload ? 1 : 0) &&
                 run_cleared == (stage > Gather ? 1 : 0) &&
                 executed == (stage > Clear ? 1 : 0) &&
                 run_sent == (stage > SetTrajectory ? 1 : 0) &&
                 saved == (stage > Send ? 1 : 0) && stopped == (stage > Gather ? 1 : 0);
    }
    if (!valid) throw std::runtime_error("PVT continued after a failed stage or cleaned up incorrectly");
}

int main() {
    try {
        const Stage cases[] = {Success, Upload, Gather, Clear, SetTrajectory, Send, Save};
        for (Stage stage : cases) verify(stage);
        const Feedback states[] = {Idle, Transition, ServoError, FailedRead};
        for (Feedback state : states) verify(Success, state);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "PASS all 11 PVT mode/failure/success scenarios\n";
    return 0;
}
