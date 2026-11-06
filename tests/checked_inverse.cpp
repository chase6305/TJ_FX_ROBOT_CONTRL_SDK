#include "FxRobot.h"
#include <stdlib.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct Rejected { int status; };
struct Accepted {};
Vect7 input = {};
std::vector<char> model_path;
bool first_forward = true, inverse_called = false, raw_matches = false;
double max_position_error = 0, max_rotation_error = 0;
} // namespace

FX_BOOL checked_test_load(FX_CHAR*, FX_INT32L types[2], FX_DOUBLE gravity[2][3],
                         FX_DOUBLE dh[2][8][4], FX_DOUBLE limits[2][7][4], FX_DOUBLE bounds[2][4][3],
                         FX_DOUBLE mass[2][7], FX_DOUBLE mcp[2][7][3], FX_DOUBLE inertia[2][7][6]) {
    return LOADMvCfg(model_path.data(), types, gravity, dh, limits, bounds, mass, mcp, inertia);
}

FX_BOOL checked_test_forward(FX_INT32L serial, FX_DOUBLE joints[7], Matrix4 pose) {
    if (first_forward) {
        std::memcpy(joints, input, sizeof(Vect7));
        first_forward = false;
    }
    return FX_Robot_Kine_FK(serial, joints, pose);
}

FX_BOOL checked_test_inverse(FX_INT32L serial, FX_InvKineSolvePara* solve) {
    inverse_called = true;
    const FX_BOOL result = FX_Robot_Kine_IK(serial, solve);
    raw_matches = result != FX_FALSE;
    for (int joint = 0; joint < 7; ++joint)
        raw_matches = raw_matches && std::isfinite(solve->m_Output_RetJoint[joint]);
    if (raw_matches) {
        Matrix4 actual = {};
        raw_matches = FX_Robot_Kine_FK(serial, solve->m_Output_RetJoint, actual) != FX_FALSE;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 4; ++column) {
                const double error = std::abs(actual[row][column] - solve->m_Input_IK_TargetTCP[row][column]);
                if (!std::isfinite(error) || error > (column == 3 ? 1e-3 : 1e-6)) raw_matches = false;
                if (column == 3) max_position_error = std::fmax(max_position_error, error);
                else max_rotation_error = std::fmax(max_rotation_error, error);
            }
        }
    }
    return result;
}

FX_BOOL checked_test_next_phase(FX_INT32L, FX_InvKineSolvePara*) { throw Accepted{}; }
void checked_test_exit(int status) { throw Rejected{status}; }

// Supply model/joint inputs and observe the phase transition; the real library
// still performs FK, IK and Jacobian calculations. No planner is reached.
#define LOADMvCfg checked_test_load
#define FX_Robot_Kine_FK checked_test_forward
#define FX_Robot_Kine_IK checked_test_inverse
#define FX_Robot_Kine_IK_NSP checked_test_next_phase
#define exit checked_test_exit
#define main checked_example_main
#include "../c++_win/my_dd.cpp"
#undef main
#undef exit
#undef FX_Robot_Kine_IK_NSP
#undef FX_Robot_Kine_IK
#undef FX_Robot_Kine_FK
#undef LOADMvCfg

int main(int argc, char** argv) {
    const std::string path = argc > 1 ? argv[1] : "c++_win/srs.MvKDCfg";
    model_path.assign(path.begin(), path.end());
    model_path.push_back('\0');
    const double poses[][7] = {
        {10, 10, 10, 10, 10, 10, 10},
        {68.21370044474475, 14.249340812534669, -39.56439238106451, -12.450462371680686,
         5.53976944144388, -0.02323262631989742, -3.668394484121661},
        {-61.53455479196488, 20.3240464454627, 21.53559577951269, -11.792535724741228,
         67.90272049815118, 0.0013752369941357756, -14.980597603604885}
    };
    try {
        for (const auto& pose : poses) {
            std::memcpy(input, pose, sizeof(Vect7));
            first_forward = true;
            inverse_called = raw_matches = false;
            max_position_error = max_rotation_error = 0;
            bool advanced = false;
            try {
                RobotKineDemo();
                throw std::runtime_error("Example ran past the monitored inverse phase");
            } catch (const Accepted&) {
                advanced = true;
            } catch (const Rejected& failure) {
                if (failure.status != EXIT_FAILURE) throw std::runtime_error("Unexpected error status");
            }
            if (!inverse_called || advanced != raw_matches)
                throw std::runtime_error("Example acceptance disagrees with real-library FK residuals");
            std::cout << "PASS inverse accepted=" << advanced << " position_mm=" << max_position_error
                      << " rotation=" << max_rotation_error << '\n';
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
