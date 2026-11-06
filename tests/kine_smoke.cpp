#include "FxRobot.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
const double radians = 0.01745329251994329577;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void multiply(const Matrix4 a, const Matrix4 b, Matrix4 result) {
    Matrix4 product = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                product[i][j] += a[i][k] * b[k][j];
    std::memcpy(result, product, sizeof(Matrix4));
}

void reference_fk(const double dh[8][4], const Vect7 joints, Matrix4 result) {
    std::memset(result, 0, sizeof(Matrix4));
    for (int i = 0; i < 4; ++i) result[i][i] = 1;
    for (int axis = 0; axis < 8; ++axis) {
        const double alpha = dh[axis][0] * radians;
        const double angle = (dh[axis][3] + (axis < 7 ? joints[axis] : 0)) * radians;
        const double ca = std::cos(alpha), sa = std::sin(alpha);
        const double ct = std::cos(angle), st = std::sin(angle);
        // Modified DH: Rx(alpha) Tx(a) Rz(theta + q) Tz(d).
        Matrix4 transform = {
            {ct, -st, 0, dh[axis][1]},
            {st * ca, ct * ca, -sa, -dh[axis][2] * sa},
            {st * sa, ct * sa, ca, dh[axis][2] * ca},
            {0, 0, 0, 1}
        };
        multiply(result, transform, result);
    }
}

void close(double actual, double expected, double tolerance, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

void check_arm(long serial, double dh[8][4]) {
    Vect7 joints = {10, 10, 10, 10, 10, 10, 10};
    Matrix4 target = {}, reference = {};
    require(FX_Robot_Kine_FK(serial, joints, target), "FK failed");
    reference_fk(dh, joints, reference);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            close(target[i][j], reference[i][j], 1e-6, "FK differs from model DH");

    FX_InvKineSolvePara solve = {};
    std::memcpy(solve.m_Input_IK_TargetTCP, target, sizeof(Matrix4));
    std::memcpy(solve.m_Input_IK_RefJoint, joints, sizeof(Vect7));
    require(FX_Robot_Kine_IK(serial, &solve), "IK failed for nominal pose");
    Matrix4 returned = {};
    require(FX_Robot_Kine_FK(serial, solve.m_Output_RetJoint, returned), "IK output FK failed");
    for (int i = 0; i < 3; ++i) {
        close(returned[i][3], target[i][3], 1e-3, "IK position round trip differs");
        for (int j = 0; j < 3; ++j)
            close(returned[i][j], target[i][j], 1e-6, "IK orientation round trip differs");
    }

    FX_Jacobi jacobian = {};
    require(FX_Robot_Kine_Jacb(serial, joints, &jacobian), "Jacobian failed");
    require(jacobian.m_AxisNum == 7, "Unexpected Jacobian axis count");
    const double delta = 1e-3;
    for (int axis = 0; axis < 7; ++axis) {
        Vect7 plus = {}, minus = {};
        std::memcpy(plus, joints, sizeof(Vect7));
        std::memcpy(minus, joints, sizeof(Vect7));
        plus[axis] += delta;
        minus[axis] -= delta;
        Matrix4 positive = {}, negative = {};
        require(FX_Robot_Kine_FK(serial, plus, positive), "Positive finite difference FK failed");
        require(FX_Robot_Kine_FK(serial, minus, negative), "Negative finite difference FK failed");
        for (int i = 0; i < 3; ++i) {
            const double numerical = (positive[i][3] - negative[i][3]) / (2 * delta);
            // FK positions use mm; the Jacobian's linear rows use m/rad.
            close(numerical, jacobian.m_Jcb[i][axis] * 1000 * radians, 1e-5,
                  "Linear Jacobian differs from FK finite difference");
        }
        double angular[3][3] = {};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    angular[i][j] += (positive[i][k] - negative[i][k]) / (2 * delta) * target[j][k];
        const double omega[3] = {(angular[2][1] - angular[1][2]) / 2,
                                 (angular[0][2] - angular[2][0]) / 2,
                                 (angular[1][0] - angular[0][1]) / 2};
        for (int i = 0; i < 3; ++i)
            close(omega[i], jacobian.m_Jcb[i + 3][axis] * radians, 1e-7,
                  "Angular Jacobian differs from FK finite difference");
    }
}
} // namespace

int main(int argc, char** argv) {
    const std::string root = argc > 1 ? argv[1] : "c++_win";
    const char* models[] = {"ccs_m3.MvKDCfg", "ccs_m6.MvKDCfg", "srs.MvKDCfg"};
    try {
        for (const char* model : models) {
            const std::string path = root + "/" + model;
            std::vector<char> writable_path(path.begin(), path.end());
            writable_path.push_back('\0');
            FX_INT32L types[2] = {};
            double gravity[2][3] = {}, dh[2][8][4] = {}, limits[2][7][4] = {}, bounds[2][4][3] = {};
            double mass[2][7] = {}, mcp[2][7][3] = {}, inertia[2][7][6] = {};
            require(LOADMvCfg(writable_path.data(), types, gravity, dh, limits, bounds, mass, mcp, inertia),
                    "Model configuration load failed");
            for (int arm = 0; arm < 2; ++arm) {
                require(FX_Robot_Init_Type(arm, types[arm]), "Robot type initialization failed");
                require(FX_Robot_Init_Kine(arm, dh[arm]), "Robot DH initialization failed");
                require(FX_Robot_Init_Lmt(arm, limits[arm], bounds[arm]), "Robot limits initialization failed");
                check_arm(arm, dh[arm]);
                std::cout << "PASS " << model << " arm " << arm << '\n';
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "Kine smoke check failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
