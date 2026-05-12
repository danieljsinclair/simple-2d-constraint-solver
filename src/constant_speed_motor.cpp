#include "../include/constant_speed_motor.h"

#include <cmath>

atg_scs::ConstantSpeedMotor::ConstantSpeedMotor() {
    m_ks = 1.0f;
    m_kd = 1.0f;
    m_maxTorque = 500.0f;
    m_speed = 1.0f;

    m_body0 = nullptr;
    m_body1 = nullptr;
}

atg_scs::ConstantSpeedMotor::~ConstantSpeedMotor() {
    /* void */
}

void atg_scs::ConstantSpeedMotor::apply(SystemState *state) {
    real_t v1;
    real_t a1;

    if (m_body0->index == -1) {
        v1 = a1 = 0;
    }
    else {
        v1 = state->v_theta[m_body0->index];
        a1 = state->a_theta[m_body0->index];
    }

    const real_t rel_v =
        state->v_theta[m_body1->index] - v1;
    const real_t rel_a =
        state->a_theta[m_body1->index] - a1;
    const real_t delta = m_speed - rel_v;

    const real_t torque = delta * m_ks;
    const real_t dampingTorque = -rel_a * m_kd;
    const real_t totalTorque = torque + dampingTorque;
    const real_t limitedTorque =
        std::fmin(m_maxTorque, std::fmax(-m_maxTorque, totalTorque));

    if (m_body0->index != -1) state->t[m_body0->index] -= limitedTorque;
    state->t[m_body1->index] += limitedTorque;
}
