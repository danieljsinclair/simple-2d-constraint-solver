#include "../include/rotation_friction_constraint.h"

#include <limits>
#include <cfloat>

atg_scs::RotationFrictionConstraint::RotationFrictionConstraint() : Constraint(1, 1) {
    m_ks = 10.0;
    m_kd = 1.0;

    m_maxTorque = std::numeric_limits<real_t>::max();
    m_minTorque = -std::numeric_limits<real_t>::max();
}

atg_scs::RotationFrictionConstraint::~RotationFrictionConstraint() {
    /* void */
}

void atg_scs::RotationFrictionConstraint::calculate(
        Output *output,
        SystemState *state)
{
    [[maybe_unused]] const int body = m_bodies[0]->index;

    output->C[0] = 0;

    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = 1.0;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->kd[0] = m_kd;
    output->ks[0] = m_ks;

    output->v_bias[0] = 0;

    output->limits[0][0] = m_minTorque;
    output->limits[0][1] = m_maxTorque;
}
