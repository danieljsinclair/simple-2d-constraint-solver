#include "../include/rigid_body.h"

#include <cmath>

using atg_scs::real_t;

atg_scs::RigidBody::RigidBody() {
    index = -1;
    reset();
}

atg_scs::RigidBody::~RigidBody() {
    /* void */
}

atg_scs::real_t atg_scs::RigidBody::energy() const {
    const real_t speed_2 = v_x * v_x + v_y * v_y;
    const real_t E_k = 0.5 * m * speed_2;
    const real_t E_r = 0.5 * I * v_theta * v_theta;

    return E_k + E_r;
}

void atg_scs::RigidBody::localToWorld(
        real_t x,
        real_t y,
        real_t *w_x,
        real_t *w_y)
{
    const real_t cos_theta = std::cos(theta);
    const real_t sin_theta = std::sin(theta);

    *w_x = cos_theta * x - sin_theta * y + p_x;
    *w_y = sin_theta * x + cos_theta * y + p_y;
}

void atg_scs::RigidBody::worldToLocal(
        real_t x,
        real_t y,
        real_t *l_x,
        real_t *l_y)
{
    const real_t cos_theta = std::cos(theta);
    const real_t sin_theta = std::sin(theta);

    *l_x = cos_theta * (x - p_x) + sin_theta * (y - p_y);
    *l_y = -sin_theta * (x - p_x) + cos_theta * (y - p_y);
}

void atg_scs::RigidBody::reset() {
    p_x = p_y = 0.0;
    v_x = v_y = 0.0;

    theta = 0.0;
    v_theta = 0.0;

    m = 0.0;
    I = 0.0;
}
