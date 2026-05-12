#include "../include/spring.h"

#include <cmath>

using atg_scs::real_t;

atg_scs::Spring::Spring() {
    m_restLength = 1.0;
    m_ks = 0;
    m_kd = 0;

    m_p1_x = m_p1_y = 0;
    m_p2_x = m_p2_y = 0;

    m_body1 = m_body2 = nullptr;
}

atg_scs::Spring::~Spring() {
    /* void */
}

void atg_scs::Spring::apply(SystemState *state) {
    if (m_body1 == nullptr || m_body2 == nullptr) return;

    real_t x1, y1;
    real_t x2, y2;

    real_t v_x1 = 0, v_y1 = 0;
    real_t v_x2 = 0, v_y2 = 0;

    if (m_body1->index != -1) {
        state->localToWorld(m_p1_x, m_p1_y, &x1, &y1, m_body1->index);
        state->velocityAtPoint(m_p1_x, m_p1_y, &v_x1, &v_y1, m_body1->index);
    }
    else {
        m_body1->localToWorld(m_p1_x, m_p1_y, &x1, &y1);
    }

    if (m_body2->index != -1) {
        state->localToWorld(m_p2_x, m_p2_y, &x2, &y2, m_body2->index);
        state->velocityAtPoint(m_p2_x, m_p2_y, &v_x2, &v_y2, m_body2->index);
    }
    else {
        m_body2->localToWorld(m_p2_x, m_p2_y, &x2, &y2);
    }

    real_t dx = x2 - x1;
    real_t dy = y2 - y1;

    const real_t l = std::sqrt(dx * dx + dy * dy);

    if (std::abs(l) >= 1E-2) {
        dx /= l;
        dy /= l;
    }
    else {
        dx = 0.0;
        dy = 0.0;
    }

    const real_t rel_v_x = (v_x2 - v_x1);
    const real_t rel_v_y = (v_y2 - v_y1);

    const real_t x = l - m_restLength;

    state->applyForce(
        m_p1_x,
        m_p1_y,
        dx * x * m_ks + rel_v_x * m_kd,
        dy * x * m_ks + rel_v_y * m_kd,
        m_body1->index
    );

    state->applyForce(
        m_p2_x,
        m_p2_y,
        -dx * x * m_ks - rel_v_x * m_kd,
        -dy * x * m_ks - rel_v_y * m_kd,
        m_body2->index
    );
}

void atg_scs::Spring::getEnds(real_t *x_1, real_t *y_1, real_t *x_2, real_t *y_2) {
    if (m_body1 == nullptr || m_body2 == nullptr) return;

    m_body1->localToWorld(m_p1_x, m_p1_y, x_1, y_1);
    m_body2->localToWorld(m_p2_x, m_p2_y, x_2, y_2);
}

atg_scs::real_t atg_scs::Spring::energy() const {
    if (m_body1 == nullptr || m_body2 == nullptr) return 0;

    real_t x1, y1;
    real_t x2, y2;

    m_body1->localToWorld(m_p1_x, m_p1_y, &x1, &y1);
    m_body2->localToWorld(m_p2_x, m_p2_y, &x2, &y2);

    const real_t dx = x2 - x1;
    const real_t dy = y2 - y1;

    const real_t l = std::sqrt(dx * dx + dy * dy);

    return 0.5 * m_ks * (l - m_restLength) * (l - m_restLength);
}
