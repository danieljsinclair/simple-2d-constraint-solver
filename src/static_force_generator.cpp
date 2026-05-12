#include "../include/static_force_generator.h"

#include <cmath>

atg_scs::StaticForceGenerator::StaticForceGenerator() {
    m_f_x = m_f_y = 0.0;
    m_p_x = m_p_y = 0.0;

    m_body = nullptr;
}

atg_scs::StaticForceGenerator::~StaticForceGenerator() {
    /* void */
}

void atg_scs::StaticForceGenerator::apply(SystemState *state) {
    state->applyForce(
        m_p_x,
        m_p_y,
        m_f_x,
        m_f_y,
        m_body->index
    );
}

void atg_scs::StaticForceGenerator::setForce(real_t f_x, real_t f_y) {
    m_f_x = f_x;
    m_f_y = f_y;
}

void atg_scs::StaticForceGenerator::setPosition(real_t p_x, real_t p_y) {
    m_p_x = p_x;
    m_p_y = p_y;
}
