#ifndef ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_CONSTANT_SPEED_MOTOR_H

#include "force_generator.h"

#include "rigid_body.h"
#include "types.h"

namespace atg_scs {
    class ConstantSpeedMotor : public ForceGenerator {
        public:
            ConstantSpeedMotor();
            virtual ~ConstantSpeedMotor();

            virtual void apply(SystemState *state);

            real_t m_ks;
            real_t m_kd;
            real_t m_maxTorque;
            real_t m_speed;

            RigidBody *m_body0;
            RigidBody *m_body1;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
