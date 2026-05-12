#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTANT_ROTATION_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTANT_ROTATION_CONSTRAINT_H

#include "constraint.h"
#include "types.h"

namespace atg_scs {
    class ConstantRotationConstraint : public Constraint {
        public:
            ConstantRotationConstraint();
            virtual ~ConstantRotationConstraint();

            void setBody(RigidBody *body) { m_bodies[0] = body; }

            virtual void calculate(Output *output, SystemState *state);

            real_t m_rotationSpeed;
            real_t m_ks;
            real_t m_kd;

            real_t m_minTorque;
            real_t m_maxTorque;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTANT_ROTATION_CONSTRAINT_H */
