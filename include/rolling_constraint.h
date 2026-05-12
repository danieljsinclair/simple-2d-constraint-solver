#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ROLLING_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ROLLING_CONSTRAINT_H

#include "constraint.h"
#include "types.h"

namespace atg_scs {
    class RollingConstraint : public Constraint {
        public:
            RollingConstraint();
            virtual ~RollingConstraint();

            void setBaseBody(RigidBody *body) { m_bodies[0] = body; }
            void setRollingBody(RigidBody *body) { m_bodies[1] = body; }

            virtual void calculate(Output *output, SystemState *system);

            real_t m_local_x;
            real_t m_local_y;
            real_t m_dx;
            real_t m_dy;
            real_t m_radius;

            real_t m_ks;
            real_t m_kd;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_ROLLING_CONSTRAINT_H */
