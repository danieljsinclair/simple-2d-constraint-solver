#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FIXED_POSITION_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FIXED_POSITION_CONSTRAINT_H

#include "constraint.h"
#include "types.h"

namespace atg_scs {
    class FixedPositionConstraint : public Constraint {
        public:
            FixedPositionConstraint();
            virtual ~FixedPositionConstraint();
            
            void setBody(RigidBody *body) { m_bodies[0] = body; }

            void setWorldPosition(real_t x, real_t y);
            void setLocalPosition(real_t x, real_t y);

            virtual void calculate(Output *output, SystemState *system);

            real_t m_local_x;
            real_t m_local_y;
            real_t m_world_x;
            real_t m_world_y;
            real_t m_ks;
            real_t m_kd;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FIXED_POSITION_CONSTRAINT_H */
