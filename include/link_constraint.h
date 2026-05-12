#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINK_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINK_CONSTRAINT_H

#include "constraint.h"
#include "types.h"

namespace atg_scs {
    class LinkConstraint : public Constraint {
        public:
            LinkConstraint();
            virtual ~LinkConstraint();
            
            void setBody1(RigidBody *body) { m_bodies[0] = body; }
            void setBody2(RigidBody *body) { m_bodies[1] = body; }

            void setLocalPosition1(real_t x, real_t y);
            void setLocalPosition2(real_t x, real_t y);

            virtual void calculate(Output *output, SystemState *system);

            real_t m_maxForce;

            real_t m_local_x_1;
            real_t m_local_y_1;
            real_t m_local_x_2;
            real_t m_local_y_2;
            real_t m_ks;
            real_t m_kd;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINK_CONSTRAINT_H */
