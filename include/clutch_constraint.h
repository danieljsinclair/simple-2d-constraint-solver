#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H

#include "constraint.h"
#include "types.h"

namespace atg_scs {
    class ClutchConstraint : public Constraint {
        public:
            ClutchConstraint();
            virtual ~ClutchConstraint();
            
            void setBody1(RigidBody *body) { m_bodies[0] = body; }
            void setBody2(RigidBody *body) { m_bodies[1] = body; }

            virtual void calculate(Output *output, SystemState *system);

            real_t m_maxTorque;
            real_t m_minTorque;

            real_t m_ks;
            real_t m_kd;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CLUTCH_CONSTRAINT_H */
