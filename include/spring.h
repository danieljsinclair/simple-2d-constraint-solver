#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H

#include "force_generator.h"

#include "rigid_body.h"
#include "types.h"

namespace atg_scs {
    class Spring : public ForceGenerator {
        public:
            Spring();
            virtual ~Spring();

            virtual void apply(SystemState *state);
            
            void getEnds(real_t *x_1, real_t *y_1, real_t *x_2, real_t *y_2);
            real_t energy() const;

            real_t m_restLength;
            real_t m_ks;
            real_t m_kd;

            real_t m_p1_x;
            real_t m_p1_y;

            real_t m_p2_x;
            real_t m_p2_y;

            RigidBody *m_body1;
            RigidBody *m_body2;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
