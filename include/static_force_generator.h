#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H

#include "force_generator.h"

#include "rigid_body.h"
#include "types.h"

namespace atg_scs {
    class StaticForceGenerator : public ForceGenerator {
        public:
            StaticForceGenerator();
            virtual ~StaticForceGenerator();

            virtual void apply(SystemState *state);

            void setForce(real_t f_x, real_t f_y);
            void setPosition(real_t p_x, real_t p_y);

            real_t m_f_x;
            real_t m_f_y;

            real_t m_p_x;
            real_t m_p_y;

            RigidBody *m_body;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_STATIC_FORCE_GENERATOR_H */
