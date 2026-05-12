#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H

#include "force_generator.h"

#include "rigid_body.h"
#include "types.h"

namespace atg_scs {
    class GravityForceGenerator : public ForceGenerator {
        public:
            GravityForceGenerator();
            virtual ~GravityForceGenerator();

            virtual void apply(SystemState *state);

            real_t m_g;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GRAVITY_FORCE_GENERATOR_H */
