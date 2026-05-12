#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_H

#include "types.h"

namespace atg_scs {
    struct RigidBody {
        public:
            RigidBody();
            ~RigidBody();

            void localToWorld(real_t x, real_t y, real_t *w_x, real_t *w_y);
            void worldToLocal(real_t x, real_t y, real_t *l_x, real_t *l_y);

            real_t p_x;
            real_t p_y;

            real_t v_x;
            real_t v_y;

            real_t theta;
            real_t v_theta;

            real_t m;
            real_t I;

            int index;

            void reset();
            real_t energy() const;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_H */
