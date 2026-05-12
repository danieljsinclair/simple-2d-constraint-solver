#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SYSTEM_STATE_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SYSTEM_STATE_H

#include "types.h"

namespace atg_scs {
    class SystemState {
        public:
            SystemState();
            ~SystemState();

            void copy(const SystemState *state);
            void resize(int bodyCount, int constraintCount);
            void destroy();

            void localToWorld(real_t x, real_t y, real_t *x_t, real_t *y_t, int body);
            void velocityAtPoint(real_t x, real_t y, real_t *v_x, real_t *v_y, int body);
            void applyForce(real_t x_l, real_t y_l, real_t f_x, real_t f_y, int body);

            int *indexMap;

            real_t *a_theta;
            real_t *v_theta;
            real_t *theta;

            real_t *a_x;
            real_t *a_y;
            real_t *v_x;
            real_t *v_y;
            real_t *p_x;
            real_t *p_y;

            real_t *f_x;
            real_t *f_y;
            real_t *t;

            real_t *r_x;
            real_t *r_y;
            real_t *r_t;

            real_t *m;

            int n;
            int n_c;
            real_t dt;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SYSTEM_STATE_H */
