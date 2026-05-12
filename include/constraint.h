#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H

#include "system_state.h"
#include "rigid_body.h"
#include "matrix.h"
#include "utilities.h"
#include "types.h"

#include <cfloat>
#include <limits>

namespace atg_scs {
    class Constraint {
        public:
            static constexpr int MaxConstraintCount = 3;
            static constexpr int MaxBodyCount = 2;

            struct Output {
                real_t C[MaxConstraintCount];
                real_t J[MaxConstraintCount][3 * MaxBodyCount];
                real_t J_dot[MaxConstraintCount][3 * MaxBodyCount];
                real_t v_bias[MaxConstraintCount];
                real_t limits[MaxConstraintCount][2];
                real_t ks[MaxConstraintCount];
                real_t kd[MaxConstraintCount];
            };

        public:
            Constraint(int constraintCount, int bodyCount);
            virtual ~Constraint();

            virtual void calculate(Output *output, SystemState *state);
            scs_force_inline int getConstraintCount() const { return m_constraintCount; }

            int m_index;
            int m_bodyCount;
            RigidBody *m_bodies[MaxBodyCount];

            real_t F_x[MaxConstraintCount][MaxBodyCount];
            real_t F_y[MaxConstraintCount][MaxBodyCount];
            real_t F_t[MaxConstraintCount][MaxBodyCount];

        protected:
            inline void noLimits(Output *output) {
                for (int i = 0; i < MaxConstraintCount; ++i) {
                    output->limits[i][0] = -std::numeric_limits<real_t>::max();
                    output->limits[i][1] = std::numeric_limits<real_t>::max();
                }
            }

        protected:
            int m_constraintCount;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H */
