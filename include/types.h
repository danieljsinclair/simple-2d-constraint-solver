#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_TYPES_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_TYPES_H

namespace atg_scs {
    typedef float real_t;
}

// Also provide real_t at global scope when included from engine-sim context
using real_t = atg_scs::real_t;

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_TYPES_H */
