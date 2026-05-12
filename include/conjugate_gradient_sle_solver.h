#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONJUGATE_GRADIENT_SLE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONJUGATE_GRADIENT_SLE_SOLVER_H

#include "sle_solver.h"
#include "types.h"

namespace atg_scs {
    class ConjugateGradientSleSolver : public SleSolver {
        public:
            ConjugateGradientSleSolver();
            virtual ~ConjugateGradientSleSolver();

            virtual bool solve(
                    SparseMatrix<3> &J,
                    Matrix &W,
                    Matrix &right,
                    Matrix *result,
                    Matrix *previous);

            void setMaxIterations(int maxIterations) { m_maxIterations = maxIterations; }
            int getMaxIterations() const { return m_maxIterations; }

            void setMaxError(real_t maxError) { m_maxError = maxError; }
            real_t getMaxError() const { return m_maxError; }

            void setMinError(real_t minError) { m_minError = minError; }
            real_t getMinError() const { return m_minError; }

        protected:
            void multiply(SparseMatrix<3> &J, Matrix &W, Matrix &x, Matrix *target);
            bool sufficientlySmall(Matrix &x, Matrix &target) const;

            Matrix
                m_mreg0,
                m_mreg1,
                m_Ap,
                m_x,
                m_r,
                m_p,
                m_A;

            int m_maxIterations;
            real_t m_maxError;
            real_t m_minError;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONJUGATE_GRADIENT_SLE_SOLVER_H */
