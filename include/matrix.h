#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_MATRIX_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_MATRIX_H

#include "utilities.h"
#include <assert.h>

namespace atg_scs {
    class Matrix {
        public:
            Matrix();
            Matrix(int width, int height, real_t value = 0.0);
            ~Matrix();

            void initialize(int width, int height, real_t value);
            void initialize(int width, int height);
            void resize(int width, int height);
            void destroy();

            void set(const real_t *data);

            scs_force_inline void set(int column, int row, real_t value) {
                assert(column >= 0 && column < m_width);
                assert(row >= 0 && row < m_height);

                m_matrix[row][column] = value;
            }

            scs_force_inline void add(int column, int row, real_t value) {
                assert(column >= 0 && column < m_width);
                assert(row >= 0 && row < m_height);

                m_matrix[row][column] += value;
            }

            scs_force_inline real_t get(int column, int row) {
                assert(column >= 0 && column < m_width);
                assert(row >= 0 && row < m_height);

                return m_matrix[row][column];
            }

            void set(Matrix *reference);

            void multiply(Matrix &b, Matrix *target);
            void componentMultiply(Matrix &b, Matrix *target);
            void transposeMultiply(Matrix &b, Matrix *target);
            void leftScale(Matrix &scale, Matrix *target);
            void rightScale(Matrix &scale, Matrix *target);
            void scale(real_t s, Matrix *target);
            void subtract(Matrix &b, Matrix *target);
            void add(Matrix &b, Matrix *target);
            void negate(Matrix *target);
            bool equals(Matrix &b, real_t err = 1e-6);
            real_t vectorMagnitudeSquared() const;
            real_t dot(Matrix &b) const;

            void madd(Matrix &b, real_t s);
            void pmadd(Matrix &b, real_t s);

            void transpose(Matrix *target);
            int getWidth() const { return m_width; }
            int getHeight() const { return m_height; }

            scs_force_inline void fastRowSwap(int a, int b) {
                real_t *temp = m_matrix[a];
                m_matrix[a] = m_matrix[b];
                m_matrix[b] = temp;
            }

        protected:
            real_t **m_matrix;
            real_t *m_data;
            int m_width;
            int m_height;
            int m_capacityWidth;
            int m_capacityHeight;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_MATRIX_H */
