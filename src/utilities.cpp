#include "../include/utilities.h"

void atg_scs::freeArray(real_t *&data) {
    delete[] data;
    data = nullptr;
}

void atg_scs::freeArray(int *&data) {
    delete[] data;
    data = nullptr;
}
