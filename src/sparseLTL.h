#pragma once

typedef float T;

void LTLInPlace(T *matrix, int *parentIndex, int n);
void backSubstitutionInPlace(T *matrix, T *b, int *parentIndex, int n);
void forwardSubstitutionInPlace(T *matrix, T *b, int *parentIndex, int n);
