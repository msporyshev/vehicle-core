#pragma once

#include <array>
#include <vector>

template<size_t size>
int LUP_decomposition(std::array<std::array<double, size>, size> &LU,
                      std::array<int, size> &P)
{
    for (int i = 0; i < size; ++i) {
        P[i] = i;
    }        

    for (int i = 0; i < size; ++i) {
        double pivot = 1e-6;
        int pivot_it = -1;
        for (int j = i; j < size; ++j) {
            if (fabs(LU[j][i]) > pivot) {
                pivot = LU[j][i];
                pivot_it = j;
            }
        }
        if (pivot_it == -1) {
            return 1; // Singular Matrix!
        }
            
        std::swap(P[i], P[pivot_it]);
        for (int j = 0; j < size; ++j) {
            std::swap(LU[i][j], LU[pivot_it][j]);
        }            

        for (int j = i + 1; j < size; ++j) {
            LU[j][i] /= LU[i][i];
            for (int k = i + 1; k < size; ++k) {
                LU[j][k] -= LU[j][i] * LU[i][k];
            }
        }
    }

    return 0;
}

template<size_t size>
void LUP_solve(std::array<std::array<double, size>, size> &LU,
               std::array<int, size> &P,
               std::array<double, size> &b,
               std::array<double, size> &x)
{
    std::vector<double> y(size);

    // Forward substitution using L
    for (int i = 0; i < size; ++i) {
        double calculated = 0;
        for (int j = 0; j < i; ++j) {
            calculated += LU[i][j] * y[j];
        }            
        y[i] = b[P[i]] - calculated;
    }

    // Back substitution using U
    for (int i = size - 1; i >= 0; --i) {
        double calculated = 0;
        for (int j = i + 1; j < size; ++j) {
            calculated += LU[i][j] * x[j];
        }            
        x[i] = (y[i] - calculated) / LU[i][i];
    }
}

template<size_t size>
int invert_matrix(std::array<std::array<double, size>, size> &origin,
                  std::array<std::array<double, size>, size> &invert)
{
    const int n = size;

    // LU - матрица, содержащая значащие элементы нижне-треугольной матрицы L и
    // значащие элементы верхне-треугольной матрицы U.
    std::array<std::array<double, n>, n> LU = origin;

    // P - массив, описывающий матрицу перестановок
    // P[i] = j означает, что i-я строка матрицы содержит 1 в столбце j. 
    std::array<int, n> P;
    
    if (LUP_decomposition(LU, P)) {
        return 1;
    }        

    for (int i = 0; i < n; ++i) {
        std::array<double, n> x;
        std::array<double, n> b {0};
        b[i] = 1;
        LUP_solve(LU, P, b, x); // Решает систему A * x = b

        for (int j = 0; j < size; ++j) {
            invert[j][i] = x[j];
        }            
    }

    return 0;
}