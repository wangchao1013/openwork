#pragma once
#include <stdio.h>
#include <stddef.h>             // for size_t
#include <math.h>               // for fabs  
#include <vector>


//! comfortable array of doubles
typedef std::vector<double> float_vect;
//! comfortable array of ints;
typedef std::vector<int>    int_vect;

// savitzky golay smoothing.
double sg_smooth(const float_vect& v, const int w, const int deg);
//! numerical derivative based on savitzky golay smoothing.
float_vect sg_derivative(const float_vect& v, const int w,
    const int deg, const double h = 1.0);