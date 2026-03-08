#ifndef GAINS_H
#define GAINS_H

#include <BasicLinearAlgebra.h>

static BLA::Matrix<3,12> gainZero = BLA::Zeros<3,12>();
static BLA::Matrix<3,12> gainEdgeMax = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, -3.189719240068948, -0.423817573914098, -0.001293942626763, -0.000992253824621, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static BLA::Matrix<3,12> gainCornerMax = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   

#endif