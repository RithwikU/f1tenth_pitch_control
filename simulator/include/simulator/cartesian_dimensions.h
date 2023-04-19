#ifndef TOWR_VARIABLES_CARTESIAN_DIMENSIONS_H_
#define TOWR_VARIABLES_CARTESIAN_DIMENSIONS_H_

#include <cassert>

namespace f1tenth {

// 2-dimensional
static constexpr int k2D = 2;
enum Dim2D { X_=0, Y_};

// 3-dimensional
static constexpr int k3D = 3;
enum Dim3D { X=0, Y, Z };
static Dim2D To2D(Dim3D dim)
{
  assert(dim != Z);
  return static_cast<Dim2D>(dim);
};

// 6-dimensional
// 'A' stands for angular, 'L' for linear.
static constexpr int k6D = 6; // X,Y,Z, roll, pitch, yaw
enum Dim6D { AX=0, AY, AZ, LX, LY, LZ };
static const Dim6D AllDim6D[] = {AX, AY, AZ, LX, LY, LZ};

} // namespace towr

#endif /* TOWR_VARIABLES_CARTESIAN_DIMENSIONS_H_ */