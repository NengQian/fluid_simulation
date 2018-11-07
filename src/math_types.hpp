#pragma once

#include <Eigen/Dense>
#include <vector>

namespace Simulator
{
    // You should probably use double while building your simulator. At the end, you can try switching to
    // float for increased performance at the cost of precision (which may cause stability issues. Or not).
    typedef double Real;

    // There are some issues with Eigen and aligned types that we'd rather
    // not have to deal with, so we specify that we do not want any alignment.
    // See https://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__Alignement.html
    // for more information.
    typedef Eigen::Quaternion<Real, Eigen::DontAlign>      RealQuaternion;

    typedef Eigen::Matrix<Real, 2, 1, Eigen::DontAlign>    RealVector2;
    typedef Eigen::Matrix<Real, 3, 1, Eigen::DontAlign>    RealVector3;
    typedef Eigen::Matrix<Real, 4, 1, Eigen::DontAlign>    RealVector4;
    typedef Eigen::Matrix<Real, Eigen::Dynamic, 1, Eigen::DontAlign>    RealVectorX;


    typedef Eigen::Matrix<Real, 2, 2, Eigen::DontAlign>    RealMatrix2;
    typedef Eigen::Matrix<Real, 3, 3, Eigen::DontAlign>    RealMatrix3;
    typedef Eigen::Matrix<Real, 4, 4, Eigen::DontAlign>    RealMatrix4;

    // Add more types as you see fit
}
