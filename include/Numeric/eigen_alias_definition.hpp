/*
 * eigen_alias_definition.hpp
 *
 *  Created on: Jul 7, 2018
 *      Author: zer0
 */

#ifndef INCLUDE_NUMERIC_EIGEN_ALIAS_DEFINITION_HPP_
#define INCLUDE_NUMERIC_EIGEN_ALIAS_DEFINITION_HPP_

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/StdVector>

#include <initializer_list>
#include <memory>
#include <vector>

// Extend EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION with initializer list support.
#define EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(...)       \
namespace std {                                                            \
  template <>                                                              \
  class vector<__VA_ARGS__, std::allocator<__VA_ARGS__>>                   \
      : public vector<__VA_ARGS__, Eigen::aligned_allocator<__VA_ARGS__>> { \
    typedef vector<__VA_ARGS__, Eigen::aligned_allocator<__VA_ARGS__>>      \
        vector_base;                                                       \
                                                                           \
   public:                                                                 \
    typedef __VA_ARGS__ value_type;                                        \
    typedef vector_base::allocator_type allocator_type;                    \
    typedef vector_base::size_type size_type;                              \
    typedef vector_base::iterator iterator;                                \
    explicit vector(const allocator_type& a = allocator_type())            \
        : vector_base(a) {}                                                \
    template <typename InputIterator>                                      \
    explicit vector(InputIterator first, InputIterator last,                        \
           const allocator_type& a = allocator_type())                     \
        : vector_base(first, last, a) {}                                   \
    vector(const vector& c) = default;                                     \
    explicit vector(size_type num, const value_type& val = value_type())   \
        : vector_base(num, val) {}                                         \
    explicit vector(iterator start, iterator end) : vector_base(start, end) {}      \
    vector& operator=(const vector& x) = default;                          \
    /* Add initializer list constructor support*/                          \
    vector(std::initializer_list<__VA_ARGS__> list)                             \
        : vector_base(list.begin(), list.end()) {}                         \
  };                                                                       \
}  // namespace std

namespace ImageProcess
{
  using Eigen::Map;

  /// Trait used for double type
  using EigenDoubleTraits = Eigen::NumTraits<double>;

  /// 3d vector using double internal format
  using Vec3 = Eigen::Vector3d;

  /// 2d vector using int internal format
  using Vec2i = Eigen::Vector2i;

  /// 2d vector using float internal format
  using Vec2f = Eigen::Vector2f;

  /// 3d vector using float internal format
  using Vec3f =Eigen::Vector3f;

  /// 9d vector using double internal format
  using Vec9 = Eigen::Matrix<double, 9, 1>;

  /// Quaternion type
  using Quaternion = Eigen::Quaternion<double>;

  /// 3x3 matrix using double internal format
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  /// 3x4 matrix using double internal format
  using Mat34 = Eigen::Matrix<double, 3, 4>;

  /// 2d vector using double internal format
  using Vec2 = Eigen::Vector2d;

  /// 4d vector using double internal format
  using Vec4 = Eigen::Vector4d;

  /// 6d vector using double internal format
  using Vec6 = Eigen::Matrix<double, 6, 1>;

  /// 4x4 matrix using double internal format
  using Mat4 = Eigen::Matrix<double, 4, 4>;

  /// generic matrix using unsigned int internal format
  using Matu = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;

  /// 3x3 matrix using double internal format with RowMajor storage
  using RMat3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

  //-- General purpose Matrix and Vector
  /// Unconstrained matrix using double internal format
  using Mat = Eigen::MatrixXd;

  /// Unconstrained vector using double internal format
  using Vec = Eigen::VectorXd;

  /// Unconstrained vector using unsigned int internal format
  using Vecu = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;

  /// Unconstrained matrix using float internal format
  using Matf = Eigen::MatrixXf;

  /// Unconstrained vector using float internal format
  using Vecf = Eigen::VectorXf;

  /// 2xN matrix using double internal format
  using Mat2X = Eigen::Matrix<double, 2, Eigen::Dynamic>;

  /// 3xN matrix using double internal format
  using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

  /// 4xN matrix using double internal format
  using Mat4X = Eigen::Matrix<double, 4, Eigen::Dynamic>;

  /// 9xN matrix using double internal format
  using MatX9 = Eigen::Matrix<double, Eigen::Dynamic, 9>;

  //-- Sparse Matrix (Column major, and row major)
  /// Sparse unconstrained matrix using double internal format
  using sMat = Eigen::SparseMatrix<double>;

  /// Sparse unconstrained matrix using double internal format and Row Major storage
  using sRMat = Eigen::SparseMatrix<double, Eigen::RowMajor>;

} // namespace ImageProcess

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec2)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec4)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec6)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec9)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec2i)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Vec3f)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Quaternion)

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Mat3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::RMat3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_INITIALIZER_LIST(ImageProcess::Mat4)



#endif /* INCLUDE_NUMERIC_EIGEN_ALIAS_DEFINITION_HPP_ */
