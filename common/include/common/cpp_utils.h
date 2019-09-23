#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

namespace cpp_utils {

//
// vector operations
//
template <typename T>
std::vector<T> linspace(T a, T b, size_t n) {
    Eigen::Matrix<T,Eigen::Dynamic,1> lin_spaced_data =
            Eigen::Matrix<T,Eigen::Dynamic,1>::LinSpaced((int)n,a,b);

    std::vector<T> array(lin_spaced_data.data(), lin_spaced_data.data() + lin_spaced_data.size());

    return array;
}

template <typename T1, typename T2>
std::vector<T2> tcast_vector(std::vector<T1> const & vec) {
    std::vector<T2> vec2(vec.begin(),vec.end());
    return vec2;
}

template <typename T>
std::vector<T> add_scalar_to_vector_copy(std::vector<T> const & vec, T scalar) {
    std::vector<T> vec_copy = vec;
    std::for_each(vec_copy.begin(),vec_copy.end(),
                  [scalar] (T & el) {el+=scalar;});
    return vec_copy;
}

template <typename T>
void add_scalar_to_vector(std::vector<T> & vec, T scalar) {
    std::for_each(vec.begin(),vec.end(),
                  [scalar] (T & el) {el+=scalar;});
}


template <typename T>
std::vector<T> multiply_scalar_vector_copy(std::vector<T> const & vec, T scalar) {
    std::vector<T> vec_copy = vec;
    std::for_each(vec_copy.begin(),vec_copy.end(),
                  [scalar] (T & el) {el*=scalar;});
    return vec_copy;
}

template <typename T>
void multiply_scalar_vector(std::vector<T> & vec, T scalar) {
    std::for_each(vec.begin(),vec.end(),
                  [scalar] (T & el) {el*=scalar;});
}



//
// coordinate transformations
//

// todo

}
