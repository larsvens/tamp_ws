#include <vector>
#include <algorithm>
#include <tf/LinearMath/Quaternion.h>
#include <stdexcept>

namespace cpp_utils {

template <typename T>
inline  std::vector<size_t> interp_lb(std::vector<T> const & x_intrp,
                                      std::vector<T> const & x)
{
    std::vector<size_t> result(x_intrp.size());
    for(size_t i=0; i<x_intrp.size(); ++i) {
      T const & x_i = x_intrp.at(i);

      auto it = std::upper_bound(x.begin(),x.end(),x_i);
      if(it == x.end()) {
        result.at(i) = it-x.begin()-1;
      } else if(it == x.begin()) {
        result.at(i) = it-x.begin();
      } else {
        result.at(i) = it-x.begin()-1;
      }
    }
    return result;
}

template <typename T>
inline  std::vector<size_t> interp_ub(std::vector<T> const & x_intrp,
                                      std::vector<T> const & x)
{
    std::vector<size_t> result(x_intrp.size());
    for(size_t i=0; i<x_intrp.size(); ++i) {
      T const & x_i = x_intrp.at(i);

      auto it = std::upper_bound(x.begin(),x.end(),x_i);
      if(it == x.end()) {
        result.at(i) = it-x.begin()-1;
      } else if(it == x.begin()) {
        result.at(i) = it-x.begin();
      } else {
        result.at(i) = it-x.begin();
      }
    }
    return result;
}


template <typename T>
/**
 * @brief interp        Linear iterpolation (with and without extrapolation)
 * @param x_intrp       Point where we want to estimate y(x_intrp)
 * @param x
 * @param y             y(x)
 * @param extrapolate
 * @return              esimated y(x_intrp)
 */
inline std::vector<T> interp(std::vector<T> const & x_intrp,
                             std::vector<T> const & x,
                             std::vector<T> const & y,
                             bool extrapolate) {

  if(x.size() != y.size()) {
    throw std::logic_error("x,y must be of the same size");
  }
  if(x.size() < 2) {
    throw std::logic_error("must have at least two values to interpolate between");
  }

  ///@todo Check for duplicate x values -- we might want to warn about this...
//  for(size_t i=0; i<x.size()-1; ++i) {
//    if(x.at(i)==x.at(i+1)) {
//      std::stringstream ss;
//      ss << __PRETTY_FUNCTION__ << ": Duplicate x values detected x[" << i << "] = " << x.at(i) << " = x["<<i+1<<"]";
//      ss << std::endl << "x = ";
//      for(T x_val : x) {
//        ss << x_val << ", ";
//      }
//      throw std::logic_error(ss.str());
//    }
//  }

  std::vector<T> result(x_intrp.size());

  for(size_t i=0; i<x_intrp.size(); ++i) {
    T const & x_i = x_intrp.at(i);
    //first element that is not less than x_ii
    //auto it = std::lower_bound(x.begin(),x.end(),x_ii);
    auto it = std::upper_bound(x.begin(),x.end(),x_i);

    if(it == x.end()) { //index of x.end is beoynd last element!
      //x_ii greater than x[-1]
      if(extrapolate) {
        size_t l = it-x.begin()-2;
        size_t u = it-x.begin()-1;
        //handle duplicate x values in a row
        while(x.at(l)==x.at(u) && l>1) {
          l--;
        }
        T delta_x = x.at(u)-x.at(l);
        T delta_y = y.at(u)-y.at(l);
        T t       = x_i-x.at(l);
        result.at(i) = y.at(l) + t*(delta_y/delta_x);
      } else {

        size_t u = it-x.begin()-1;
        result.at(i) = y.at(u);
      }
    } else if(it == x.begin()) {
      //x[0] greater than x_ii
      if(extrapolate) {
        size_t l = it-x.begin();
        size_t u = it-x.begin()+1;
        //handle duplicate x values in a row
        while(x.at(l)==x.at(u) && u<x.size()-1) {
          u++;
        }
        T delta_x = x.at(u)-x.at(l);
        T delta_y = y.at(u)-y.at(l);
        T t       = x_i-x.at(l);
        result.at(i) = y.at(l) + t*(delta_y/delta_x);
      } else {
        size_t l = it-x.begin();
        result.at(i) = y.at(l);
      }
    } else {
      size_t l = it-x.begin()-1;
      size_t u = it-x.begin();

      //handle duplicate x values in a row
      while(x.at(l)==x.at(u) && l>1) {
        l--;
      }

      T delta_x = x.at(u)-x.at(l);
      T delta_y = y.at(u)-y.at(l);
      T t       = x_i-x.at(l);
      result.at(i) = y.at(l) + t*(delta_y/delta_x);
    }
  }
  return result;
}

template <typename T>
inline std::vector<tf::Quaternion> interp_angle(std::vector<T> const & x_intrp,
                                                std::vector<T> const & x,
                                                std::vector<tf::Quaternion> const & quats,
                                                bool extrapolate) {

  assert(x.size() == quats.size());
  assert(x.size() > 1);
  std::vector<tf::Quaternion> result(x_intrp.size());

  for(size_t i=0; i<x_intrp.size(); ++i) {
    T const & x_i = x_intrp.at(i);
    //first element that is not less than x_ii
    //auto it = std::lower_bound(x.begin(),x.end(),x_ii);
    auto it = std::upper_bound(x.begin(),x.end(),x_i);

    if(it == x.end()) { //index of x.end is beoynd last element!
      //x_ii greater than x[-1]
      if(extrapolate) {
        size_t l = it-x.begin()-2;
        size_t u = it-x.begin()-1;
        //handle duplicate x values in a row
        while(x.at(l)==x.at(u) && l>0) {
          l--;
        }

        T t       = x_i-x.at(l);

        //t is larger than 1... hope this works..
        result.at(i) = quats.at(l).slerp(quats.at(u),t);

      } else {

        size_t u = it-x.begin()-1;
        result.at(i) = quats.at(u);
      }
    } else if(it == x.begin()) {
      //x[0] greater than x_ii
      if(extrapolate) {
        size_t l = it-x.begin();
        size_t u = it-x.begin()+1;
        //handle duplicate x values in a row
        while(x.at(l)==x.at(u) && u<x.size()) {
          u++;
        }

        T t       = x_i-x.at(l);
        //t is less than 0... hope this works..
        result.at(i) = quats.at(l).slerp(quats.at(u),t);
      } else {
        size_t l = it-x.begin();
        result.at(i) = quats.at(l);
      }
    } else {
      size_t l = it-x.begin()-1;
      size_t u = it-x.begin();

      //handle duplicate x values in a row
      while(x.at(l)==x.at(u) && l>0) {
        l--;
      }
      T t       = x_i-x.at(l);
      result.at(i) = quats.at(l).slerp(quats.at(u),t);
    }
  }
  return result;
}

}
