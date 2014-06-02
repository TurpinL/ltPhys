#ifndef LTPHYS_SCALAR_HPP
#define LTPHYS_SCALAR_HPP

#include <limits>

// Make sure math knows what Scalar is!
#define scalar_pow powf

namespace lt
{

// You can change this to a double, that's nice. Right?
typedef float Scalar;
const Scalar SCALAR_MAX = std::numeric_limits<float>::max();

} // namespace lt

#endif // LTPHYS_SCALAR_HPP