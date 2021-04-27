/// @brief fundamental datatypes that use the standard lib.
/// @file : types.h
/// @author : s.aparajith@live.com
/// @date : 25/4/2021
/// @details : it is better to have proper types so that 
/// changes in the architecture (x32 or x64) wont cause an issue with types when dealt with embedded systems 
/// @copyright : none reserved. No liabilities. this source code is free to be distributed and copied. use under own resposibility. MIT-License.

#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
namespace type{
using uint64 = std::uint64_t;
using uint32 = std::uint32_t;
using uint16 = std::uint16_t;
using uint8 = std::uint8_t;

using int64 = std::int64_t;
using int32 = std::int32_t;
using int16 = std::int16_t;
using int8 = std::int8_t;

using float32 = float;
using float64 = double;

}
#endif