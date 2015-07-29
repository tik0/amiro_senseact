//======================================================================
/*! \file io_prototypes.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 24, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_IO_PROTOTYPES_HPP_SEEN
#define IBEOSDK_IO_PROTOTYPES_HPP_SEEN

//======================================================================

#include <iostream>

//======================================================================

namespace ibeosdk {

//======================================================================

template<typename T> void writeLE(std::ostream& os, const T& value);
template<typename T> void writeBE(std::ostream& os, const T& value);

template<typename T> void readLE(std::istream& is, T& value);
template<typename T> void readBE(std::istream& is, T& value);

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_IO_PROTOTYPES_HPP_SEEN


//======================================================================
