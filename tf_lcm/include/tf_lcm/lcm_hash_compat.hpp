#ifndef TF_LCM_HASH_COMPAT_HPP
#define TF_LCM_HASH_COMPAT_HPP

/**
 * @file lcm_hash_compat.hpp
 * @brief Cross-platform compatibility for LCM hash functions
 * 
 * This header provides functions and macros to handle LCM hash function pointer 
 * conversions that are compatible with both macOS and Linux.
 */

#include <cstdint>

namespace tf_lcm {
namespace compat {

/**
 * @brief Convert a hash function pointer to void* in a platform-independent way
 * 
 * This function safely converts an int64_t function pointer to void*.
 * It works on both macOS and Linux platforms.
 * 
 * @param func The hash function pointer to convert
 * @return void* A safely converted pointer
 */
inline void* hash_func_to_voidptr(int64_t (*func)()) {
    // Use a union to perform the conversion without compiler warnings
    union {
        int64_t (*func_ptr)();
        void* void_ptr;
    } converter;
    
    converter.func_ptr = func;
    return converter.void_ptr;
}

} // namespace compat
} // namespace tf_lcm

// Define macros to replace hash pointer creation
#define LCM_HASH_PTR(parent_ptr, hash_func) \
    { (parent_ptr), tf_lcm::compat::hash_func_to_voidptr(hash_func) }

#endif // TF_LCM_HASH_COMPAT_HPP
