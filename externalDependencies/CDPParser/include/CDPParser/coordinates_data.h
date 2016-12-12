// ALL RIGHTS RESERVED - Copyright 2016 - Ciholas, Inc.
// Any use of this code without explicit written permission is prohibited

#ifndef _UTILS__COORDINATES_DATA__H_
#define _UTILS__COORDINATES_DATA__H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


//! Contains the X, Y, and Z coordinates from a separately defined location.
/*!
 * @note Each field is transmitted in little endian.
 */
typedef struct __attribute__((__packed__)) {
    //! The distance in millimeters from the location along the X axis.
    /*!
     * This value is a signed two's complement integer.
     */
    int32_t x;
    //! The distance in millimeters from the location along the Y axis.
    /*!
     * This value is a signed two's complement integer.
     */
    int32_t y;
    //! The distance in millimeters from the location along the Z axis.
    /*!
     * This value is a signed two's complement integer.
     */
    int32_t z;
} coordinates_t;

#ifdef __cplusplus
}
#endif

#endif // _UTILS__COORDINATES_DATA__H_

/*!
 * @body coordinates
 * @copydoc coordinates_t
 *
 * <table>
 * <caption id="coordinatesStructure">Coordinates Structure</caption>
 * <tr><th> Name <th> Byte Length <th> Description
 * <tr><td> x <td> 4 <td> @copydoc coordinates_t::x
 * <tr><td> y <td> 4 <td> @copydoc coordinates_t::y
 * <tr><td> z <td> 4 <td> @copydoc coordinates_t::z
 * </table>
 */
