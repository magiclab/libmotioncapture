// ALL RIGHTS RESERVED - Copyright 2016 - Ciholas, Inc.
// Any use of this code without explicit written permission is prohibited

#ifndef _UTILS__CDP__DATATYPES__MPU9250_QUATERNION__H_
#define _UTILS__CDP__DATATYPES__MPU9250_QUATERNION__H_

#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include <stdint.h>
    
    
    //! The type for a MPU9250 quaternion data item.
#define CDP_DATATYPE_MPU9250_QUATERNION (0x000A)
    
    //! Reports the quaternion data from an onboard MPU-9250.
    /*!
     * @headerfile mpu9250_quaternion.h "utils/cdp/datatypes/mpu9250_quaternion.h"
     *
     * @note Each field is transmitted in little endian.
     */
    typedef struct __attribute__((__packed__)) {
        //! The raw X quaternion value.
        /*!
         * @_mpu9250_datasheet
         */
        int32_t x;
        //! The raw Y quaternion value.
        /*!
         * @_mpu9250_datasheet
         */
        int32_t y;
        //! The raw Z quaternion value.
        /*!
         * @_mpu9250_datasheet
         */
        int32_t z;
        //! The raw W quaternion value.
        /*!
         * @_mpu9250_datasheet
         */
        int32_t w;
        //! The timestamp when the MPU-9250 recorded the data.
        /*!
         * @_gnt
         */
        uint32_t gnt;
    } cdp_data_mpu9250_quaternion_t;
    
    typedef struct __attribute__((__packed__)) {
        //! The raw quaternion value.
        /*!
         * @_mpu9250_datasheet
         */
        int32_t xyzw[4];
        uint32_t gnt;
    } quaternion_data;;
    
    
    // If we are in C++, add a class definition around the helper function(s) to prevent compiler warnings.
#ifdef __cplusplus
    
    //! Provides functionality involving quaternion data from the MPU9250.
    /*!
     * @headerfile mpu9250_quaternion.h "utils/cdp/datatypes/mpu9250_quaternion.h"
     */
    class Mpu9250Quaternion {
    public:
#endif
        
        //! Converts the raw quaterion value from the MPU9250 into a quaternion value.
        /*!
         * @param[in]   value   The raw value from the MPU9250.
         *
         * @return The quaternion value.
         */
        static double ConvertRawValueToQuaternion(int32_t value) {
            double _rv = value;
            return _rv / pow(2, 30);
        }
        
#ifdef __cplusplus
    };
}
#endif

#endif // _UTILS__CDP__DATATYPES__MPU9250_QUATERNION__H_

/*!
 * @body mpu9250QuaternionV1
 * @copydoc cdp_data_mpu9250_quaternion_t
 *
 * <table>
 * <caption id="mpu9250QuaternionV1Structure">MPU-9250 Quaternion V1 Structure</caption>
 * <tr><th> Name <th> Byte Length <th> Description
 * <tr><td> x <td> 4 <td> @copydoc cdp_data_mpu9250_quaternion_t::x
 * <tr><td> y <td> 4 <td> @copydoc cdp_data_mpu9250_quaternion_t::y
 * <tr><td> z <td> 4 <td> @copydoc cdp_data_mpu9250_quaternion_t::z
 * <tr><td> w <td> 4 <td> @copydoc cdp_data_mpu9250_quaternion_t::w
 * <tr><td> gnt <td> 4 <td> @copydoc cdp_data_mpu9250_quaternion_t::gnt
 * </table>
 */
