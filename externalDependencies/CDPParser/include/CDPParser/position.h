// ALL RIGHTS RESERVED - Copyright 2016 - Ciholas, Inc.
// Any use of this code without explicit written permission is prohibited

#ifndef _UTILS__CDP__DATATYPES__POSITION__H_
#define _UTILS__CDP__DATATYPES__POSITION__H_

#ifdef __cplusplus
extern "C" {
#endif
#include "coordinates_data.h"
#include <stdint.h>


//! The type for a position data item.
#define CDP_DATATYPE_POSITION (0x0100)

//! Reports the 3-dimensional position of the reporting device.
/*!
 * @headerfile position.h "utils/cdp/datatypes/position.h"
 *
 * @note Each field is transmitted in little endian.
 */
typedef struct __attribute__((__packed__)) {
    //! The coordinates from the origin.
    coordinates_t coordinates;
    //! Currently unused
    uint32_t quality;
    //! The effective smoothing factor (the number of positions averaged minus 1).
    uint16_t smoothing;
    //! The sequence number of the packet from the reporting device that was used to calculate the position.
    /*!
     * This value is helpful in determining if any packets have been missed.
     *
     * @note Currently, the sequence number is stored as a one byte number, so it will wrap at 255. CDP will represent the sequence number as 2 bytes for future compatibility.
     */
    uint16_t sequence;
    //! The calculated timestamp of the transmission of the packet from the reporting device used to calculate the position.
    /*!
     * @_gnt
     */
    uint32_t gnt;
} cdp_data_position_t;

#ifdef __cplusplus
}
#endif

#endif // _UTILS__CDP__DATATYPES__POSITION__H_

/*!
 * @body positionV1
 * @copydoc cdp_data_position_t
 *
 * <table>
 * <caption id="positionV1Structure">Position V1 Structure</caption>
 * <tr><th> Name <th> Byte Length <th> Description
 * <tr><td> coordinates <td> 12 <td> @copydoc cdp_data_position_t::coordinates
 * <tr><td> quality <td> 4 <td> @copydoc cdp_data_position_t::quality
 * <tr><td> smoothing <td> 2 <td> @copydoc cdp_data_position_t::smoothing
 * <tr><td> sequence <td> 2 <td> @copydoc cdp_data_position_t::sequence
 * <tr><td> gnt <td> 4 <td> @copydoc cdp_data_position_t::gnt
 * </table>
 */
