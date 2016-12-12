#pragma once
#include <stdint.h>
#include <vector>
#ifdef __APPLE__
#include "endian_osx.h"
#else
#include <endian.h>
#endif

//! Contains meta information about each CDP Data Item.
/*!
 * @note Each field is transmitted in little endian.
 */
typedef struct __attribute__((__packed__)) {
    //! The type of the data.
    uint16_t type;
    //! The size of the data item in bytes.
    uint16_t size;
    //! A variable length buffer containing all of the data contained in the data item.
    uint8_t data[];
} cdp_data_item_t;

class CdpDataItem {
public:
    /*!
     * @name Class Functions
     */
    //@{
    //! Constructs \b this as an empty data item.
    CdpDataItem(void){
        mType = 0xffff;
        mSize = 0;
        _BuildData(NULL);
    }
    //! Constructs \b this.
    /*!
     * @param[in]    type    The CDP data type of the data item.
     * @param[in]    size    The size in bytes of the Data Item's data.
     * @param[in]    data    A pointer to the buffer that contains the data of the data item.
     */
    CdpDataItem(uint16_t type, uint16_t size, void *data){
        mType = type;
        mSize = size;
        _BuildData(data);
    }
    //! Constructs \b this.
    /*!
     * @pre \b data_item is in little endian.
     *
     * @param[in]   data_item   A buffer containing a CDP data item.
     */
    CdpDataItem(const uint8_t* data_item){
        cdp_data_item_t* _data_item = (cdp_data_item_t*)data_item;
        
        mSize = le16toh(_data_item->size);
        mType = le16toh(_data_item->type);
        _BuildData(_data_item->data);
    }
    //! Copies \b other into \b this.
    /*!
     * @param[in]   other   The other CDP data item to be copied.
     */
    CdpDataItem(const CdpDataItem& other){
        mType = other.mType;
        mSize = other.mSize;
        mData = other.mData;
    }
    //! Destroys \b this.
    ~CdpDataItem(void){}
    //@}
    
    /*!
     * @name Const Functions
     */
    //@{
    //! Returns the data item's data.
    /*!
     * @return A pointer to data item's data.
     */
    void* GetData(void) const{
        return (void*)(mData.data());
    }
    //! Returns the size of the data item in bytes.
    /*!
     * @return The size of the data item in bytes.
     */
    uint16_t GetSize(void) const{return mSize + sizeof(cdp_data_item_t);}
    //! Returns the type of the data item.
    /*!
     * @return The type of the data item.
     */
    uint16_t GetType(void) const{return mType;}
    //! Returns the size of the data item's data.
    /*!
     * @return The size of the data item's data.
     */
    uint16_t GetDataSize(void) const{return mSize;}
    //@}
    
    /*!
     * @name Modifier Functions
     */
    //@{
    //! Writes the CDP data item to \b buffer.
    /*!
     * @param[in,out]   buffer  The buffer to write the data item to.
     *
     * @return The number of bytes written to \b buffer.
     */
    uint16_t WriteToBuffer(uint8_t* buffer){
        cdp_data_item_t* _data_item = (cdp_data_item_t*)buffer;
        
        _data_item->size = htole16(mSize);
        _data_item->type = htole16(mType);
        memcpy(_data_item->data, mData.data(), GetDataSize());
        
        return GetSize();
    }
    //@}
    
private:
    //! The type of the data item.
    uint16_t mType;
    //! The size in bytes of the data item.
    uint16_t mSize;
    //! The data of the data item.
    std::vector<uint8_t> mData;
    
    //! Allocates and copies \b data to the data item's data.
    /*!
     * @pre #mSize has been set correctly.
     * @pre #mData has not been allocated.
     *
     * @param[in]   data    A pointer to the data to be copied.
     */
    void _BuildData(void *data){
        if (data == NULL) {
            return;
        }
        
        if (GetDataSize() > 0){
            uint16_t dataSize = GetDataSize();
            uint8_t* byteArray = (uint8_t*)data;
            
            for (uint16_t i = 0; i < dataSize; i++) {
                mData.push_back(byteArray[i]);
            }
        }
    }
};


/*!
 * @body cdpHeader
 * @copydoc cdp_data_item_t
 *
 * <table>
 * <caption id="cdpDataHeaderStructure">Cdp Data Header Structure</caption>
 * <tr><th> Name <th> ByteLength <th> Description
 * <tr><td> size <td> 2 <td> @copydoc cdp_data_item_t::size
 * <tr><td> type <td> 2 <td> @copydoc cdp_data_item_t::type
 * </table>
 */
