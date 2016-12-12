#pragma once

#include "data_item.h"
#ifdef __APPLE__
#include "endian_osx.h"
#include <tr1/memory>
#else
#include <endian.h>
#include <memory>
#endif
#include <deque>

using namespace std;
#ifdef __APPLE__
using namespace std::tr1;
#endif

#define CDP_DATATYPE_ANCHOR_STATUS (0x0103)
#define CDP_DATATYPE_ANNOUNCEMENT_STATUS (0x0104)
#define CDP_DATATYPE_DISTANCE (0x0101)
#define CDP_DATATYPE_INFRASTRUCTURE (0x0102)
#define CDP_DATATYPE_LPS25H_PRESSURE (0x0005)
#define CDP_DATATYPE_LPS25H_TEMPERATURE (0x0006)
#define CDP_DATATYPE_MPU9250_ACCELEROMETER (0x0008)
#define CDP_DATATYPE_MPU9250_GYROSCOPE (0x0009)
#define CDP_DATATYPE_MPU9250_MAGNETOMETER (0x0003)
#define CDP_DATATYPE_MPU9250_QUATERNION (0x000A)
#define CDP_DATATYPE_POSITION (0x0100)
#define CDP_DATATYPE_CUSTOM (0x0007)
#define CDP_DATATYPE_ACCELEROMETER (0x0001)
#define CDP_DATATYPE_GYRO (0x0002)
#define CDP_DATATYPE_QUAT (0x0004)

//! The 4 byte magic word for LCM.
/*!
 * @note This is in little endian.
 */
#define CDP_HEADER__MARK    (0x3230434c)
//! The string that identifies the LCM channel for CDP.
/*!
 * @note The header string "CDP0001" was used for a beta version of CDP that is no longer supported by the CUWB Server.
 */
#define CDP_HEADER__STRING  ("CDP0002")

//! The start of every CDP packet.
/*!
 * @note Unless otherwise specified, each field is transmitted in little endian.
 */
typedef struct __attribute__((__packed__)) CDP_HEADER_T {
    //! The 4 byte magic word for LCM (0x3230434C in little endian).
    uint32_t mark;
    //! The sequence number of the CDP packet.
    /*!
     * @note The sequence number is incremented on every transmission from a given CDP Connection.
     */
    uint32_t sequence;
    //! An ASCII string of "CDP0001" with a null terminator.
    /*!
     * @note This is not byte swapped.
     */
    char string[8];
    //! The serial number of the UWB device reporting the CDP packet.
    /*!
     * @note If the reporting device is unknown, 0 is used.
     */
    uint32_t serial_number;
} cdp_header_t;

//! The format of a full CDP packet.
/*!
 * @note This is transmitted in little endian.
 */
typedef struct __attribute__((__packed__)) {
    //! The CDP header.
    cdp_header_t header;
    //! A variable length buffer containing all of the data items contained in the packet.
    uint8_t data_items[];
} cdp_packet_t;

class CDPPacket{
public:
    CDPPacket(uint32_t serial_number = 0){
        mSerialNumber = serial_number;
        mSequence = 0;
    }
    //! Copies \b other into \b this.
    /*!
     * @param[in]   other   The other CDP packet to be copied.
     */
    CDPPacket(const CDPPacket& other){
        _CopyCdpPacket(other);
    }
    //! Destroys \b this.
    CDPPacket(void){ClearDataItems();}
    //! Assigns \b other to \b this.
    /*!
     * @param[in]   other   The other CdpPacket to be assigned.
     *
     * @return A reference to \b this.
     */
    CDPPacket& operator=(const CDPPacket& other){
        _CopyCdpPacket(other);
        
        return *this;
    }
    //@}
    
    /*!
     * @name Const Functions
     */
    //@{
    //! Returns the CDP data items in the packet.
    /*!
     * @return The CDP data items in the packet.
     */
    std::deque<shared_ptr<CdpDataItem>> GetDataItems(void) const{return mDataItems;}
    //! Returns the serial number of the packet.
    /*!
     * @return The serial number of the packet.
     */
    uint32_t GetSerialNumber(void) const{return mSerialNumber;}
    //! Returns the sequence number of the packet.
    /*!
     * @return The sequence number of the packet.
     */
    uint32_t GetSequence(void) const{return mSequence;}
    //! Checks if \b header is a valid CDP packet header.
    /*!
     * @pre \b header is stored in little endian.
     *
     * @param[in]   header  A CDP packet header.
     *
     * @return The validity of \b header being a CDP packet header.
     */
    bool HasValidHeader(const uint8_t* packet_buffer) const{
        cdp_header_t* _header = (cdp_header_t*)packet_buffer;
        
        if ((_header->mark == CDP_HEADER__MARK)
            && (memcmp(_header->string, CDP_HEADER__STRING, 8) == 0)) {
            return true;
        }
        
        return false;
    }
    //! Returns the size of the packet
    /*!
     * @return The size in bytes of the CDP packet.
     */
    uint16_t GetPacketSize(void) const{
        uint16_t _size = sizeof(cdp_header_t);
        
        
        for (shared_ptr<CdpDataItem> _data_item : mDataItems) {
            _size += _data_item->GetSize();
        }
        
        return _size;
    }
    //@}
    
    /*!
     * @name Modifier Functions
     */
    //@{
    //! Appends a CDP data item to the packet's data items.
    /*!
     * @param[in]   data_item   The CDP data item to add.
     */
    void AddDataItem(CdpDataItem* data_item){
        shared_ptr<CdpDataItem> _data_item(data_item);
        
        //    printType(_data_item);
        mDataItems.push_back(_data_item);
        
    }
    //! Constructs a CDP data item and appends it to the packet's data items.
    /*!
     * @param[in]   type    The CDP type of the data item to be added.
     * @param[in]   size    The size in bytes of \b data.
     * @param[in]   data    A pointer to the data that makes up the data item to be added.
     */
    void AddDataItem(uint16_t type, uint16_t size, void *data){
        CdpDataItem* _data_item = new CdpDataItem(type, size, data);
        
        AddDataItem(_data_item);
    }
    //! Deletes the packet's data items.
    void ClearDataItems(void){mDataItems.clear();}
    //! Sets the CDP packet's serial number.
    /*!
     * @param[in]   serial_number   The serial_number of the CDP packet.
     */
    void SetSerialNumber(uint32_t serial_number){mSerialNumber=serial_number;}
    //! Allocates and writes a buffer that contains the full CDP packet.
    /*!
     * @param[in]   sequence_number The sequence number of the CDP packet.
     *
     * @return A CDP packet buffer.
     */
    uint8_t* CreateBufferWithSequenceNumber(uint32_t sequence_number){
        cdp_header_t _header;
        uint16_t _header_size = sizeof(cdp_header_t);
        uint16_t _index = _header_size;
        
        mSequence = sequence_number;
        _header.mark = CDP_HEADER__MARK;
        _header.sequence = htole32(mSequence);
        strcpy(_header.string, CDP_HEADER__STRING);
        _header.serial_number = htole32(mSerialNumber);
        
        uint8_t* _buffer = new uint8_t[GetPacketSize()];
        
        memcpy(_buffer, &_header, _header_size);
        
        for (shared_ptr<CdpDataItem> _data_item : mDataItems) {
            _index += _data_item->WriteToBuffer(&(_buffer[_index]));
        }
        
        return _buffer;
    }
    //! Sets all of the data contained in \b packet_buffer for the CDP packet.
    /*!
     * @pre \b packet_buffer is in little endian.
     * @pre The packet does not contain any data items before being called.
     *
     * @param[in]   packet_buffer   A buffer containing the CDP packet.
     * @param[in]   size            The size in bytes of \b packet_buffer.
     */
    void printType(shared_ptr<CdpDataItem> data_item){
        switch (data_item->GetType()) {
            case CDP_DATATYPE_ANCHOR_STATUS:
                cout<<"CDP_DATATYPE_ANCHOR_STATUS"<<endl;
                break;
            case CDP_DATATYPE_INFRASTRUCTURE:
                cout<<"CDP_DATATYPE_INFRASTRUCTURE"<<endl;
                break;
            case CDP_DATATYPE_MPU9250_QUATERNION:
                cout<<"CDP_DATATYPE_MPU9250_QUATERNION"<<endl;
                break;
            case CDP_DATATYPE_POSITION:
                cout<<"CDP_DATATYPE_POSITION"<<endl;
                break;
            case CDP_DATATYPE_DISTANCE:
                cout<<"CDP_DATATYPE_DISTANCE"<<endl;
                break;
            case CDP_DATATYPE_LPS25H_PRESSURE:
                cout<<"CDP_DATATYPE_LPS25H_PRESSURE"<<endl;
                break;
            case CDP_DATATYPE_MPU9250_GYROSCOPE:
                cout<<"CDP_DATATYPE_MPU9250_GYROSCOPE"<<endl;
                break;
            case CDP_DATATYPE_LPS25H_TEMPERATURE:
                cout<<"CDP_DATATYPE_LPS25H_TEMPERATURE"<<endl;
                break;
            case CDP_DATATYPE_ANNOUNCEMENT_STATUS:
                cout<<"CDP_DATATYPE_ANNOUNCEMENT_STATUS"<<endl;
                break;
            case CDP_DATATYPE_GYRO:
                cout<<"CDP_DATATYPE_GYRO"<<endl;
                break;
            case CDP_DATATYPE_QUAT:
                cout<<"CDP_DATATYPE_QUAT"<<endl;
                break;
            case CDP_DATATYPE_ACCELEROMETER:
                cout<<"CDP_DATATYPE_ACCELEROMETER"<<endl;
                break;
            default:
                //            cout<<"UNDEFINED TYPE 0x"<<ofToHex(data_item->GetType())<<endl;
                break;
        }
    }
    void ConvertPacketBuffer(const uint8_t* packet_buffer, uint16_t size){
        uint16_t _index = 0;
        uint16_t _data_items_size = (uint16_t)(sizeof(packet_buffer) - sizeof(cdp_header_t));
        cdp_packet_t* _packet = (cdp_packet_t*)packet_buffer;
        
        mSerialNumber = le32toh(_packet->header.serial_number);
        mSequence = le32toh(_packet->header.sequence);
        
        while (_index < _data_items_size) {
            CdpDataItem* _data_item = new CdpDataItem(_packet->data_items + _index);
            if(_data_item->GetType() == 0){
                break;
            }
            _index += _data_item->GetSize();
            if(_data_item->GetSize() > 0){
                AddDataItem(_data_item);
            }else{
                break;
            }
        }
    }
    //@}

    
private:
    //! The serial number of the packet.
    uint32_t mSerialNumber;
    //! The sequence number of the packet.
    uint32_t mSequence;
    //! The data items of the packet.

    std::deque<shared_ptr<CdpDataItem>> mDataItems;
    
    //! Copies \b other to \b this.
    /*!
     * @param[in]   other   The other CdpPacket to be copied.
     */
    void _CopyCdpPacket(const CDPPacket& other){
        ClearDataItems();
        mSerialNumber = other.mSerialNumber;
        mSequence = other.mSequence;

        mDataItems = std::deque<shared_ptr<CdpDataItem>>(other.mDataItems);
    }
};