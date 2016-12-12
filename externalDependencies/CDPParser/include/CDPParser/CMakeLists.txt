SET( H_FILES
	"CDPClient.h"
	"CDPPacket.h"
	"CDPFrame.h"
	"endian_osx.h"
	"data_item.h"
	"coordinates_data.h"
	"position.h"
	"mpu9250_quaternion.h"
	"UDPManager.h"
)

INSTALL(
	FILES ${H_FILES}
	DESTINATION include/CDPParser
)