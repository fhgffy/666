#ifndef SNDCARDDRIVER_H
#define SNDCARDDRIVER_H

#include <sysTypes.h>
#define VENDOR_ID	0x1d87
#define	DEVICE_ID	0x356a

/****************************************
* @function brief 	    : snd card initilized
* @para input dev	    : number of pcie controller
* @para input vendorID	: pcie vendor id
* @para input deviceId	: pcie device id
* @para input index	    : index of pcie bus 
* @return 			    : success 0,otherwise -1
* @note				    : none
****************************************/
int32_t sndCardInit(uint32_t dev, uint16_t vendorID, uint16_t deviceId, uint32_t index);

/****************************************
* @function brief 	    : snd card released
* @para input dev	    : number of pcie controller
* @para input vendorID	: pcie vendor id
* @para input deviceId	: pcie device id
* @para input index	    : index of pcie bus 
* @return 			    : none
* @note				    : none
****************************************/
void sndCardRelease(uint32_t dev, uint16_t vendorID, uint16_t deviceId, uint32_t index);

/****************************************
* @function brief 	    : get data from snd card
* @para input data	    : point to address of data
* @para int len	        : length of get data
* @para output actualLen: length of actual get data
* @para input flag	    : =0,nonblock, >0 wait us,<0 block,default use -1
* @return 			    : success 0,otherwise -1
* @note				    : check actualLen,actualLen=0 indicate data not ready
****************************************/
int32_t sndCardGetdata(uint8_t *data, int32_t len, int32_t *actualLen, int32_t flag);

/****************************************
* @function brief 	    : get temperature
* @para input data	    : point to address of data
* @para input len	    : length of get data
* @return 			    : success 0,otherwise -1
* @note				    : none
****************************************/
int32_t sndCardPutdata(uint8_t *data, int32_t len);

#endif
