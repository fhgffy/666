
#include "sndCardDriver.h"
#include "ac653ApexTypes.h"
//#include "apexProcess.h"
#include <os/pos/apex/apexLib.h>
#define ELBI_APP		(0x0e00)  	/* CFG: ELBI Application Register */

/* pcie config space register */
#define CONFIG_SPACE_BAR_0_ADDR					(0x10)

/* elbi register */
#define USP_PCIE_ELBI_APP_ELBI_INT_GEN0			(0x00)
#define USP_PCIE_ELBI_APP_ELBI_INT_GEN1			(0x04)
#define USP_PCIE_ELBI_APP_ELBI_INTMSK0			(0x08)
#define USP_PCIE_ELBI_APP_ELBI_INTMSK1			(0x0C)
#define USP_PCIE_ELBI_APP_ELBI_USER4			(0x10)		
#define USP_PCIE_ELBI_APP_ELBI_USER5			(0x14)			
#define USP_PCIE_ELBI_APP_ELBI_USER6			(0x18)

#define SIZE_LINIT 		(0x200000)    //2MB

uint32_t send_cnt = 0;
uint32_t recv_cnt = 0;
typedef struct pcie_private_data{
	uint8_t opened;
	int32_t dev;
	int32_t pBusNo;
	int32_t pDeviceNo;
	int32_t pFuncNo;
	uint32_t bar0Addr;
}G_VAR;

G_VAR gVar = {0, 0, 0, 0, 0, 0};

/****************************************
* @function brief 	    : snd card initilized
* @para input dev	    : number of pcie controller
* @para input vendorID	: pcie vendor id
* @para input deviceId	: pcie device id
* @para input index	    : index of pcie bus 
* @return 			    : success 0,otherwise -1
* @note				    : none
****************************************/
int32_t sndCardInit(uint32_t dev, uint16_t vendorID, uint16_t deviceId, uint32_t index)
{
	int32_t ret = -1, rev = -1;
	if(gVar.opened)
	{
		printf("snd card already initilized\n");
		return 0;
	}

	ret = bspPciFindDevice(dev, vendorID, deviceId, index, &gVar.pBusNo, &gVar.pDeviceNo, &gVar.pFuncNo);
	if(ret == 0)
	{
		printf("sndCardDriver : bspPciFindDevice success %x-%x\n", vendorID, deviceId);
		gVar.dev = dev;

		gVar.dev = 0;
		gVar.pBusNo = 0x1;
		gVar.pDeviceNo = 0x0;
		gVar.pFuncNo = 0x0;

		ret = bspPciConfigInLong(gVar.dev, gVar.pBusNo, gVar.pDeviceNo, gVar.pFuncNo, \
				CONFIG_SPACE_BAR_0_ADDR, &gVar.bar0Addr);
		if(ret == 0)
		{
			printf("sndCardDriver : bspPciConfigInLong bar0 addr : %x\n", gVar.bar0Addr);
			gVar.opened = 1;
			rev = 0;
		}
		else
		{
			printf("sndCardDriver : bspPciConfigInLong error %d\n", ret);
			rev = -1;
		}

	}
	else
	{
		printf("sndCardDriver : bspPciFindDevice error %d\n", ret);
		rev = -1;
	}

	return rev;
}

/****************************************
* @function brief 	    : snd card released
* @para input dev	    : number of pcie controller
* @para input vendorID	: pcie vendor id
* @para input deviceId	: pcie device id
* @para input index	    : index of pcie bus 
* @return 			    : none
* @note				    : none
****************************************/
void sndCardRelease(uint32_t dev, uint16_t vendorID, uint16_t deviceId, uint32_t index)
{
	if(!gVar.opened)
	{
		printf("sndCardDriver : sndCardRelease snd card don't initilized\n");
		return;
	}
	gVar.opened = 0;
	return;
}

/****************************************
* @function brief 	    : get data from snd card
* @para input data	    : point to address of data
* @para int len	        : length of get data
* @para output actualLen: length of actual get data
* @para input flag	    : =0,nonblock, >0 wait us,<0 block,default use -1
* @return 			    : success 0,otherwise -1
* @note				    : none
****************************************/
int32_t sndCardGetdata(uint8_t *data, int32_t len, int32_t *actualLen, int32_t flag)
{
	int32_t ret = -1;
	uint32_t pData = 0;
	int32_t cnt = flag;
	*actualLen = 0;
	RETURN_CODE_TYPE retCode = 0;

	if(!gVar.opened)
	{
		printf("sndCardDriver : sndCardGetdata snd card don't initilized\n");
		return -1;		
	}

	if(len > SIZE_LINIT)
	{
		printf("sndCardDriver : sndCardGetdata len over 2MB\n");
		return -1;
	}

	if(data == NULL)
	{
		printf("sndCardDriver : sndCardGetdata point data is NULL\n");
		return -1;
	}

	while(1)
	{
		//data ready ?
		ret = bspPciConfigInLong(gVar.dev, gVar.pBusNo, gVar.pDeviceNo, gVar.pFuncNo, \
				(ELBI_APP + USP_PCIE_ELBI_APP_ELBI_USER6), &pData);

		if(ret != 0)
		{
			printf("sndCardDriver : sndCardGetdata bspPciConfigInLong error %d\n", ret);
			ret = -1;
			break;
		}

		if(pData == 0x1)		//ready
		{
			//get data size
			ret = bspPciConfigInLong(gVar.dev, gVar.pBusNo, gVar.pDeviceNo, gVar.pFuncNo, \
					(ELBI_APP + USP_PCIE_ELBI_APP_ELBI_USER5), &pData);
			if(ret != 0) // 0
			{
				printf("sndCardDriver : sndCardGetdata bspPciConfigInLong error %d\n", ret);
				ret = -1;
				break;
			}

			if(pData != 0)
			{
				//memcpy(data, ((uint32_t *)gVar.bar0Addr + SIZE_LINIT), pData);
				memcpy(data, (gVar.bar0Addr + SIZE_LINIT), pData);
			}

			ret = bspPciConfigOutLong(gVar.dev, gVar.pBusNo, gVar.pDeviceNo, gVar.pFuncNo, \
					(ELBI_APP + USP_PCIE_ELBI_APP_ELBI_USER6), 0x0);
			if(ret != 0)
			{
				printf("sndCardDriver : sndCardGetdata bspPciConfigOutLong error %d\n", ret);
				ret = -1;
				break;
			}
			*actualLen = pData;
			ret = 0;
			recv_cnt ++;
			if(recv_cnt > 1000000)
				recv_cnt = 0;
//			printf("PCIe recv cnt : %lu\n", recv_cnt);
			break;
		}
		else
		{
			if(flag == 0) // !0
			{
//				printf("sndCardDriver : sndCardGetdata data not ready\n");
				ret = -1;
				break;
			}
			else if(flag < 0)
			{
				// usleep(1);

				SUSPEND_SELF(10000,&retCode);  //10us
				continue;
			}
			else
			{
				if(cnt == 0)
				{
//					printf("sndCardDriver : sndCardGetdata timeout\n");
					ret = -1;
					break;
				}
				// usleep(1);
				SUSPEND_SELF(1000,&retCode);
				cnt--;
				continue;
			}
		}
	}	
	return ret;
}

/****************************************
* @function brief 	    : get temperature
* @para input data	    : point to address of data
* @para input len	    : length of get data
* @return 			    : success 0,otherwise -1
* @note				    : none
****************************************/
int32_t sndCardPutdata(uint8_t *data, int32_t len)
{
	int32_t ret = -1;

	if(!gVar.opened)
	{
		printf("sndCardDriver : sndCardPutdata snd card don't initilized\n");
		return -1;		
	}

	if(len > SIZE_LINIT)
	{
		printf("sndCardDriver : sndCardPutdata len over 2MB\n");
		return -1;
	}

	if(data == NULL)
	{
		printf("sndCardDriver : sndCardPutdata point data is NULL\n");
		return -1;
	}

	//memcpy((uint32_t *)gVar.bar0Addr, data, len);
	memcpy(gVar.bar0Addr, data, len);

	ret = bspPciConfigOutLong(gVar.dev, gVar.pBusNo, gVar.pDeviceNo, gVar.pFuncNo, \
			(ELBI_APP + USP_PCIE_ELBI_APP_ELBI_USER4), len);

	if(ret != 0)
	{
		printf("sndCardDriver : sndCardPutdata bspPciConfigOutLong error %d\n", ret);
		return -1;
	}

	ret = bspPciConfigOutLong(gVar.dev, gVar.pBusNo, gVar.pDeviceNo, gVar.pFuncNo, \
		(ELBI_APP + USP_PCIE_ELBI_APP_ELBI_INT_GEN0), 0x00010001);
	if(ret != 0)
	{
		printf("sndCardDriver : sndCardPutdata bspPciConfigOutLong error %d\n", ret);
		return -1;
	}
	
	send_cnt ++;
	if(send_cnt > 1000000)
		send_cnt = 0;
//	printf("PCIe send cnt : %lu\n", send_cnt);

	return 0;
}
