#ifndef USBD_DESC_H_
#define USBD_DESC_H_

#include "usbd_def.h"

#define DEVICE_ID1 (UID_BASE)
#define DEVICE_ID2 (UID_BASE + 0x4)
#define DEVICE_ID3 (UID_BASE + 0x8)

#define  USB_SIZ_STRING_SERIAL 0x1A

extern USBD_DescriptorsTypeDef FS_Desc;

#endif
