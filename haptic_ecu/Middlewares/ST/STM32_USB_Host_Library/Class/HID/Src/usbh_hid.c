/**
  ******************************************************************************
  * @file    usbh_hid.c
  * @author  MCD Application Team
  * @brief   USB Host HID class
  *
  * Modified for SIMAGIC P2000 Generic HID support
  ******************************************************************************
  */

#include "usbh_hid.h"
#include "usbh_hid_parser.h"

/* Private function prototypes -----------------------------------------------*/
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost);
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf);

/* P2000 Generic HID */
static USBH_StatusTypeDef USBH_HID_GenericInit(USBH_HandleTypeDef *phost);

extern USBH_StatusTypeDef USBH_HID_MouseInit(USBH_HandleTypeDef *phost);
extern USBH_StatusTypeDef USBH_HID_KeybdInit(USBH_HandleTypeDef *phost);


/* HID Class structure -------------------------------------------------------*/
USBH_ClassTypeDef HID_Class =
{
  "HID",
  USB_HID_CLASS,
  USBH_HID_InterfaceInit,
  USBH_HID_InterfaceDeInit,
  USBH_HID_ClassRequest,
  USBH_HID_Process,
  USBH_HID_SOFProcess,
  NULL,
};


/**
  * @brief  USBH_HID_InterfaceInit
  */
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status;
  HID_HandleTypeDef *HID_Handle;
  uint16_t ep_mps;
  uint8_t max_ep;
  uint8_t num = 0U;
  uint8_t interface;

  /*
   * Original ST driver searches only Boot HID.
   *
   * P2000:
   * Class    = 0x03
   * SubClass = 0x00
   * Protocol = 0x00
   *
   * Therefore accept any HID subclass/protocol.
   */
  interface = USBH_FindInterface(phost,
                                 phost->pActiveClass->ClassCode,
                                 0xFFU,
                                 0xFFU);

  if ((interface == 0xFFU) ||
      (interface >= USBH_MAX_NUM_INTERFACES))
  {
    USBH_DbgLog("Cannot Find the interface for %s class.",
                phost->pActiveClass->Name);

    return USBH_FAIL;
  }

  status = USBH_SelectInterface(phost, interface);

  if (status != USBH_OK)
  {
    return USBH_FAIL;
  }

  phost->pActiveClass->pData =
      (HID_HandleTypeDef *)USBH_malloc(sizeof(HID_HandleTypeDef));

  HID_Handle =
      (HID_HandleTypeDef *)phost->pActiveClass->pData;

  if (HID_Handle == NULL)
  {
    USBH_DbgLog("Cannot allocate memory for HID Handle");

    return USBH_FAIL;
  }

  /* Initialize HID handler */
  (void)USBH_memset(HID_Handle,
                    0,
                    sizeof(HID_HandleTypeDef));

  HID_Handle->state = USBH_HID_ERROR;


  /* =========================================================
   * HID device type detection
   * ========================================================= */

  /* Keyboard */
  if ((phost->device.CfgDesc.Itf_Desc[interface].bInterfaceSubClass
          == HID_BOOT_CODE) &&
      (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol
          == HID_KEYBRD_BOOT_CODE))
  {
    USBH_UsrLog("KeyBoard device found!");

    HID_Handle->Init = USBH_HID_KeybdInit;
  }

  /* Mouse */
  else if ((phost->device.CfgDesc.Itf_Desc[interface].bInterfaceSubClass
               == HID_BOOT_CODE) &&
           (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol
               == HID_MOUSE_BOOT_CODE))
  {
    USBH_UsrLog("Mouse device found!");

    HID_Handle->Init = USBH_HID_MouseInit;
  }

  /* Generic HID - SIMAGIC P2000 */
  else if ((phost->device.CfgDesc.Itf_Desc[interface].bInterfaceClass
                == USB_HID_CLASS) &&
           (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceSubClass
                == 0x00U) &&
           (phost->device.CfgDesc.Itf_Desc[interface].bInterfaceProtocol
                == 0x00U))
  {
    USBH_UsrLog("Generic HID device found!");

    HID_Handle->Init = USBH_HID_GenericInit;
  }

  else
  {
    USBH_UsrLog("HID protocol not supported.");

    USBH_free(phost->pActiveClass->pData);
    phost->pActiveClass->pData = NULL;

    return USBH_FAIL;
  }


  HID_Handle->state     = USBH_HID_INIT;
  HID_Handle->ctl_state = USBH_HID_REQ_INIT;

  /*
   * Endpoint information
   */
  HID_Handle->ep_addr =
      phost->device.CfgDesc.Itf_Desc[interface]
          .Ep_Desc[0].bEndpointAddress;

  HID_Handle->length =
      phost->device.CfgDesc.Itf_Desc[interface]
          .Ep_Desc[0].wMaxPacketSize;

  HID_Handle->poll =
      phost->device.CfgDesc.Itf_Desc[interface]
          .Ep_Desc[0].bInterval;

  if (HID_Handle->poll < HID_MIN_POLL)
  {
    HID_Handle->poll = HID_MIN_POLL;
  }


  /* Number of endpoints */
  max_ep =
      ((phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints
        <= USBH_MAX_NUM_ENDPOINTS) ?

       phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints :

       USBH_MAX_NUM_ENDPOINTS);


  /* Decode endpoint IN / OUT */
  for (num = 0U; num < max_ep; num++)
  {
    if ((phost->device.CfgDesc.Itf_Desc[interface]
             .Ep_Desc[num].bEndpointAddress & 0x80U) != 0U)
    {
      HID_Handle->InEp =
          phost->device.CfgDesc.Itf_Desc[interface]
              .Ep_Desc[num].bEndpointAddress;

      HID_Handle->InPipe =
          USBH_AllocPipe(phost,
                         HID_Handle->InEp);

      ep_mps =
          phost->device.CfgDesc.Itf_Desc[interface]
              .Ep_Desc[num].wMaxPacketSize;

      (void)USBH_OpenPipe(phost,
                          HID_Handle->InPipe,
                          HID_Handle->InEp,
                          phost->device.address,
                          phost->device.speed,
                          USB_EP_TYPE_INTR,
                          ep_mps);

      (void)USBH_LL_SetToggle(phost,
                              HID_Handle->InPipe,
                              0U);
    }
    else
    {
      HID_Handle->OutEp =
          phost->device.CfgDesc.Itf_Desc[interface]
              .Ep_Desc[num].bEndpointAddress;

      HID_Handle->OutPipe =
          USBH_AllocPipe(phost,
                         HID_Handle->OutEp);

      ep_mps =
          phost->device.CfgDesc.Itf_Desc[interface]
              .Ep_Desc[num].wMaxPacketSize;

      (void)USBH_OpenPipe(phost,
                          HID_Handle->OutPipe,
                          HID_Handle->OutEp,
                          phost->device.address,
                          phost->device.speed,
                          USB_EP_TYPE_INTR,
                          ep_mps);

      (void)USBH_LL_SetToggle(phost,
                              HID_Handle->OutPipe,
                              0U);
    }
  }

  return USBH_OK;
}


/**
  * @brief Generic HID initialization
  *
  * P2000 is controlled through Feature Reports.
  * Keyboard/Mouse FIFO initialization is not required.
  */
static USBH_StatusTypeDef USBH_HID_GenericInit(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle;

  HID_Handle =
      (HID_HandleTypeDef *)phost->pActiveClass->pData;

  if (HID_Handle == NULL)
  {
    return USBH_FAIL;
  }

  USBH_UsrLog("Generic HID initialized");

  return USBH_OK;
}


/**
  * @brief USBH_HID_InterfaceDeInit
  */
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle =
      (HID_HandleTypeDef *)phost->pActiveClass->pData;

  if (HID_Handle != NULL)
  {
    if (HID_Handle->InPipe != 0x00U)
    {
      (void)USBH_ClosePipe(phost,
                           HID_Handle->InPipe);

      (void)USBH_FreePipe(phost,
                          HID_Handle->InPipe);

      HID_Handle->InPipe = 0U;
    }

    if (HID_Handle->OutPipe != 0x00U)
    {
      (void)USBH_ClosePipe(phost,
                           HID_Handle->OutPipe);

      (void)USBH_FreePipe(phost,
                          HID_Handle->OutPipe);

      HID_Handle->OutPipe = 0U;
    }
  }

  if (phost->pActiveClass->pData != NULL)
  {
    USBH_free(phost->pActiveClass->pData);

    phost->pActiveClass->pData = NULL;
  }

  return USBH_OK;
}


/**
  * @brief USBH_HID_ClassRequest
  */
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_BUSY;
  USBH_StatusTypeDef classReqStatus = USBH_BUSY;

  HID_HandleTypeDef *HID_Handle =
      (HID_HandleTypeDef *)phost->pActiveClass->pData;

  uint8_t interface =
      phost->device.current_interface;

  uint8_t is_generic =
      ((phost->device.CfgDesc.Itf_Desc[interface]
            .bInterfaceSubClass == 0x00U) &&
       (phost->device.CfgDesc.Itf_Desc[interface]
            .bInterfaceProtocol == 0x00U));


  switch (HID_Handle->ctl_state)
  {
    case USBH_HID_REQ_INIT:

    case USBH_HID_REQ_GET_HID_DESC:

      USBH_HID_ParseHIDDesc(
          &HID_Handle->HID_Desc,
          phost->device.CfgDesc_Raw);

      HID_Handle->ctl_state =
          USBH_HID_REQ_GET_REPORT_DESC;

      break;


    case USBH_HID_REQ_GET_REPORT_DESC:

      classReqStatus =
          USBH_HID_GetHIDReportDescriptor(
              phost,
              HID_Handle->HID_Desc.wItemLength);

      if (classReqStatus == USBH_OK)
      {
        /*
         * P2000 Generic HID does not require
         * Boot HID SET_IDLE / SET_PROTOCOL.
         */
        if (is_generic != 0U)
        {
          HID_Handle->ctl_state =
              USBH_HID_REQ_IDLE;

          phost->pUser(phost,
                       HOST_USER_CLASS_ACTIVE);

          status = USBH_OK;
        }
        else
        {
          HID_Handle->ctl_state =
              USBH_HID_REQ_SET_IDLE;
        }
      }

      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
        USBH_ErrLog(
            "Control error: HID: Device Get Report Descriptor request failed");

        status = USBH_FAIL;
      }

      break;


    case USBH_HID_REQ_SET_IDLE:

      /*
       * Generic HID should normally never reach here.
       */
      if (is_generic != 0U)
      {
        HID_Handle->ctl_state =
            USBH_HID_REQ_IDLE;

        phost->pUser(phost,
                     HOST_USER_CLASS_ACTIVE);

        status = USBH_OK;

        break;
      }

      classReqStatus =
          USBH_HID_SetIdle(phost,
                           0U,
                           0U);

      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state =
            USBH_HID_REQ_SET_PROTOCOL;
      }

      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
        HID_Handle->ctl_state =
            USBH_HID_REQ_SET_PROTOCOL;
      }

      break;


    case USBH_HID_REQ_SET_PROTOCOL:

      /*
       * Generic HID should normally never reach here.
       */
      if (is_generic != 0U)
      {
        HID_Handle->ctl_state =
            USBH_HID_REQ_IDLE;

        phost->pUser(phost,
                     HOST_USER_CLASS_ACTIVE);

        status = USBH_OK;

        break;
      }

      classReqStatus =
          USBH_HID_SetProtocol(phost,
                               0U);

      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state =
            USBH_HID_REQ_IDLE;

        phost->pUser(phost,
                     HOST_USER_CLASS_ACTIVE);

        status = USBH_OK;
      }

      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
        USBH_ErrLog(
            "Control error: HID: Device Set protocol request failed");

        status = USBH_FAIL;
      }

      break;


    case USBH_HID_REQ_IDLE:

      status = USBH_OK;

      break;


    default:

      break;
  }

  return status;
}


/**
  * @brief USBH_HID_Process
  */
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_OK;

  HID_HandleTypeDef *HID_Handle =
      (HID_HandleTypeDef *)phost->pActiveClass->pData;

  uint32_t XferSize;

  uint8_t interface =
      phost->device.current_interface;


  /*
   * P2000 Generic HID
   *
   * No continuous Input Report polling is required.
   * Feature Reports are sent from main.c using
   * USBH_HID_SetReport().
   */
  if ((phost->device.CfgDesc.Itf_Desc[interface]
           .bInterfaceSubClass == 0x00U) &&
      (phost->device.CfgDesc.Itf_Desc[interface]
           .bInterfaceProtocol == 0x00U))
  {
    return USBH_OK;
  }


  switch (HID_Handle->state)
  {
    case USBH_HID_INIT:

      status = HID_Handle->Init(phost);

      if (status == USBH_OK)
      {
        HID_Handle->state =
            USBH_HID_IDLE;
      }
      else
      {
        USBH_ErrLog(
            "HID Class Init failed");

        HID_Handle->state =
            USBH_HID_ERROR;

        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      USBH_OS_PutMessage(phost,
                         USBH_URB_EVENT,
                         0U,
                         0U);
#endif

      break;


    case USBH_HID_IDLE:

      status =
          USBH_HID_GetReport(
              phost,
              0x01U,
              0U,
              HID_Handle->pData,
              (uint8_t)HID_Handle->length);

      if (status == USBH_OK)
      {
        HID_Handle->state =
            USBH_HID_SYNC;
      }

      else if (status == USBH_BUSY)
      {
        HID_Handle->state =
            USBH_HID_IDLE;

        status = USBH_OK;
      }

      else if (status == USBH_NOT_SUPPORTED)
      {
        HID_Handle->state =
            USBH_HID_SYNC;

        status = USBH_OK;
      }

      else
      {
        HID_Handle->state =
            USBH_HID_ERROR;

        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      USBH_OS_PutMessage(phost,
                         USBH_URB_EVENT,
                         0U,
                         0U);
#endif

      break;


    case USBH_HID_SYNC:

      if ((phost->Timer & 1U) != 0U)
      {
        HID_Handle->state =
            USBH_HID_GET_DATA;
      }

#if (USBH_USE_OS == 1U)
      USBH_OS_PutMessage(phost,
                         USBH_URB_EVENT,
                         0U,
                         0U);
#endif

      break;


    case USBH_HID_GET_DATA:

      (void)USBH_InterruptReceiveData(
          phost,
          HID_Handle->pData,
          (uint8_t)HID_Handle->length,
          HID_Handle->InPipe);

      HID_Handle->state =
          USBH_HID_POLL;

      HID_Handle->timer =
          phost->Timer;

      HID_Handle->DataReady =
          0U;

      break;


    case USBH_HID_POLL:

      if (USBH_LL_GetURBState(
              phost,
              HID_Handle->InPipe)
          == USBH_URB_DONE)
      {
        XferSize =
            USBH_LL_GetLastXferSize(
                phost,
                HID_Handle->InPipe);

        if ((HID_Handle->DataReady == 0U) &&
            (XferSize != 0U) &&
            (HID_Handle->fifo.buf != NULL))
        {
          (void)USBH_HID_FifoWrite(
              &HID_Handle->fifo,
              HID_Handle->pData,
              HID_Handle->length);

          HID_Handle->DataReady =
              1U;

          USBH_HID_EventCallback(phost);

#if (USBH_USE_OS == 1U)
          USBH_OS_PutMessage(
              phost,
              USBH_URB_EVENT,
              0U,
              0U);
#endif
        }
      }

      else
      {
        if (USBH_LL_GetURBState(
                phost,
                HID_Handle->InPipe)
            == USBH_URB_STALL)
        {
          if (USBH_ClrFeature(
                  phost,
                  HID_Handle->ep_addr)
              == USBH_OK)
          {
            HID_Handle->state =
                USBH_HID_GET_DATA;
          }
        }
      }

      break;


    default:

      break;
  }

  return status;
}


/**
  * @brief USBH_HID_SOFProcess
  */
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle =
      (HID_HandleTypeDef *)phost->pActiveClass->pData;

  uint8_t interface =
      phost->device.current_interface;


  /* P2000 Generic HID */
  if ((phost->device.CfgDesc.Itf_Desc[interface]
           .bInterfaceSubClass == 0x00U) &&
      (phost->device.CfgDesc.Itf_Desc[interface]
           .bInterfaceProtocol == 0x00U))
  {
    return USBH_OK;
  }


  if (HID_Handle->state == USBH_HID_POLL)
  {
    if ((phost->Timer -
         HID_Handle->timer)
        >= HID_Handle->poll)
    {
      HID_Handle->state =
          USBH_HID_GET_DATA;

#if (USBH_USE_OS == 1U)
      USBH_OS_PutMessage(
          phost,
          USBH_URB_EVENT,
          0U,
          0U);
#endif
    }
  }

  return USBH_OK;
}


/**
  * @brief Get HID Report Descriptor
  */
USBH_StatusTypeDef USBH_HID_GetHIDReportDescriptor(
    USBH_HandleTypeDef *phost,
    uint16_t length)
{
  USBH_StatusTypeDef status;

  if (length > sizeof(phost->device.Data))
  {
    USBH_ErrLog(
        "Control error: Get HID Report Descriptor failed, data buffer size issue");

    return USBH_NOT_SUPPORTED;
  }

  status =
      USBH_GetDescriptor(
          phost,
          USB_REQ_RECIPIENT_INTERFACE |
          USB_REQ_TYPE_STANDARD,
          USB_DESC_HID_REPORT,
          phost->device.Data,
          length);

  return status;
}


/**
  * @brief Get HID Descriptor
  */
USBH_StatusTypeDef USBH_HID_GetHIDDescriptor(
    USBH_HandleTypeDef *phost,
    uint16_t length)
{
  USBH_StatusTypeDef status;

  if (length > sizeof(phost->device.Data))
  {
    USBH_ErrLog(
        "Control error: Get HID Descriptor failed, data buffer size issue");

    return USBH_NOT_SUPPORTED;
  }

  status =
      USBH_GetDescriptor(
          phost,
          USB_REQ_RECIPIENT_INTERFACE |
          USB_REQ_TYPE_STANDARD,
          USB_DESC_HID,
          phost->device.Data,
          length);

  return status;
}


/**
  * @brief Set Idle
  */
USBH_StatusTypeDef USBH_HID_SetIdle(
    USBH_HandleTypeDef *phost,
    uint8_t duration,
    uint8_t reportId)
{
  phost->Control.setup.b.bmRequestType =
      USB_H2D |
      USB_REQ_RECIPIENT_INTERFACE |
      USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest =
      USB_HID_SET_IDLE;

  phost->Control.setup.b.wValue.w =
      (uint16_t)(((uint32_t)duration << 8U) |
                 (uint32_t)reportId);

  phost->Control.setup.b.wIndex.w =
      0U;

  phost->Control.setup.b.wLength.w =
      0U;

  return USBH_CtlReq(phost,
                     NULL,
                     0U);
}


/**
  * @brief Set Report
  *
  * Used by P2000 Feature Report.
  */
USBH_StatusTypeDef USBH_HID_SetReport(
    USBH_HandleTypeDef *phost,
    uint8_t reportType,
    uint8_t reportId,
    uint8_t *reportBuff,
    uint8_t reportLen)
{
  phost->Control.setup.b.bmRequestType =
      USB_H2D |
      USB_REQ_RECIPIENT_INTERFACE |
      USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest =
      USB_HID_SET_REPORT;

  phost->Control.setup.b.wValue.w =
      (uint16_t)(((uint32_t)reportType << 8U) |
                 (uint32_t)reportId);

  phost->Control.setup.b.wIndex.w =
      0U;

  phost->Control.setup.b.wLength.w =
      reportLen;

  return USBH_CtlReq(
      phost,
      reportBuff,
      (uint16_t)reportLen);
}


/**
  * @brief Get Report
  */
USBH_StatusTypeDef USBH_HID_GetReport(
    USBH_HandleTypeDef *phost,
    uint8_t reportType,
    uint8_t reportId,
    uint8_t *reportBuff,
    uint8_t reportLen)
{
  phost->Control.setup.b.bmRequestType =
      USB_D2H |
      USB_REQ_RECIPIENT_INTERFACE |
      USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest =
      USB_HID_GET_REPORT;

  phost->Control.setup.b.wValue.w =
      (uint16_t)(((uint32_t)reportType << 8U) |
                 (uint32_t)reportId);

  phost->Control.setup.b.wIndex.w =
      0U;

  phost->Control.setup.b.wLength.w =
      reportLen;

  return USBH_CtlReq(
      phost,
      reportBuff,
      (uint16_t)reportLen);
}


/**
  * @brief Set Protocol
  */
USBH_StatusTypeDef USBH_HID_SetProtocol(
    USBH_HandleTypeDef *phost,
    uint8_t protocol)
{
  phost->Control.setup.b.bmRequestType =
      USB_H2D |
      USB_REQ_RECIPIENT_INTERFACE |
      USB_REQ_TYPE_CLASS;

  phost->Control.setup.b.bRequest =
      USB_HID_SET_PROTOCOL;

  if (protocol != 0U)
  {
    phost->Control.setup.b.wValue.w =
        0U;
  }
  else
  {
    phost->Control.setup.b.wValue.w =
        1U;
  }

  phost->Control.setup.b.wIndex.w =
      0U;

  phost->Control.setup.b.wLength.w =
      0U;

  return USBH_CtlReq(
      phost,
      NULL,
      0U);
}


/**
  * @brief Parse HID descriptor
  */
static void USBH_HID_ParseHIDDesc(
    HID_DescTypeDef *desc,
    uint8_t *buf)
{
  USBH_DescHeader_t *pdesc =
      (USBH_DescHeader_t *)buf;

  uint16_t CfgDescLen;
  uint16_t ptr;

  CfgDescLen =
      LE16(buf + 2U);

  if (CfgDescLen > USB_CONFIGURATION_DESC_SIZE)
  {
    ptr = USB_LEN_CFG_DESC;

    while (ptr < CfgDescLen)
    {
      pdesc =
          USBH_GetNextDesc(
              (uint8_t *)pdesc,
              &ptr);

      if (pdesc->bDescriptorType ==
          USB_DESC_TYPE_HID)
      {
        desc->bLength =
            *((uint8_t *)pdesc + 0U);

        desc->bDescriptorType =
            *((uint8_t *)pdesc + 1U);

        desc->bcdHID =
            LE16((uint8_t *)pdesc + 2U);

        desc->bCountryCode =
            *((uint8_t *)pdesc + 4U);

        desc->bNumDescriptors =
            *((uint8_t *)pdesc + 5U);

        desc->bReportDescriptorType =
            *((uint8_t *)pdesc + 6U);

        desc->wItemLength =
            LE16((uint8_t *)pdesc + 7U);

        break;
      }
    }
  }
}


/**
  * @brief Get HID Device Type
  */
HID_TypeTypeDef USBH_HID_GetDeviceType(
    USBH_HandleTypeDef *phost)
{
  HID_TypeTypeDef type =
      HID_UNKNOWN;

  uint8_t InterfaceProtocol;

  if (phost->gState == HOST_CLASS)
  {
    InterfaceProtocol =
        phost->device.CfgDesc
            .Itf_Desc[phost->device.current_interface]
            .bInterfaceProtocol;

    if (InterfaceProtocol ==
        HID_KEYBRD_BOOT_CODE)
    {
      type = HID_KEYBOARD;
    }
    else if (InterfaceProtocol ==
             HID_MOUSE_BOOT_CODE)
    {
      type = HID_MOUSE;
    }
  }

  return type;
}


/**
  * @brief Get HID Poll Interval
  */
uint8_t USBH_HID_GetPollInterval(
    USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle =
      (HID_HandleTypeDef *)phost->pActiveClass->pData;

  if ((phost->gState == HOST_CLASS_REQUEST) ||
      (phost->gState == HOST_INPUT) ||
      (phost->gState == HOST_SET_CONFIGURATION) ||
      (phost->gState == HOST_CHECK_CLASS) ||
      (phost->gState == HOST_CLASS))
  {
    return (uint8_t)HID_Handle->poll;
  }

  return 0U;
}


/**
  * @brief FIFO Init
  */
void USBH_HID_FifoInit(
    FIFO_TypeDef *f,
    uint8_t *buf,
    uint16_t size)
{
  f->head = 0U;
  f->tail = 0U;
  f->lock = 0U;
  f->size = size;
  f->buf  = buf;
}


/**
  * @brief FIFO Read
  */
uint16_t USBH_HID_FifoRead(
    FIFO_TypeDef *f,
    void *buf,
    uint16_t nbytes)
{
  uint16_t i;
  uint8_t *p;

  p = (uint8_t *)buf;

  if (f->lock == 0U)
  {
    f->lock = 1U;

    for (i = 0U;
         i < nbytes;
         i++)
    {
      if (f->tail != f->head)
      {
        *p++ =
            f->buf[f->tail];

        f->tail++;

        if (f->tail ==
            f->size)
        {
          f->tail = 0U;
        }
      }
      else
      {
        f->lock = 0U;

        return i;
      }
    }
  }

  f->lock = 0U;

  return nbytes;
}


/**
  * @brief FIFO Write
  */
uint16_t USBH_HID_FifoWrite(
    FIFO_TypeDef *f,
    void *buf,
    uint16_t nbytes)
{
  uint16_t i;
  uint8_t *p;

  p = (uint8_t *)buf;

  if (f->lock == 0U)
  {
    f->lock = 1U;

    for (i = 0U;
         i < nbytes;
         i++)
    {
      if (((f->head + 1U) ==
           f->tail) ||
          (((f->head + 1U) ==
            f->size) &&
           (f->tail == 0U)))
      {
        f->lock = 0U;

        return i;
      }
      else
      {
        f->buf[f->head] =
            *p++;

        f->head++;

        if (f->head ==
            f->size)
        {
          f->head = 0U;
        }
      }
    }
  }

  f->lock = 0U;

  return nbytes;
}


/**
  * @brief HID Data Event Callback
  */
__weak void USBH_HID_EventCallback(
    USBH_HandleTypeDef *phost)
{
  UNUSED(phost);
}
