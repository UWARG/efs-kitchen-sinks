

#pragma once
#include <com.hobbywing.esc.GetMaintenanceInformation_req.h>
#include <com.hobbywing.esc.GetMaintenanceInformation_res.h>

#define COM_HOBBYWING_ESC_GETMAINTENANCEINFORMATION_ID 241
#define COM_HOBBYWING_ESC_GETMAINTENANCEINFORMATION_SIGNATURE (0xB81DBD4EC9A5977DULL)


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
#include <canard/cxx_wrappers.h>
SERVICE_MESSAGE_CXX_IFACE(com_hobbywing_esc_GetMaintenanceInformation, COM_HOBBYWING_ESC_GETMAINTENANCEINFORMATION_ID, COM_HOBBYWING_ESC_GETMAINTENANCEINFORMATION_SIGNATURE, COM_HOBBYWING_ESC_GETMAINTENANCEINFORMATION_REQUEST_MAX_SIZE, COM_HOBBYWING_ESC_GETMAINTENANCEINFORMATION_RESPONSE_MAX_SIZE);
#endif
