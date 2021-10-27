
/********************************************************************************************
 * SNMP : User Customization Part
 *  - OID Registration
 *  - User defined functions for OID related
 *  	+ Integer value, String
 *  	+ I/O control / Chip registers
 *  	+ Network Informations
 *  	+ Etc.
 *
 *********************************************************************************************/
#include "stm32f10x.h"                  // Device header
#include "snmp_custom.h"

extern uint8_t gps1_stt;
extern uint8_t gps2_stt;
extern uint8_t power1_stt;
extern uint8_t power2_stt;

void get_GPS_stt(void *ptr, uint8_t *len);
void get_POWER_stt(void *ptr, uint8_t *len);
	
dataEntryType snmpData[] =
{
    // System MIB
	// SysDescr Entry
	{8, {0x2b, 6, 1, 2, 1, 1, 1, 0},
	SNMPDTYPE_OCTET_STRING, 30, {"GPS clock time server"},
	NULL, NULL},

	// SysObjectID Entry
	{8, {0x2b, 6, 1, 2, 1, 1, 2, 0},
	SNMPDTYPE_OBJ_ID, 8, {"\x2b\x06\x01\x02\x01\x01\x02\x00"},
	NULL, NULL},

	// SysUptime Entry
	{8, {0x2b, 6, 1, 2, 1, 1, 3, 0},
	SNMPDTYPE_TIME_TICKS, 0, {""},
	currentUptime, NULL},

	// sysContact Entry
	{8, {0x2b, 6, 1, 2, 1, 1, 4, 0},
	SNMPDTYPE_OCTET_STRING, 30, {"https://attech.com.vn/lien-he-voi-chung-toi/"},
	NULL, NULL},

	// sysName Entry
	{8, {0x2b, 6, 1, 2, 1, 1, 5, 0},
	SNMPDTYPE_OCTET_STRING, 30, {"https://attech.com.vn/"},
	NULL, NULL},

	// Location Entry
	{8, {0x2b, 6, 1, 2, 1, 1, 6, 0},
	SNMPDTYPE_OCTET_STRING, 30, {"ATTECH"},
	NULL, NULL},

	// SysServices
	{8, {0x2b, 6, 1, 2, 1, 1, 7, 0},
	SNMPDTYPE_INTEGER, 4, {""},
	NULL, NULL},
	{8, {0x2b, 6, 1, 4, 1, 6, 1, 0},
	SNMPDTYPE_OCTET_STRING, 40, {""},
	get_GPS_stt, NULL},
	{8, {0x2b, 6, 1, 4, 1, 6, 1, 1},
	SNMPDTYPE_OCTET_STRING, 40, {""},
	get_POWER_stt, NULL},
};

const int32_t maxData = (sizeof(snmpData) / sizeof(dataEntryType));

void initTable()
{
	// Example integer value for [OID 1.3.6.1.2.1.1.7.0]
	snmpData[6].u.intval = -15;
}

void get_GPS_stt(void *ptr, uint8_t *len)
{
	*len = sprintf((char *)ptr, "GPS1: [%s]; GPS2: [%s]", gps1_stt?"ON":"OFF", gps2_stt?"ON":"OFF");
}

void get_POWER_stt(void *ptr, uint8_t *len)
{
	*len = sprintf((char *)ptr, "POWER1: [%s]; POWER2: [%s]", power1_stt?"ON":"OFF", power2_stt?"ON":"OFF");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void initial_Trap(uint8_t * managerIP, uint8_t * agentIP)
{
	// SNMP Trap: WarmStart(1) Trap
	{
		dataEntryType enterprise_oid = {0x0a, {0x2b, 0x06, 0x01, 0x04, 0x01, 0x81, 0x9b, 0x19, 0x01, 0x00},
												SNMPDTYPE_OBJ_ID, 0x0a, {"\x2b\x06\x01\x04\x01\x81\x9b\x19\x10\x00"},	NULL, NULL};
		// Generic Trap: warmStart COMMUNITY
		snmp_sendTrap(managerIP, agentIP, (void *)COMMUNITY, enterprise_oid, SNMPTRAP_WARMSTART, 0, 0);
	}

}
