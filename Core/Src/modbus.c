/*******************************************************
 * 
 * Implementation reference: 
 * 
 *      http://www.mayor.de/lian98/doc.en/html/u_mbusser-rtu_struct.htm
 *      https://www.modbustools.com/modbus.html
 * 
 * Example telegrams: 
 *      Read holding registers: 010300000002c40b (read 2 registers starting from 40001)
 *      Read holding registers: 0103000000044409 (read 4 registers starting from 40001)
 * 
 *      Write single register: 01060000007bc9e9 (write "123" to register 40001)
 * 
 *      Write multiple registers: 01100000000204007b0237c300
 * 
 *******************************************************/

#include "modbus.h"

void modbus_lib_end_of_telegram(ModbusConfig *config, uint16_t start,
		uint16_t length) {

	// Check CRC
	/*  CRC_t expected = usMBCRC16(g_modbus_lib_received_telegram, g_modbus_lib_received_length-2);
	 UCHAR got_low = g_modbus_lib_received_telegram[g_modbus_lib_received_length-2];
	 UCHAR got_high = g_modbus_lib_received_telegram[g_modbus_lib_received_length-1];
	 if ((expected.bytes.low != got_low) || (expected.bytes.high != got_high)){
	 modbus_lib_send_error(MBUS_RESPONSE_NONE);
	 return;
	 }*/

	// Check address
	if (config->slaveId != config->RxBuffer[start + 0]) {
		//modbus_lib_send_error(config, MBUS_RESPONSE_NONE);
		return;
	}

	// Telegram is okay, call the relevant handler
	// -------------------------------------------
	//uint8_t outgoing_telegram[MODBUS_LIB_MAX_BUFFER];
	//uint16_t oindex = 0;

	//outgoing_telegram[oindex++] = config->slaveId;

	//volatile MbDataField start_addr, count, res, addr, value;

	switch (config->RxBuffer[start + 1]) {
	case MB_FUNC_READ_HOLDING_REGISTERS:
		/*  start_addr.bytes.high = g_modbus_lib_received_telegram[2];
		 start_addr.bytes.low = g_modbus_lib_received_telegram[3];
		 count.bytes.high = g_modbus_lib_received_telegram[4];
		 count.bytes.low = g_modbus_lib_received_telegram[5];

		 outgoing_telegram[oindex++] = g_modbus_lib_received_telegram[1];    // function code
		 outgoing_telegram[oindex++] = count.value * 2;                      // byte count

		 for (uint16_t i=0; i < count.value; i++){
		 res.value = modbus_lib_read_handler(start_addr.value + i + MB_ADDRESS_HOLDING_REGISTER_OFFSET);
		 if (g_modbus_lib_exception_occurred){
		 g_modbus_lib_exception_occurred = 0;
		 return;
		 }
		 outgoing_telegram[oindex++] = res.bytes.high;
		 outgoing_telegram[oindex++] = res.bytes.low;
		 }

		 CRC_t crc = usMBCRC16(outgoing_telegram, oindex);
		 outgoing_telegram[oindex++] = crc.bytes.low;
		 outgoing_telegram[oindex++] = crc.bytes.high;

		 modbus_lib_transport_write(outgoing_telegram, oindex);*/
		break;
	case MB_FUNC_WRITE_REGISTER: {
		uint16_t address = (config->RxBuffer[start + 2] << 8)
				| (config->RxBuffer[start + 3]);
		uint16_t value = (config->RxBuffer[start + 4] << 8)
				| (config->RxBuffer[start + 5]);

		modbus_lib_write_handler(address + MB_ADDRESS_HOLDING_REGISTER_OFFSET,
				value);
		break;
	}
	default:
		return;
	}
}