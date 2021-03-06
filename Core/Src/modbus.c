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

void modbus_lib_end_of_telegram(volatile ModbusConfig *config) {
	const uint16_t length = config->length;

	uint8_t buffer[length];
	memcpy(&buffer[0], &config->RxBuffer[config->start], length);

	// Check address
	uint16_t address = buffer[0];
	if (config->slaveId != address) {
		return;
	}

	const uint16_t actualCRC = (((uint16_t) buffer[length - 2]) << 8)
			| buffer[length - 1];

	const uint16_t computedCRC = usMBCRC16(&buffer[0], length - 2);

	if (computedCRC != actualCRC) {
		//Oops, crc didn't match
		modbus_lib_send_error(config, MBUS_RESPONSE_NONE);
		return;
	}

	switch (buffer[1]) {
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
		uint16_t address = (buffer[2] << 8) | (buffer[3]);
		uint16_t value = (buffer[4] << 8) | (buffer[5]);

		uint8_t isSuccess = modbus_lib_write_handler(
				address + MB_ADDRESS_HOLDING_REGISTER_OFFSET, value);

		if (isSuccess) {
			//Return the response to indicate success
			modbus_lib_transport_write(buffer, 0, length);
		}
		break;
	}
	default:
		return;
	}
}

uint16_t modbus_lib_send_error(volatile ModbusConfig *config, int error_code) {
	if (error_code != MBUS_RESPONSE_NONE) {
		uint8_t res[MB_EXCEPTION_LENGTH] = { config->slaveId,
				config->RxBuffer[config->start + 1] | 0x80, error_code };
		uint16_t crc = 0; //TODOCRC16_2(config->RxBuffer[config->start], config->length);
		res[MB_EXCEPTION_LENGTH - 2] = (crc >> 8) & 0xff;
		res[MB_EXCEPTION_LENGTH - 1] = (crc & 0xff);

		modbus_lib_transport_write(&res[0], 0, MB_EXCEPTION_LENGTH);
	}
	return -1;
}
