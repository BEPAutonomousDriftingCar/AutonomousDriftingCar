/*

 Code based on:
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.

 Modified to work only with Dynamixel XL-320 actuator.

 Modifications made by Luis G III for XL320 robot.
 Webpage: http://hellospoonrobot.com
 Twitter: @XL320
 Youtube: http://youtube.com/user/hellospoonrobot

 This file can be used and be modified by anyone,
 don't forget to say thank you to OP!

 */

#include "Arduino.h"
#include "XL320.h"


#define SERVO_TIME_DELAY 12000

	///////////////// utility for value ///////////////////////////
#define DXL_LOBYTE(w)           ((uint8_t)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((uint8_t)((((unsigned long)(w)) >> 8) & 0xff))

void enableTX()
{
	uint8_t c;
	c = UART2_C3;
	c |= UART_C3_TXDIR;
	UART2_C3 = c;
}

void enableRX()
{
	uint8_t c;
	c = UART2_C3;
	c &= ~UART_C3_TXDIR;
	UART2_C3 = c;
}

void XL320::Begin(Stream &stream)
{
	this->m_Stream = &stream;
}


void XL320::QuickTest() {

}

int XL320::Write(int id, Address address, int value) {

	/*Dynamixel 2.0 communication protocol
	used by Dynamixel XL-320 and Dynamixel PRO only.
	*/

	// technically i think we need 14bytes for this packet 

	const int bufsize = 16;
	uint8_t txbuffer[bufsize] = { 0 };
	uint8_t param[2];
	int paramSize = XL320::GetAddressSize(address);
	if (paramSize == 1)
	{
		param[0] = value;
	}
	else // addresse size == 2
	{
		param[0] = DXL_LOBYTE(value);
		param[1] = DXL_HIBYTE(value);
	}

	Packet p(txbuffer, bufsize, id, DXL_INST_WRITE, address, paramSize, param);

	//debug
	//Serial.println();
	//for (int i = 0; i < bufsize; i++)
	//{
	//	Serial.print((int)p.data[i], HEX);
	//	Serial.print(',');
	//}
	//Serial.println();

	enableTX();
	m_Stream->write(txbuffer, p.getSize());

	Flush();
	
	NDelay(SERVO_TIME_DELAY);

	return bufsize;
}

int XL320::SendReadPacket(int id, Address address, uint16_t requestedSize) {

	/*Dynamixel 2.0 communication protocol
	used by Dynamixel XL-320 and Dynamixel PRO only.
	*/

	const int bufsize = 16;

	byte txbuffer[bufsize];

	Packet p(txbuffer, bufsize, id, DXL_INST_READ, address, 2, (uint8_t*)&requestedSize);

	enableTX();
	m_Stream->write(txbuffer, p.getSize());

	Flush();

	return p.getSize();
}

int XL320::GetValue(int id, Address address)
{
	delay(10);
	while (this->m_Stream->available() > 0) {
		this->m_Stream->read();
	}

	int size = XL320::GetAddressSize(address);
	SendReadPacket(id, address, size);

	enableRX();

	uint8_t buffer[255] = { 0 };
	if (this->ReadPacket(buffer, 255) > 0) 
	{
		Packet p(buffer, 255);
		
		//p.debugToStream(Serial);

		if (!p.isValid())
			return -1;

		if (size == 1)
		{
			return *p.getParameters();
		}
		else// size == 2
		{
			return *((uint16_t*)p.getParameters());
		}
	}
	return -2;
}

void XL320::DebugAllValue(int id, Stream & stream)
{
	stream.printf("MODEL_NUMBER: %d\r\n", GetValue(id, Address::MODEL_NUMBER));
	stream.printf("VERSION: %d\r\n", GetValue(id, Address::VERSION));
	stream.printf("ID: %d\r\n", GetValue(id, Address::ID));
	stream.printf("BAUD_RATE: %d\r\n", GetValue(id, Address::BAUD_RATE));
	stream.printf("RETURN_DELAY_TIME: %d\r\n", GetValue(id, Address::RETURN_DELAY_TIME));
	stream.printf("CW_ANGLE_LIMIT: %d\r\n", GetValue(id, Address::CW_ANGLE_LIMIT));
	stream.printf("CCW_ANGLE_LIMIT: %d\r\n", GetValue(id, Address::CCW_ANGLE_LIMIT));
	stream.printf("CONTROL_MODE: %d\r\n", GetValue(id, Address::CONTROL_MODE));
	stream.printf("LIMIT_TEMPERATURE: %d\r\n", GetValue(id, Address::LIMIT_TEMPERATURE));
	stream.printf("LOWER_LIMIT_VOLTAGE: %d\r\n", GetValue(id, Address::LOWER_LIMIT_VOLTAGE));
	stream.printf("UPPPER_LIMIT_VOLTAGE: %d\r\n", GetValue(id, Address::UPPPER_LIMIT_VOLTAGE));
	stream.printf("MAX_TORQUE: %d\r\n", GetValue(id, Address::MAX_TORQUE));
	stream.printf("RETURN_LEVEL: %d\r\n", GetValue(id, Address::RETURN_LEVEL));
	stream.printf("ALARM_SHUTDOWN: %d\r\n", GetValue(id, Address::ALARM_SHUTDOWN));
	stream.printf("TORQUE_ENABLE: %d\r\n", GetValue(id, Address::TORQUE_ENABLE));
	stream.printf("LED: %d\r\n", GetValue(id, Address::LED));
	stream.printf("D_GAIN: %d\r\n", GetValue(id, Address::D_GAIN));
	stream.printf("I_GAIN: %d\r\n", GetValue(id, Address::I_GAIN));
	stream.printf("P_GAIN: %d\r\n", GetValue(id, Address::P_GAIN));
	stream.printf("GOAL_POSITION: %d\r\n", GetValue(id, Address::GOAL_POSITION));
	stream.printf("GOAL_SPEED: %d\r\n", GetValue(id, Address::GOAL_SPEED));
	stream.printf("GOAL_TORQUE: %d\r\n", GetValue(id, Address::GOAL_TORQUE));
	stream.printf("PRESENT_POSITION: %d\r\n", GetValue(id, Address::PRESENT_POSITION));
	stream.printf("PRESENT_SPEED: %d\r\n", GetValue(id, Address::PRESENT_SPEED));
	stream.printf("PRESENT_LOAD: %d\r\n", GetValue(id, Address::PRESENT_LOAD));
	stream.printf("PRESENT_VOLTAGE: %d\r\n", GetValue(id, Address::PRESENT_VOLTAGE));
	stream.printf("PRESENT_TEMPERATURE: %d\r\n", GetValue(id, Address::PRESENT_TEMPERATURE));
	stream.printf("REGISTERED_INSTRUCTION: %d\r\n", GetValue(id, Address::REGISTERED_INSTRUCTION));
	stream.printf("MOVING: %d\r\n", GetValue(id, Address::MOVING));
	stream.printf("HARDWARE_ERROR: %d\r\n", GetValue(id, Address::HARDWARE_ERROR));
	stream.printf("PUNCH: %d\r\n", GetValue(id, Address::PUNCH));

}

uint8_t XL320::GetAddressSize(Address address)
{
	switch (address)
	{
	case Address::MODEL_NUMBER:
	case Address::CW_ANGLE_LIMIT:
	case Address::CCW_ANGLE_LIMIT:
	case Address::MAX_TORQUE:
	case Address::GOAL_POSITION:
	case Address::GOAL_SPEED:
	case Address::GOAL_TORQUE:
	case Address::PRESENT_POSITION:
	case Address::PRESENT_SPEED:
	case Address::PRESENT_LOAD:
	case Address::PUNCH:
		return 2;
	default:
		return 1;
	}
}

void XL320::NDelay(uint32_t nTime) {
	/*
	volatile uint32_t max;
	for( max=0; max < nTime; max++){

	}
	*/
}

uint16_t XL320::UpdateCrc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
	static const uint16_t crc_table[256] = { 0x0000,
		0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
		0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
		0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
		0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
		0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
		0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
		0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
		0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
		0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
		0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
		0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
		0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
		0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
		0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
		0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
		0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
		0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
		0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
		0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
		0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
		0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
		0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
		0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
		0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
		0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
		0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
		0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
		0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
		0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
		0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
		0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
		0x820D, 0x8207, 0x0202 };

	for (uint16_t j = 0; j < data_blk_size; j++)
	{
		uint16_t i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

void XL320::Flush() {
	this->m_Stream->flush();
}

// from http://stackoverflow.com/a/133363/195061

#define FSM
#define STATE(x)        s_##x : if(!m_Stream->readBytes(&BUFFER[I++],1)) goto sx_timeout ; if(I>=SIZE) goto sx_overflow; sn_##x :
#define THISBYTE        (BUFFER[I-1])
#define NEXTSTATE(x)    goto s_##x
#define NEXTSTATE_NR(x) goto sn_##x
#define LOVERFLOW       sx_overflow :
#define TIMEOUT         sx_timeout :

int XL320::ReadPacket(uint8_t *BUFFER, int SIZE) {
	int I = 0;

	int length = 0;

	// state names normally name the last parsed symbol


	FSM{
	  STATE(start) {
	if (THISBYTE == 0xFF) NEXTSTATE(header_ff_1);
	I = 0; NEXTSTATE(start);
	  }
	  STATE(header_ff_1) {
	if (THISBYTE == 0xFF) NEXTSTATE(header_ff_2);
	I = 0; NEXTSTATE(start);
	  }
	  STATE(header_ff_2) {
	if (THISBYTE == 0xFD) NEXTSTATE(header_fd);
	// yet more 0xFF's? stay in this state
	if (THISBYTE == 0xFF) NEXTSTATE(header_ff_2);
	// anything else? restart
	I = 0; NEXTSTATE(start);
	  }
	  STATE(header_fd) {
		  // reading reserved, could be anything in theory, normally 0
		  }
		  STATE(header_reserved) {
			  // id = THISBYTE
			  }
			  STATE(id) {
			length = THISBYTE;
			  }
			  STATE(length_1) {
			length += THISBYTE << 8; // eg: length=4
			  }
			  STATE(length_2) {
			  }
			  STATE(instr) {
				  // instr = THISBYTE
					  // check length because
					  // action and reboot commands have no parameters
				  if (I - length >= 5) NEXTSTATE(checksum_1);
					}
					STATE(params) {
						// check length and maybe skip to checksum
						if (I - length >= 5) NEXTSTATE(checksum_1);
						// or keep reading params
						NEXTSTATE(params);
						}
						STATE(checksum_1) {
						}
						STATE(checksum_2) {
							// done
							return I;
							}
							LOVERFLOW {
								return -1;
							}
							TIMEOUT {
							return -2;
							}

	}
}


XL320::Packet::Packet(
	uint8_t *data,
	size_t data_size,
	uint8_t id,
	DynamixelIntruction instruction,
	Address address,
	int paramSize, 
	uint8_t *param) {


	// [ff][ff][fd][00][id][len1][len2] { [instr][params(parameter_data_size)][crc1][crc2] }
	unsigned int length = 5 + paramSize;
	if (!data) {
		// [ff][ff][fd][00][id][len1][len2] { [data(length)] }
		this->dataSize = 7 + length;
		this->data = (uint8_t*)malloc(dataSize);
		this->m_FreeData = true;
	}
	else {
		this->data = data;
		this->dataSize = data_size;
		this->m_FreeData = false;
	}
	this->data[0] = 0xFF;
	this->data[1] = 0xFF;
	this->data[2] = 0xFD;
	this->data[3] = 0x00;
	this->data[4] = id;
	this->data[5] = length & 0xff;
	this->data[6] = (length >> 8) & 0xff;
	this->data[7] = instruction;
	this->data[8] = DXL_LOBYTE(address);
	this->data[9] = DXL_HIBYTE(address);
	memcpy(&this->data[10], param, paramSize);
	unsigned short crc = UpdateCrc(0, this->data, this->getSize() - 2);
	this->data[length + 5] = crc & 0xff;
	this->data[length + 6] = (crc >> 8) & 0xff;
}

XL320::Packet::Packet(uint8_t *data, size_t size) {
	this->data = data;
	this->dataSize = size;
	this->m_FreeData = false;
}


XL320::Packet::~Packet() {
	if (this->m_FreeData == true) {
		free(this->data);
	}
}

void XL320::Packet::debugToStream(Stream &stream) {
	stream.println("raw:");
	for (int i = 0; i < 10 + this->getLength(); i++)
	{
		stream.print((int)data[i], HEX);
		stream.print(',');
	}
	stream.print("\r\nid: ");
	stream.println(this->getId(), DEC);
	stream.print("length: ");
	stream.println(this->getLength(), DEC);
	stream.print("instruction: ");
	stream.println(this->getInstruction(), HEX);
	stream.print("parameter count: ");
	stream.println(this->getParameterCount(), DEC);
	for (int i = 0; i < this->getParameterCount(); i++) {
		stream.print((int)this->getParameters()[i], HEX);
		if (i < this->getParameterCount() - 1) {
			stream.print(',');
		}
	}
	stream.println();
	stream.print("valid: ");
	stream.println(this->isValid() ? "yes" : "no");
}

uint8_t XL320::Packet::getId() {
	return data[4];
}

int XL320::Packet::getLength() {
	return data[5] + ((data[6] & 0xff) << 8);
}

int XL320::Packet::getSize() {
	return getLength() + 7;
}

int XL320::Packet::getParameterCount() {
	return getLength() - 4;
}

uint8_t XL320::Packet::getInstruction() {
	return data[7];
}

uint8_t* XL320::Packet::getParameters() {
	return &data[9];
}

bool XL320::Packet::isValid() {
	int length = getLength();
	unsigned short storedChecksum = data[length + 5] + (data[length + 6] << 8);
	return storedChecksum == UpdateCrc(0, data, length + 5);
}
