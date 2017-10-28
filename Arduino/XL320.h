/*
 Code based on:
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.

 Modified to work only with Dynamixel XL-320 actuator.

 Modifications made by Luis G III for HelloSpoon robot.
 Webpage: http://hellospoonrobot.com
 Twitter: @HelloSpoon
 Youtube: http://youtube.com/user/hellospoonrobot

 This file can be used and be modified by anyone,
 don't forget to say thank you to OP!

 */

#ifndef XL320_H_
#define XL320_H_

#include <inttypes.h>
#include <Stream.h>

class XL320 {
public:
	enum class Address : uint8_t
	{
	/*EEPROM Area*/
		MODEL_NUMBER             = 0, /**< Model number [R] (default=350) */
		VERSION                  = 2, /**< Information on the version of firmware [R] */
		ID                       = 3, /**< ID of Dynamixel [RW] (default=1 ; min=0 ; max=252) */
		BAUD_RATE                = 4, /**< Baud Rate of Dynamixel [RW] (default=3 ; min=0 ; max=3) 0: 9600, 1:57600, 2:115200, 3:1Mbps*/
		RETURN_DELAY_TIME        = 5, /**< Return Delay Time [RW] (default=250 ; min=0 ; max=254) */
		CW_ANGLE_LIMIT           = 6, /**< clockwise Angle Limit [RW] (default=0 ; min=0 ; max=1023) */
		CCW_ANGLE_LIMIT          = 8, /**< counterclockwise Angle Limit [RW] (default=1023 ; min=0 ; max=1023) */
		CONTROL_MODE             = 11, /**< Control Mode [RW] (default=2 ; min=1 ; max=2) */
		LIMIT_TEMPERATURE        = 12, /**< Internal Limit Temperature [RW] (default=65 ; min=0 ; max=150) */
		LOWER_LIMIT_VOLTAGE      = 13, /**< Lowest Limit Voltage [RW] (default=60 ; min=50 ; max=250) */
		UPPPER_LIMIT_VOLTAGE     = 14, /**< Upper Limit Voltage [RW] (default=90 ; min=50 ; max=250) */
		MAX_TORQUE               = 15, /**< Lowest byte of Max. Torque [RW] (default=1023 ; min=0 ; max=1023) */
		RETURN_LEVEL             = 17, /**< Return Level [RW] (default=2 ; min=0 ; max=2) */
		ALARM_SHUTDOWN           = 18, /**< Shutdown for Alarm [RW] (default=3 ; min=0 ; max=7) */
	/*RAM Area*/
		TORQUE_ENABLE            = 24, /**< Torque On/Off [RW] (default=0 ; min=0 ; max=1) */
		LED                      = 25, /**< LED On/Off [RW] (default=0 ; min=0 ; max=7) */
		D_GAIN    				 = 27, /**< D Gain [RW] (default=0 ; min=0 ; max=254) */
		I_GAIN      			 = 28, /**< I Gain [RW] (default=0 ; min=0 ; max=254) */
		P_GAIN    				 = 29, /**< P Gain [RW] (default=32 ; min=0 ; max=254) */
		GOAL_POSITION            = 30, /**< Goal Position [RW] (min=0 ; max=1023) */
		GOAL_SPEED               = 32, /**< Goal Speed [RW] (min=0 ; max=2047) */
		GOAL_TORQUE 		     = 35, /**< Goal Torque [RW] (min=0 ; max=1023) */
		PRESENT_POSITION         = 37, /**< Current Position [R] */
		PRESENT_SPEED            = 39, /**< Current Speed [R] */
		PRESENT_LOAD             = 41, /**< Current Load [R] */
		PRESENT_VOLTAGE          = 45, /**< Current Voltage [R] */
		PRESENT_TEMPERATURE      = 46, /**< Present temperature [R] */
		REGISTERED_INSTRUCTION   = 47, /**< Registered Instruction [R] (default=0) */
		MOVING                   = 49, /**< Moving [R] (default=0) */
		HARDWARE_ERROR           = 50, /**< Hardware error status [R] (default=0) */
		PUNCH                    = 51  /**< Punch [RW] (default=32 ; min=0 ; max=1023) */
	};

	void Begin(Stream &stream);

	void QuickTest();

	int Write(int id, Address Address, int value);

	int GetValue(int id, Address address);

	void DebugAllValue(int id, Stream &stream);

	void Flush();

private:
	enum DynamixelIntruction {
		DXL_INST_PING          = 0x01, /**< checks if ID is associated to a Device */
		DXL_INST_READ          = 0x02, /**< read data from the Device */
		DXL_INST_WRITE         = 0x03, /**< write data on the Device */
		DXL_INST_REG_WRITE     = 0x04, /**< registers the write instruction to a standby status */
		DXL_INST_ACTION        = 0x05, /**< executes the write instruction previously registered */
		DXL_INST_FACTORY_RESET = 0x06, /**< resets the Control Table to its initial factory default settings */
		DXL_INST_REBOOT        = 0x08, /**< reboot the Device */
		DXL_INST_STATUS        = 0x55, /**< Return Instruction for the Instruction Packet */
		DXL_INST_SYNC_READ     = 0x82, /**< (Multiple devices) read data with same Address and length at once */
		DXL_INST_SYNC_WRITE    = 0x83, /**< (Multiple devices) write data on the same Address and length at once */
		DXL_INST_BULK_READ     = 0x92, /**< (Multiple devices) read data from different Addresses and lengths at once */
		DXL_INST_BULK_WRITE    = 0x93, /**< (Multiple devices) write data on different Addresses and lengths at once */
	};

	int SendReadPacket(int id, Address Address, uint16_t requestedSize);

	int ReadPacket(uint8_t *buffer, int size);

	static uint8_t GetAddressSize(Address address);
	void NDelay(uint32_t nTime);
	static uint16_t UpdateCrc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

	Stream *m_Stream;

	class Packet {
		bool m_FreeData;
	public:
		uint8_t *data;
		size_t dataSize;

		// wrap a received data stream in an Packet object for analysis
		Packet(uint8_t *data, size_t size);
		// build a packet into the pre-allocated data array
		// if data is null it will be malloc'ed and free'd on destruction.
		Packet(uint8_t *data, size_t  size, uint8_t id, DynamixelIntruction instruction, Address address, int paramSize, uint8_t *param);
		~Packet();
		uint8_t getId();
		int getLength();
		int getSize();
		int getParameterCount();
		uint8_t getInstruction();
		uint8_t* getParameters();
		bool isValid();

		void debugToStream(Stream &stream);

	};
};

#endif
