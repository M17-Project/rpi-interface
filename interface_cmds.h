/*
 * interface_cmds.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Wojciech Kaczmarski, SP5WWP
 *              M17 Project
 */
#pragma once

//TODO: sync this with CARI
enum cmd_t
{
	CMD_PING,

	//SET
	CMD_SET_RX_FREQ,
	CMD_SET_TX_FREQ,
	CMD_SET_TX_POWER,
	CMD_SET_RESERVED,
	CMD_SET_FREQ_CORR,
	CMD_SET_AFC,
	CMD_SET_TX_START,
	CMD_SET_RX,

	//GET
	CMD_GET_IDENT = 0x80,
	CMD_GET_CAPS,
	CMD_GET_RX_FREQ,
	CMD_GET_TX_FREQ,
	CMD_GET_TX_POWER,
	CMD_GET_FREQ_CORR
};
