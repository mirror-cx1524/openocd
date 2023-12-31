// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
	File : jtag.c															*
	Contents : Jtag handling functions code for NanoXplore					*
	USB-JTAG ANGIE adapter hardware.										*
	Based on openULINK project code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#include "jtag.h"
#include "io.h"
#include "msgtypes.h"
#include "reg_ezusb.h"
#include <stdbool.h>
#include <serial.h>
#include <stdio.h>

/** Delay value for SCAN_IN operations with less than maximum TCK frequency */
uint8_t delay_scan_in;

/** Delay value for SCAN_OUT operations with less than maximum TCK frequency */
uint8_t delay_scan_out;

/** Delay value for SCAN_IO operations with less than maximum TCK frequency */
uint8_t delay_scan_io;

/** Delay value for CLOCK_TCK operations with less than maximum frequency */
uint8_t delay_tck;

/** Delay value for CLOCK_TMS operations with less than maximum frequency */
uint8_t delay_tms;

/**
 * Perform JTAG SCAN-IN operation at maximum TCK frequency.
 *
 * Dummy data is shifted into the JTAG chain via TDI, TDO data is sampled and
 * stored in the EP2 IN buffer.
 *
 * Maximum achievable TCK frequency is 182 kHz for ANGIE clocked at 24 MHz.
 *
 * @param out_offset offset in EP1OUTBUF where payload data starts
 * @param in_offset
 */
void jtag_scan_in(uint8_t out_offset, uint8_t in_offset)
{
	uint8_t scan_size_bytes, bits_last_byte;
	uint8_t tms_count_start, tms_count_end;
	uint8_t tms_sequence_start, tms_sequence_end;
	uint8_t tdo_data, i, j;

	uint8_t outb_buffer;

	/* Get parameters from EP1OUTBUF */
	scan_size_bytes = EP1OUTBUF[out_offset];
	bits_last_byte = EP1OUTBUF[out_offset + 1];
	tms_count_start = (EP1OUTBUF[out_offset + 2] >> 4) & 0x0F;
	tms_count_end = EP1OUTBUF[out_offset + 2] & 0x0F;
	tms_sequence_start = EP1OUTBUF[out_offset + 3];
	tms_sequence_end = EP1OUTBUF[out_offset + 4];

	if (tms_count_start > 0)
		jtag_clock_tms(tms_count_start, tms_sequence_start);

	outb_buffer = IOB & ~(bmbit1 | bmbit2 | bmbit3);

	/* Shift all bytes except the last byte */
	for (i = 0; i < scan_size_bytes - 1; i++) {
		tdo_data = 0;

		for (j = 0; j < 8; j++) {
			IOB = outb_buffer;	/* TCK changes here */
			tdo_data = tdo_data >> 1;
			IOB = (outb_buffer | bmbit2);

			if (PIN_TDO)
				tdo_data |= 0x80;
		}

		/* Copy TDO data to EP1INBUF */
		EP1INBUF[i + in_offset] = tdo_data;
	}
	tdo_data = 0;

	/* Shift the last byte */
	for (j = 0; j < bits_last_byte; j++) {
		/* Assert TMS signal if requested and this is the last bit */
		if (j == (bits_last_byte - 1) && tms_count_end > 0) {
			outb_buffer |= bmbit1;
			tms_count_end--;
			tms_sequence_end = tms_sequence_end >> 1;
		}

		IOB = outb_buffer;	/* TCK changes here */
		tdo_data = tdo_data >> 1;
		IOB = (outb_buffer | bmbit2);

		if (PIN_TDO)
			tdo_data |= 0x80;
	}
	tdo_data = tdo_data >> (8 - bits_last_byte);

	/* Copy TDO data to EP1INBUF */
	EP1INBUF[i + in_offset] = tdo_data;

	/* Move to correct end state */
	if (tms_count_end > 0)
		jtag_clock_tms(tms_count_end, tms_sequence_end);
}


/**
 * Perform JTAG SCAN-IN operation at variable TCK frequency.
 *
 * Dummy data is shifted into the JTAG chain via TDI, TDO data is sampled and
 * stored in the EP2 IN buffer.
 *
 * Maximum achievable TCK frequency is 113 kHz for ANGIE clocked at 24 MHz.
 *
 * @param out_offset offset in EP1OUTBUF where payload data starts
 * @param in_offset
 */
void jtag_slow_scan_in(uint8_t out_offset, uint8_t in_offset)
{
	uint8_t scan_size_bytes, bits_last_byte;
	uint8_t tms_count_start, tms_count_end;
	uint8_t tms_sequence_start, tms_sequence_end;
	uint8_t tdo_data, i, j, k;
	uint8_t outb_buffer;

	/* Get parameters from EP1OUTBUF */
	scan_size_bytes = EP1OUTBUF[out_offset];
	bits_last_byte = EP1OUTBUF[out_offset + 1];
	tms_count_start = (EP1OUTBUF[out_offset + 2] >> 4) & 0x0F;
	tms_count_end = EP1OUTBUF[out_offset + 2] & 0x0F;
	tms_sequence_start = EP1OUTBUF[out_offset + 3];
	tms_sequence_end = EP1OUTBUF[out_offset + 4];

	if (tms_count_start > 0)
		jtag_slow_clock_tms(tms_count_start, tms_sequence_start);

	outb_buffer = IOB & ~(bmbit3 | bmbit2 | bmbit1);

	/* Shift all bytes except the last byte */
	for (i = 0; i < scan_size_bytes - 1; i++) {
		tdo_data = 0;

		for (j = 0; j < 8; j++) {
			IOB = outb_buffer;	/* TCK changes here */
			for (k = 0; k < delay_scan_in; k++)
				;
			tdo_data = tdo_data >> 1;

			IOB = (outb_buffer | bmbit2);
			for (k = 0; k < delay_scan_in; k++)
				;

			if (PIN_TDO)
				tdo_data |= 0x80;
		}

		/* Copy TDO data to EP1INBUF */
		EP1INBUF[i + in_offset] = tdo_data;
	}

	tdo_data = 0;

	/* Shift the last byte */
	for (j = 0; j < bits_last_byte; j++) {
		/* Assert TMS signal if requested and this is the last bit */
		if (j == (bits_last_byte - 1) && tms_count_end > 0) {
			outb_buffer |= bmbit1;
			tms_count_end--;
			tms_sequence_end = tms_sequence_end >> 1;
		}

		IOB = outb_buffer;	/* TCK changes here */
		for (k = 0; k < delay_scan_in; k++)
			;
		tdo_data = tdo_data >> 1;

		IOB = (outb_buffer | bmbit2);
		for (k = 0; k < delay_scan_in; k++)
			;

		if (PIN_TDO)
			tdo_data |= 0x80;
	}
	tdo_data = tdo_data >> (8 - bits_last_byte);

	/* Copy TDO data to EP1INBUF */
	EP1INBUF[i + in_offset] = tdo_data;

	/* Move to correct end state */
	if (tms_count_end > 0)
		jtag_slow_clock_tms(tms_count_end, tms_sequence_end);
}


/**
 * Perform JTAG SCAN-OUT operation at maximum TCK frequency.
 *
 * Data stored in EP2 OUT buffer is shifted into the JTAG chain via TDI, TDO
 * data is not sampled.
 * The TAP-FSM state is always left in the PAUSE-DR/PAUSE-IR state.
 *
 * Maximum achievable TCK frequency is 142 kHz for ANGIE clocked at 24 MHz.
 *
 * @param out_offset offset in EP1OUTBUF where payload data starts
 */
void jtag_scan_out(uint8_t out_offset)
{
	uint8_t scan_size_bytes, bits_last_byte;
	uint8_t tms_count_start, tms_count_end;
	uint8_t tms_sequence_start, tms_sequence_end;
	uint8_t tdi_data, i, j;
	uint8_t outb_buffer;

	/* Get parameters from EP1OUTBUF */
	scan_size_bytes = EP1OUTBUF[out_offset];
	bits_last_byte = EP1OUTBUF[out_offset + 1];
	tms_count_start = (EP1OUTBUF[out_offset + 2] >> 4) & 0x0F;
	tms_count_end = EP1OUTBUF[out_offset + 2] & 0x0F;
	tms_sequence_start = EP1OUTBUF[out_offset + 3];
	tms_sequence_end = EP1OUTBUF[out_offset + 4];

	if (tms_count_start > 0)
		jtag_clock_tms(tms_count_start, tms_sequence_start);
	outb_buffer = IOB & ~(bmbit2 | bmbit1);

	/* Shift all bytes except the last byte */
	for (i = 0; i < scan_size_bytes - 1; i++) {
		tdi_data = EP1OUTBUF[i + out_offset + 5];

		for (j = 0; j < 8; j++) {
			if (tdi_data & 0x01)
				outb_buffer |= bmbit3;
			else
				outb_buffer &= ~bmbit3;

			IOB = outb_buffer;	/* TDI and TCK change here */
			tdi_data = tdi_data >> 1;
			IOB = (outb_buffer | bmbit2);
		}
	}
	tdi_data = EP1OUTBUF[i + out_offset + 5];

	/* Shift the last byte */
	for (j = 0; j < bits_last_byte; j++) {
		if (tdi_data & 0x01)
			outb_buffer |= bmbit3;
		else
			outb_buffer &= ~bmbit3;

		/* Assert TMS signal if requested and this is the last bit */
		if (j == (bits_last_byte - 1) && tms_count_end > 0) {
			outb_buffer |= bmbit1;
			tms_count_end--;
			tms_sequence_end = tms_sequence_end >> 1;
		}
		IOB = outb_buffer;	/* TDI and TCK change here */
		tdi_data = tdi_data >> 1;
		IOB = (outb_buffer | bmbit2);
	}

	/* Move to correct end state */
	if (tms_count_end > 0)
		jtag_clock_tms(tms_count_end, tms_sequence_end);
}

/**
 * Perform JTAG SCAN-OUT operation at maximum TCK frequency.
 *
 * Data stored in EP2 OUT buffer is shifted into the JTAG chain via TDI, TDO
 * data is not sampled.
 * The TAP-FSM state is always left in the PAUSE-DR/PAUSE-IR state.
 *
 * Maximum achievable TCK frequency is 97 kHz for ANGIE clocked at 24 MHz.
 *
 * @param out_offset offset in EP1OUTBUF where payload data starts
 */
void jtag_slow_scan_out(uint8_t out_offset)
{
	uint8_t scan_size_bytes, bits_last_byte;
	uint8_t tms_count_start, tms_count_end;
	uint8_t tms_sequence_start, tms_sequence_end;
	uint8_t tdi_data, i, j, k;
	uint8_t outb_buffer;

	/* Get parameters from EP1OUTBUF */
	scan_size_bytes = EP1OUTBUF[out_offset];
	bits_last_byte = EP1OUTBUF[out_offset + 1];
	tms_count_start = (EP1OUTBUF[out_offset + 2] >> 4) & 0x0F;
	tms_count_end = EP1OUTBUF[out_offset + 2] & 0x0F;
	tms_sequence_start = EP1OUTBUF[out_offset + 3];
	tms_sequence_end = EP1OUTBUF[out_offset + 4];

	if (tms_count_start > 0)
		jtag_slow_clock_tms(tms_count_start, tms_sequence_start);
	outb_buffer = IOB & ~(bmbit2 | bmbit1);

	/* Shift all bytes except the last byte */
	for (i = 0; i < scan_size_bytes - 1; i++) {
		tdi_data = EP1OUTBUF[i + out_offset + 5];

		for (j = 0; j < 8; j++) {
			if (tdi_data & 0x01)
				outb_buffer |= bmbit3;
			else
				outb_buffer &= ~bmbit3;
			IOB = outb_buffer;	/* TDI and TCK change here */
			for (k = 0; k < delay_scan_out; k++)
				;
			tdi_data = tdi_data >> 1;
			IOB = (outb_buffer | bmbit2);
			for (k = 0; k < delay_scan_out; k++)
				;
		}
	}
	tdi_data = EP1OUTBUF[i + out_offset + 5];

	/* Shift the last byte */
	for (j = 0; j < bits_last_byte; j++) {
		if (tdi_data & 0x01)
			outb_buffer |= bmbit3;
		else
			outb_buffer &= ~bmbit3;

		/* Assert TMS signal if requested and this is the last bit */
		if (j == (bits_last_byte - 1) && tms_count_end > 0) {
			outb_buffer |= bmbit1;
			tms_count_end--;
			tms_sequence_end = tms_sequence_end >> 1;
		}
		IOB = outb_buffer;	/* TDI and TCK change here */
		for (k = 0; k < delay_scan_out; k++)
			;
		tdi_data = tdi_data >> 1;
		IOB = (outb_buffer | bmbit2);
		for (k = 0; k < delay_scan_out; k++)
			;
	}

	/* Move to correct end state */
	if (tms_count_end > 0)
		jtag_slow_clock_tms(tms_count_end, tms_sequence_end);
}


/**
 * Perform bidirectional JTAG SCAN operation at maximum TCK frequency.
 *
 * Data stored in EP2 OUT buffer is shifted into the JTAG chain via TDI, TDO
 * data is sampled and stored in the EP2 IN buffer.
 * The TAP-FSM state is always left in the PAUSE-DR/PAUSE-IR state.
 *
 * Maximum achievable TCK frequency is 100 kHz for ANGIE clocked at 24 MHz.
 *
 * @param out_offset offset in EP1OUTBUF where payload data starts
 * @param in_offset
 */
int it;
void jtag_scan_io(uint8_t out_offset, uint8_t in_offset)
{
	uint8_t scan_size_bytes, bits_last_byte;
	uint8_t tms_count_start, tms_count_end;
	uint8_t tms_sequence_start, tms_sequence_end;
	uint8_t tdi_data, tdo_data, i, j;
	uint8_t outb_buffer;

	it++;
	/* Get parameters from EP1OUTBUF */
	scan_size_bytes = EP1OUTBUF[out_offset];
	bits_last_byte = EP1OUTBUF[out_offset + 1];
	tms_count_start = (EP1OUTBUF[out_offset + 2] >> 4) & 0x0F;
	tms_count_end = EP1OUTBUF[out_offset + 2] & 0x0F;
	tms_sequence_start = EP1OUTBUF[out_offset + 3];
	tms_sequence_end = EP1OUTBUF[out_offset + 4];

	if (tms_count_start > 0)
		jtag_clock_tms(tms_count_start, tms_sequence_start);
	outb_buffer = IOB & ~(bmbit2 | bmbit1);

	/* Shift all bytes except the last byte */
	for (i = 0; i < scan_size_bytes - 1; i++) {
		tdi_data = EP1OUTBUF[i + out_offset + 5];
		tdo_data = 0;
		for (j = 0; j < 8; j++) {
			if (tdi_data & 0x01)
				outb_buffer |= bmbit3;
			else
				outb_buffer &= ~bmbit3;
			IOB = outb_buffer;	/* TDI and TCK change here */
			tdi_data = tdi_data >> 1;
			IOB = (outb_buffer | bmbit2);
			tdo_data = tdo_data >> 1;
			if (PIN_TDO)
				tdo_data |= 0x80;
		}

		/* Copy TDO data to EP1INBUF */
		EP1INBUF[i + in_offset] = tdo_data;
	}
	tdi_data = EP1OUTBUF[i + out_offset + 5];
	tdo_data = 0;

	/* Shift the last byte */
	for (j = 0; j < bits_last_byte; j++) {
		if (tdi_data & 0x01)
			outb_buffer |= bmbit3;
		else
			outb_buffer &= ~bmbit3;

		/* Assert TMS signal if requested and this is the last bit */
		if (j == (bits_last_byte - 1) && tms_count_end > 0) {
			outb_buffer |= bmbit1;
			tms_count_end--;
			tms_sequence_end = tms_sequence_end >> 1;
		}
		IOB = outb_buffer;	/* TDI and TCK change here */
		tdi_data = tdi_data >> 1;
		IOB = (outb_buffer | bmbit2);
		tdo_data = tdo_data >> 1;
		if (PIN_TDO)
			tdo_data |= 0x80;
	}
	tdo_data = tdo_data >> (8 - bits_last_byte);

	/* Copy TDO data to EP1INBUF */
	EP1INBUF[i + in_offset] = tdo_data;

	/* Move to correct end state */
	if (tms_count_end > 0)
		jtag_clock_tms(tms_count_end, tms_sequence_end);
}

/**
 * Perform bidirectional JTAG SCAN operation at maximum TCK frequency.
 *
 * Data stored in EP2 OUT buffer is shifted into the JTAG chain via TDI, TDO
 * data is sampled and stored in the EP2 IN buffer.
 * The TAP-FSM state is always left in the PAUSE-DR/PAUSE-IR state.
 *
 * Maximum achievable TCK frequency is 78 kHz for ANGIE clocked at 24 MHz.
 *
 * @param out_offset offset in EP1OUTBUF where payload data starts
 * @param in_offset
 */
void jtag_slow_scan_io(uint8_t out_offset, uint8_t in_offset)
{
	uint8_t scan_size_bytes, bits_last_byte;
	uint8_t tms_count_start, tms_count_end;
	uint8_t tms_sequence_start, tms_sequence_end;
	uint8_t tdi_data, tdo_data, i, j, k;
	uint8_t outb_buffer;

	/* Get parameters from EP1OUTBUF */
	scan_size_bytes = EP1OUTBUF[out_offset];
	bits_last_byte = EP1OUTBUF[out_offset + 1];
	tms_count_start = (EP1OUTBUF[out_offset + 2] >> 4) & 0x0F;
	tms_count_end = EP1OUTBUF[out_offset + 2] & 0x0F;
	tms_sequence_start = EP1OUTBUF[out_offset + 3];
	tms_sequence_end = EP1OUTBUF[out_offset + 4];

	if (tms_count_start > 0)
		jtag_slow_clock_tms(tms_count_start, tms_sequence_start);
	outb_buffer = IOB & ~(bmbit2 | bmbit1);

	/* Shift all bytes except the last byte */
	for (i = 0; i < scan_size_bytes - 1; i++) {
		tdi_data = EP1OUTBUF[i + out_offset + 5];
		tdo_data = 0;
		for (j = 0; j < 8; j++) {
			if (tdi_data & 0x01)
				outb_buffer |= bmbit3;
			else
				outb_buffer &= ~bmbit3;
			IOB = outb_buffer;	/* TDI and TCK change here */
			for (k = 0; k < delay_scan_io; k++)
				;
			tdi_data = tdi_data >> 1;
			IOB = (outb_buffer | bmbit2);
			for (k = 0; k < delay_scan_io; k++)
				;
			tdo_data = tdo_data >> 1;
			if (PIN_TDO)
				tdo_data |= 0x80;
		}

		/* Copy TDO data to EP1INBUF */
		EP1INBUF[i + in_offset] = tdo_data;
	}
	tdi_data = EP1OUTBUF[i + out_offset + 5];
	tdo_data = 0;

	/* Shift the last byte */
	for (j = 0; j < bits_last_byte; j++) {
		if (tdi_data & 0x01)
			outb_buffer |= bmbit3;
		else
			outb_buffer &= ~bmbit3;

		/* Assert TMS signal if requested and this is the last bit */
		if (j == (bits_last_byte - 1) && tms_count_end > 0) {
			outb_buffer |= bmbit1;
			tms_count_end--;
			tms_sequence_end = tms_sequence_end >> 1;
		}
		IOB = outb_buffer;	/* TDI and TCK change here */
		for (k = 0; k < delay_scan_io; k++)
			;
		tdi_data = tdi_data >> 1;
		IOB = (outb_buffer | bmbit2);
		for (k = 0; k < delay_scan_io; k++)
			;
		tdo_data = tdo_data >> 1;
		if (PIN_TDO)
			tdo_data |= 0x80;
	}
	tdo_data = tdo_data >> (8 - bits_last_byte);

	/* Copy TDO data to EP1INBUF */
	EP1INBUF[i + in_offset] = tdo_data;

	/* Move to correct end state */
	if (tms_count_end > 0)
		jtag_slow_clock_tms(tms_count_end, tms_sequence_end);
}

/**
 * Generate TCK clock cycles.
 *
 * Maximum achievable TCK frequency is 375 kHz for ANGIE clocked at 24 MHz.
 *
 * @param count number of TCK clock cycles to generate.
 */
void jtag_clock_tck(uint16_t count)
{
	uint16_t i;
	uint8_t outb_buffer = IOB & ~(bmbit2);

	for (i = 0; i < count; i++) {
		IOB = outb_buffer;
		IOB = outb_buffer | bmbit2;
	}
}

/**
 * Generate TCK clock cycles at variable frequency.
 *
 * Maximum achievable TCK frequency is 166.6 kHz for ANGIE clocked at 24 MHz.
 *
 * @param count number of TCK clock cycles to generate.
 */
void jtag_slow_clock_tck(uint16_t count)
{
	uint16_t i;
	uint8_t j;
	uint8_t outb_buffer = IOB & ~(bmbit2);

	for (i = 0; i < count; i++) {
		IOB = outb_buffer;
		for (j = 0; j < delay_tck; j++)
			;
		IOB = outb_buffer | bmbit2;
		for (j = 0; j < delay_tck; j++)
			;
	}
}

/**
 * Perform TAP FSM state transitions at maximum TCK frequency.
 *
 * Maximum achievable TCK frequency is 176 kHz for ANGIE clocked at 24 MHz.
 *
 * @param count the number of state transitions to perform.
 * @param sequence the TMS pin levels for each state transition, starting with
 *  the least-significant bit.
 */
void jtag_clock_tms(uint8_t count, uint8_t sequence)
{
	uint8_t outb_buffer = IOB & ~(bmbit2);
	uint8_t i;

	for (i = 0; i < count; i++) {
		/* Set TMS pin according to sequence parameter */
		if (sequence & 0x1)
			outb_buffer |= bmbit1;
		else
			outb_buffer &= ~bmbit1;
		IOB = outb_buffer;
		sequence = sequence >> 1;
		IOB = outb_buffer | bmbit2;
	}
}

/**
 * Perform TAP-FSM state transitions at less than maximum TCK frequency.
 *
 * Maximum achievable TCK frequency is 117 kHz for ANGIE clocked at 24 MHz.
 *
 * @param count the number of state transitions to perform.
 * @param sequence the TMS pin levels for each state transition, starting with
 *  the least-significant bit.
 */
void jtag_slow_clock_tms(uint8_t count, uint8_t sequence)
{
	uint8_t outb_buffer = IOB & ~(bmbit2);
	uint8_t i, j;

	for (i = 0; i < count; i++) {
		/* Set TMS pin according to sequence parameter */
		if (sequence & 0x1)
			outb_buffer |= bmbit1;
		else
			outb_buffer &= ~bmbit1;
		IOB = outb_buffer;
		for (j = 0; j < delay_tms; j++)
			;
		sequence = sequence >> 1;
		IOB = outb_buffer | bmbit2;
		for (j = 0; j < delay_tms; j++)
			;
	}
}

uint16_t jtag_get_signals(void)
{
	uint8_t input_signal_state, output_signal_state;
	input_signal_state = 0;
	output_signal_state = 0;

	/* Get states of input pins */
	if (PIN_TDO)
		input_signal_state |= SIGNAL_TDO;

	/* Get states of output pins */
	output_signal_state = IOB & MASK_PORTB_DIRECTION_OUT;

	return ((uint16_t)input_signal_state << 8) | ((uint16_t)output_signal_state);
}

/**
 * Set state of JTAG output signals.
 *
 * @param low signals which should be de-asserted.
 * @param high signals which should be asserted.
 */
void jtag_set_signals(uint8_t low, uint8_t high)
{
	IOB &= ~(low & MASK_PORTB_DIRECTION_OUT);
	IOB |= (high & MASK_PORTB_DIRECTION_OUT);
}

/**
 * Configure TCK delay parameters.
 *
 * @param scan_in number of delay cycles in scan_in operations.
 * @param scan_out number of delay cycles in scan_out operations.
 * @param scan_io number of delay cycles in scan_io operations.
 * @param tck number of delay cycles in clock_tck operations.
 * @param tms number of delay cycles in clock_tms operations.
 */
void jtag_configure_tck_delay(uint8_t scan_in, uint8_t scan_out,
	uint8_t scan_io, uint8_t tck, uint8_t tms)
{
	delay_scan_in = scan_in;
	delay_scan_out = scan_out;
	delay_scan_io = scan_io;
	delay_tck = tck;
	delay_tms = tms;
}
