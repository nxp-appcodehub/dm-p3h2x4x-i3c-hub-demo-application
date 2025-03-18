/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdarg.h>
#include <stdlib.h>
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#include <stdio.h>
#endif
#include <math.h>
#include "board.h"
#include <hardware_init.h>
#include "fsl_debug_console_cmsis.h"
#include "fsl_lpuart_cmsis.h"
#include "fsl_str.h"

#define SCANF_SHOW_INPUT_CHARACTERS 0

volatile bool rxBufferEmpty           = true;
volatile bool txBufferFull             = false;
volatile bool txOnGoing               = false;
volatile bool rxOnGoing                = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void USART_SignalEvent(uint32_t event)
{
	if (ARM_USART_EVENT_SEND_COMPLETE == event)
	{
		txBufferFull = false;
		txOnGoing    = false;
	}

	if (ARM_USART_EVENT_RECEIVE_COMPLETE == event)
	{
		rxBufferEmpty = false;
		rxOnGoing     = false;
	}
}

/*! @brief Keil: suppress ellipsis warning in va_arg usage below. */
#if defined(__CC_ARM)
#pragma diag_suppress 1256
#endif /* __CC_ARM */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief This definition is maximum line that debugconsole can scanf each time.*/
#define IO_MAXLINE 20U
int32_t status;

/*! @brief The overflow value.*/
#ifndef HUGE_VAL
#define HUGE_VAL (99.e99)
#endif /* HUGE_VAL */

/*! @brief Type of KSDK printf function pointer. */
typedef int (*PUTCHAR_FUNC)(int a);


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if (defined(SDK_DEBUGCONSOLE) && (SDK_DEBUGCONSOLE == DEBUGCONSOLE_REDIRECT_TO_SDK))
static int DebugConsole_PrintfFormattedData(PUTCHAR_FUNC func_ptr, const char *fmt, va_list ap);
#endif /* SDK_DEBUGCONSOLE */

/*******************************************************************************
 * Code
 ******************************************************************************/

/*************Code for DbgConsole Init, Deinit, Printf, Scanf *******************************/

#if ((SDK_DEBUGCONSOLE == DEBUGCONSOLE_REDIRECT_TO_SDK) || defined(SDK_DEBUGCONSOLE_UART))

status_t DebugConsole_Init(void)
{
	status = USART_LPUART.Initialize(USART_SignalEvent);
	if (ARM_DRIVER_OK != status)
	{
		return ARM_DRIVER_ERROR;
	}
}

status_t DebugConsole_Deinit(void)
{
	status = USART_LPUART.Uninitialize();
	if (ARM_DRIVER_OK != status)
	{
		return ARM_DRIVER_ERROR;
	}
}

status_t DebugConsole_PowerControl(void)
{
	status = USART_LPUART.PowerControl(ARM_POWER_FULL);
	if (ARM_DRIVER_OK != status)
	{
		return ARM_DRIVER_ERROR;
	}
}

status_t DebugConsole_Control(void)
{
	status = USART_LPUART.Control(ARM_USART_MODE_ASYNCHRONOUS, BOARD_DEBUG_UART_BAUDRATE);
	if (ARM_DRIVER_OK != status)
	{
		return ARM_DRIVER_ERROR;
	}
}

#endif /* DEBUGCONSOLE_REDIRECT_TO_SDK */

#if (defined(SDK_DEBUGCONSOLE) && (SDK_DEBUGCONSOLE == DEBUGCONSOLE_REDIRECT_TO_SDK))

int DebugConsole_Printf(const char *fmt_s, ...)
{
	va_list ap;
	int result = 0;

	va_start(ap, fmt_s);
	result = DebugConsole_Vprintf(fmt_s, ap);
	va_end(ap);

	return result;
}


int DebugConsole_Vprintf(const char *fmt_s, va_list formatStringArg)
{
	int result = 0;
	result = DebugConsole_PrintfFormattedData(DebugConsole_Putchar, fmt_s, formatStringArg);

	return result;
}

int DebugConsole_Putchar(int dbgConsoleCh)
{
	txOnGoing = true;
	status = USART_LPUART.Send((uint8_t *)(&dbgConsoleCh), 1);
	if (ARM_DRIVER_OK != status)
	{
		return ARM_DRIVER_ERROR;
	}
	while (txOnGoing);
	return 1;
}

int DebugConsole_Scanf(char *fmt_s, ...)
{
	/* Plus one to store end of string char */
	char temp_buf[IO_MAXLINE + 1];
	va_list ap;
	int32_t i;
	char result;

	va_start(ap, fmt_s);
	temp_buf[0] = '\0';

	i = 0;
	while (true)
	{
		if (i >= (int32_t)IO_MAXLINE)
		{
			break;
		}

		result      = (char)DebugConsole_Getchar();
#if SCANF_SHOW_INPUT_CHARACTERS
		DebugConsole_Putchar(result);
#endif
		temp_buf[i] = result;

		if ((result == '\r') || (result == '\n'))
		{
			/* End of Line. */
			if (i == 0)
			{
				temp_buf[i] = '\0';
				i           = -1;
			}
			else
			{
				break;
			}
		}

		i++;
	}

	if (i == (int32_t)IO_MAXLINE)
	{
		temp_buf[i] = '\0';
	}
	else
	{
		temp_buf[i + 1] = '\0';
	}
	result = (char)StrFormatScanf(temp_buf, fmt_s, ap);
	va_end(ap);

	return (int)result;
}

int DebugConsole_Getchar(void)
{
	char dbgConsoleCh;

	/* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
	if (!rxOnGoing)
	{
		rxOnGoing = true;

		if (kStatus_Success != USART_LPUART.Receive((uint8_t *)(&dbgConsoleCh), 1))
		{
			return ARM_DRIVER_ERROR;
		}
		while (rxOnGoing);
	}

	return (int)dbgConsoleCh;

}

/*************Code for process formatted data*******************************/
/*!
 * @brief This function puts padding character.
 *
 * @param[in] c         Padding character.
 * @param[in] curlen    Length of current formatted string .
 * @param[in] width     Width of expected formatted string.
 * @param[in] count     Number of characters.
 * @param[in] func_ptr  Function to put character out.
 */
static void DebugConsole_PrintfPaddingCharacter(
		char c, int32_t curlen, int32_t width, int32_t *count, PUTCHAR_FUNC func_ptr)
{
	int32_t i;

	for (i = curlen; i < width; i++)
	{
		(void)func_ptr(c);
		(*count)++;
	}
}

/*!
 * @brief Converts a radix number to a string and return its length.
 *
 * @param[in] numstr    Converted string of the number.
 * @param[in] nump      Pointer to the number.
 * @param[in] neg       Polarity of the number.
 * @param[in] radix     The radix to be converted to.
 * @param[in] use_caps  Used to identify %x/X output format.

 * @return Length of the converted string.
 */
static int32_t DebugConsole_ConvertRadixNumToString(char *numstr, void *nump, int32_t neg, int32_t radix, bool use_caps)
{
#if PRINTF_ADVANCED_ENABLE
	long long int a;
	long long int b;
	long long int c;

	unsigned long long int ua;
	unsigned long long int ub;
	unsigned long long int uc;
	unsigned long long int uc_param;
#else
	int a;
	int b;
	int c;

	unsigned int ua;
	unsigned int ub;
	unsigned int uc;
	unsigned int uc_param;
#endif /* PRINTF_ADVANCED_ENABLE */

	int32_t nlen;
	char *nstrp;

	nlen     = 0;
	nstrp    = numstr;
	*nstrp++ = '\0';

#if !(PRINTF_ADVANCED_ENABLE > 0)
	neg = 0;
#endif

#if PRINTF_ADVANCED_ENABLE
	a        = 0;
	b        = 0;
	c        = 0;
	ua       = 0ULL;
	ub       = 0ULL;
	uc       = 0ULL;
	uc_param = 0ULL;
#else
	a        = 0;
	b        = 0;
	c        = 0;
	ua       = 0U;
	ub       = 0U;
	uc       = 0U;
	uc_param = 0U;
#endif /* PRINTF_ADVANCED_ENABLE */

	(void)a;
	(void)b;
	(void)c;
	(void)ua;
	(void)ub;
	(void)uc;
	(void)uc_param;
	(void)neg;
	/*
	 * Fix MISRA issue: CID 15985711 (#15 of 15): MISRA C-2012 Control Flow Expressions (MISRA C-2012 Rule 14.3)
	 * misra_c_2012_rule_14_3_violation: Execution cannot reach this statement: a = *((int *)nump);
	 */
#if PRINTF_ADVANCED_ENABLE
	if (0 != neg)
	{
#if PRINTF_ADVANCED_ENABLE
		a = *(long long int *)nump;
#else
		a = *(int *)nump;
#endif /* PRINTF_ADVANCED_ENABLE */
		if (a == 0)
		{
			*nstrp = '0';
			++nlen;
			return nlen;
		}
		while (a != 0)
		{
#if PRINTF_ADVANCED_ENABLE
			b = (long long int)a / (long long int)radix;
			c = (long long int)a - ((long long int)b * (long long int)radix);
			if (c < 0)
			{
				uc       = (unsigned long long int)c;
				uc_param = ~uc;
				c        = (long long int)uc_param + 1 + (long long int)'0';
			}
#else
	b = (int)a / (int)radix;
	c = (int)a - ((int)b * (int)radix);
	if (c < 0)
	{
		uc       = (unsigned int)c;
		uc_param = ~uc;
		c        = (int)uc_param + 1 + (int)'0';
	}
#endif /* PRINTF_ADVANCED_ENABLE */
else
{
	c = c + (int)'0';
}
	a        = b;
	*nstrp++ = (char)c;
	++nlen;
		}
	}
	else
#endif /* PRINTF_ADVANCED_ENABLE */
	{
#if PRINTF_ADVANCED_ENABLE
		ua = *(unsigned long long int *)nump;
#else
		ua = *(unsigned int *)nump;
#endif /* PRINTF_ADVANCED_ENABLE */
		if (ua == 0U)
		{
			*nstrp = '0';
			++nlen;
			return nlen;
		}
		while (ua != 0U)
		{
#if PRINTF_ADVANCED_ENABLE
			ub = (unsigned long long int)ua / (unsigned long long int)radix;
			uc = (unsigned long long int)ua - ((unsigned long long int)ub * (unsigned long long int)radix);
#else
			ub = ua / (unsigned int)radix;
			uc = ua - (ub * (unsigned int)radix);
#endif /* PRINTF_ADVANCED_ENABLE */

			if (uc < 10U)
			{
				uc = uc + (unsigned int)'0';
			}
			else
			{
				uc = uc - 10U + (unsigned int)(use_caps ? 'A' : 'a');
			}
			ua       = ub;
			*nstrp++ = (char)uc;
			++nlen;
		}
	}
	return nlen;
}

#if PRINTF_FLOAT_ENABLE
/*!
 * @brief Converts a floating radix number to a string and return its length.
 *
 * @param[in] numstr            Converted string of the number.
 * @param[in] nump              Pointer to the number.
 * @param[in] radix             The radix to be converted to.
 * @param[in] precision_width   Specify the precision width.

 * @return Length of the converted string.
 */
static int32_t DebugConsole_ConvertFloatRadixNumToString(char *numstr,
		void *nump,
		int32_t radix,
		uint32_t precision_width)
{
	int32_t a;
	int32_t b;
	int32_t c;
	uint32_t i;
	double fa;
	double dc;
	double fb;
	double r;
	double fractpart;
	double intpart;

	int32_t nlen;
	char *nstrp;
	nlen     = 0;
	nstrp    = numstr;
	*nstrp++ = '\0';
	r        = *(double *)nump;
	if (0.0 == r)
	{
		*nstrp = '0';
		++nlen;
		return nlen;
	}
	fractpart = modf((double)r, (double *)&intpart);
	/* Process fractional part. */
	for (i = 0; i < precision_width; i++)
	{
		fractpart *= (double)radix;
	}
	if (r >= 0.0)
	{
		fa = fractpart + (double)0.5;
		if (fa >= pow((double)10, (double)precision_width))
		{
			intpart++;
		}
	}
	else
	{
		fa = fractpart - (double)0.5;
		if (fa <= -pow((double)10, (double)precision_width))
		{
			intpart--;
		}
	}
	for (i = 0; i < precision_width; i++)
	{
		fb = fa / (double)radix;
		dc = (fa - (double)(long long int)fb * (double)radix);
		c  = (int32_t)dc;
		if (c < 0)
		{
			c = (int32_t)'0' - c;
		}
		else
		{
			c = c + '0';
		}
		fa       = fb;
		*nstrp++ = (char)c;
		++nlen;
	}
	*nstrp++ = (char)'.';
	++nlen;
	a = (int32_t)intpart;
	if (a == 0)
	{
		*nstrp++ = '0';
		++nlen;
	}
	else
	{
		while (a != 0)
		{
			b = (int32_t)a / (int32_t)radix;
			c = (int32_t)a - ((int32_t)b * (int32_t)radix);
			if (c < 0)
			{
				c = (int32_t)'0' - c;
			}
			else
			{
				c = c + '0';
			}
			a        = b;
			*nstrp++ = (char)c;
			++nlen;
		}
	}
	return nlen;
}
#endif /* PRINTF_FLOAT_ENABLE */

/*!
 * @brief This function outputs its parameters according to a formatted string.
 *
 * @note I/O is performed by calling given function pointer using following
 * (*func_ptr)(c);
 *
 * @param[in] func_ptr  Function to put character out.
 * @param[in] fmt       Format string for printf.
 * @param[in] ap        Arguments to printf.
 *
 * @return Number of characters
 */
static int DebugConsole_PrintfFormattedData(PUTCHAR_FUNC func_ptr, const char *fmt, va_list ap)
{
	/* va_list ap; */
	const char *p;
	char c;

	char vstr[33];
	char *vstrp  = NULL;
	int32_t vlen = 0;

	bool done;
	int32_t count = 0;

	uint32_t field_width;
	uint32_t precision_width;
	char *sval;
	int32_t cval;
	bool use_caps;
	uint8_t radix = 0;

#if PRINTF_ADVANCED_ENABLE
	uint32_t flags_used;
	char schar;
	bool dschar;
	long long int ival;
	unsigned long long int uval = 0;
	bool valid_precision_width;
#else
	int ival;
	unsigned int uval = 0;
#endif /* PRINTF_ADVANCED_ENABLE */

#if PRINTF_FLOAT_ENABLE
	double fval;
#endif /* PRINTF_FLOAT_ENABLE */

	/* Start parsing apart the format string and display appropriate formats and data. */
	p = fmt;
	while (true)
	{
		if ('\0' == *p)
		{
			break;
		}
		c = *p;
		/*
		 * All formats begin with a '%' marker.  Special chars like
		 * '\n' or '\t' are normally converted to the appropriate
		 * character by the __compiler__.  Thus, no need for this
		 * routine to account for the '\' character.
		 */
		if (c != '%')
		{
			(void)func_ptr(c);
			count++;
			p++;
			/* By using 'continue', the next iteration of the loop is used, skipping the code that follows. */
			continue;
		}

		use_caps = true;

#if PRINTF_ADVANCED_ENABLE
		/* First check for specification modifier flags. */
		flags_used = 0;
		done       = false;
		while (!done)
		{
			switch (*++p)
			{
			case '-':
			flags_used |= (uint32_t)kPRINTF_Minus;
			break;
			case '+':
				flags_used |= (uint32_t)kPRINTF_Plus;
				break;
			case ' ':
				flags_used |= (uint32_t)kPRINTF_Space;
				break;
			case '0':
				flags_used |= (uint32_t)kPRINTF_Zero;
				break;
			case '#':
				flags_used |= (uint32_t)kPRINTF_Pound;
				break;
			default:
				/* We've gone one char too far. */
				--p;
				done = true;
				break;
			}
		}
#endif /* PRINTF_ADVANCED_ENABLE */

		/* Next check for minimum field width. */
		field_width = 0;
		done        = false;
		while (!done)
		{
			c = *++p;
			if ((c >= '0') && (c <= '9'))
			{
				field_width = (field_width * 10U) + ((uint32_t)c - (uint32_t)'0');
			}
#if PRINTF_ADVANCED_ENABLE
			else if (c == '*')
			{
				field_width = (uint32_t)va_arg(ap, unsigned int);
			}
#endif /* PRINTF_ADVANCED_ENABLE */
			else
			{
				/* We've gone one char too far. */
				--p;
				done = true;
			}
		}
		/* Next check for the width and precision field separator. */
#if (PRINTF_ADVANCED_ENABLE || PRINTF_FLOAT_ENABLE)
		precision_width = 6U; /* MISRA C-2012 Rule 2.2 */
#endif
#if PRINTF_ADVANCED_ENABLE
		valid_precision_width = false;
#endif /* PRINTF_ADVANCED_ENABLE */
		if (*++p == '.')
		{
			/* Must get precision field width, if present. */
			precision_width = 0U;
			done            = false;
			while (!done)
			{
				c = *++p;
				if ((c >= '0') && (c <= '9'))
				{
					precision_width = (precision_width * 10U) + ((uint32_t)c - (uint32_t)'0');
#if PRINTF_ADVANCED_ENABLE
					valid_precision_width = true;
#endif /* PRINTF_ADVANCED_ENABLE */
				}
#if PRINTF_ADVANCED_ENABLE
				else if (c == '*')
				{
					precision_width       = (uint32_t)va_arg(ap, unsigned int);
					valid_precision_width = true;
				}
#endif /* PRINTF_ADVANCED_ENABLE */
				else
				{
					/* We've gone one char too far. */
					--p;
					done = true;
				}
			}
		}
		else
		{
			/* We've gone one char too far. */
			--p;
		}
#if PRINTF_ADVANCED_ENABLE
		/*
		 * Check for the length modifier.
		 */
		switch (/* c = */ *++p)
		{
		case 'h':
			if (*++p != 'h')
			{
				flags_used |= (uint32_t)kPRINTF_LengthShortInt;
				--p;
			}
			else
			{
				flags_used |= (uint32_t)kPRINTF_LengthChar;
			}
			break;
		case 'l':
			if (*++p != 'l')
			{
				flags_used |= (uint32_t)kPRINTF_LengthLongInt;
				--p;
			}
			else
			{
				flags_used |= (uint32_t)kPRINTF_LengthLongLongInt;
			}
			break;
		case 'z':
			if (sizeof(size_t) == sizeof(uint32_t))
			{
				flags_used |= (uint32_t)kPRINTF_LengthLongInt;
			}
			else if (sizeof(size_t) == (2U * sizeof(uint32_t)))
			{
				flags_used |= (uint32_t)kPRINTF_LengthLongLongInt;
			}
			else if (sizeof(size_t) == sizeof(uint16_t))
			{
				flags_used |= (uint32_t)kPRINTF_LengthShortInt;
			}
			else
			{
				/* MISRA C-2012 Rule 15.7 */
			}
			break;
		default:
			/* we've gone one char too far */
			--p;
			break;
		}
#endif /* PRINTF_ADVANCED_ENABLE */
		/* Now we're ready to examine the format. */
		c = *++p;
		{
			if ((c == 'd') || (c == 'i') || (c == 'f') || (c == 'F') || (c == 'x') || (c == 'X') || (c == 'o') ||
					(c == 'b') || (c == 'p') || (c == 'u'))
			{
				if ((c == 'd') || (c == 'i'))
				{
#if PRINTF_ADVANCED_ENABLE
					if (0U != (flags_used & (uint32_t)kPRINTF_LengthLongLongInt))
					{
						ival = (long long int)va_arg(ap, long long int);
					}
					else if (0U != (flags_used & (uint32_t)kPRINTF_LengthLongInt))
					{
						ival = (long int)va_arg(ap, long int);
					}
					else
#endif /* PRINTF_ADVANCED_ENABLE */
{
						ival = (int)va_arg(ap, int);
}
					vlen  = DebugConsole_ConvertRadixNumToString(vstr, &ival, 1, 10, use_caps);
					vstrp = &vstr[vlen];
#if PRINTF_ADVANCED_ENABLE
					if (ival < 0)
					{
						schar = '-';
						++vlen;
					}
					else
					{
						if (0U != (flags_used & (uint32_t)kPRINTF_Plus))
						{
							schar = '+';
							++vlen;
						}
						else
						{
							if (0U != (flags_used & (uint32_t)kPRINTF_Space))
							{
								schar = ' ';
								++vlen;
							}
							else
							{
								schar = '\0';
							}
						}
					}
					dschar = false;
					/* Do the ZERO pad. */
					if (0U != (flags_used & (uint32_t)kPRINTF_Zero))
					{
						if ('\0' != schar)
						{
							(void)func_ptr(schar);
							count++;
						}
						dschar = true;

						DebugConsole_PrintfPaddingCharacter('0', vlen, (int32_t)field_width, &count, func_ptr);
						vlen = (int32_t)field_width;
					}
					else
					{
						if (0U == (flags_used & (uint32_t)kPRINTF_Minus))
						{
							DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
							if ('\0' != schar)
							{
								(void)func_ptr(schar);
								count++;
							}
							dschar = true;
						}
					}
					/* The string was built in reverse order, now display in correct order. */
					if ((!dschar) && ('\0' != schar))
					{
						(void)func_ptr(schar);
						count++;
					}
#endif /* PRINTF_ADVANCED_ENABLE */
				}

#if PRINTF_FLOAT_ENABLE
				if ((c == 'f') || (c == 'F'))
				{
					fval  = (double)va_arg(ap, double);
					vlen  = DebugConsole_ConvertFloatRadixNumToString(vstr, &fval, 10, precision_width);
					vstrp = &vstr[vlen];

#if PRINTF_ADVANCED_ENABLE
					if (fval < 0.0)
					{
						schar = '-';
						++vlen;
					}
					else
					{
						if (0U != (flags_used & (uint32_t)kPRINTF_Plus))
						{
							schar = '+';
							++vlen;
						}
						else
						{
							if (0U != (flags_used & (uint32_t)kPRINTF_Space))
							{
								schar = ' ';
								++vlen;
							}
							else
							{
								schar = '\0';
							}
						}
					}
					dschar = false;
					if (0U != (flags_used & (uint32_t)kPRINTF_Zero))
					{
						if ('\0' != schar)
						{
							(void)func_ptr(schar);
							count++;
						}
						dschar = true;
						DebugConsole_PrintfPaddingCharacter('0', vlen, (int32_t)field_width, &count, func_ptr);
						vlen = (int32_t)field_width;
					}
					else
					{
						if (0U == (flags_used & (uint32_t)kPRINTF_Minus))
						{
							DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
							if ('\0' != schar)
							{
								(void)func_ptr(schar);
								count++;
							}
							dschar = true;
						}
					}
					if ((!dschar) && ('\0' != schar))
					{
						(void)func_ptr(schar);
						count++;
					}
#endif /* PRINTF_ADVANCED_ENABLE */
				}
#endif /* PRINTF_FLOAT_ENABLE */
				if ((c == 'X') || (c == 'x'))
				{
					if (c == 'x')
					{
						use_caps = false;
					}
#if PRINTF_ADVANCED_ENABLE
					if (0U != (flags_used & (uint32_t)kPRINTF_LengthLongLongInt))
					{
						uval = (unsigned long long int)va_arg(ap, unsigned long long int);
					}
					else if (0U != (flags_used & (uint32_t)kPRINTF_LengthLongInt))
					{
						uval = (unsigned long int)va_arg(ap, unsigned long int);
					}
					else
#endif /* PRINTF_ADVANCED_ENABLE */
{
						uval = (unsigned int)va_arg(ap, unsigned int);
}
					vlen  = DebugConsole_ConvertRadixNumToString(vstr, &uval, 0, 16, use_caps);
					vstrp = &vstr[vlen];

#if PRINTF_ADVANCED_ENABLE
					dschar = false;
					if (0U != (flags_used & (uint32_t)kPRINTF_Zero))
					{
						if (0U != (flags_used & (uint32_t)kPRINTF_Pound))
						{
							(void)func_ptr('0');
							(void)func_ptr((use_caps ? 'X' : 'x'));
							count += 2;
							/*vlen += 2;*/
							dschar = true;
						}
						DebugConsole_PrintfPaddingCharacter('0', vlen, (int32_t)field_width, &count, func_ptr);
						vlen = (int32_t)field_width;
					}
					else
					{
						if (0U == (flags_used & (uint32_t)kPRINTF_Minus))
						{
							if (0U != (flags_used & (uint32_t)kPRINTF_Pound))
							{
								vlen += 2;
							}
							DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
							if (0U != (flags_used & (uint32_t)kPRINTF_Pound))
							{
								(void)func_ptr('0');
								(void)func_ptr(use_caps ? 'X' : 'x');
								count += 2;

								dschar = true;
							}
						}
					}

					if ((0U != (flags_used & (uint32_t)kPRINTF_Pound)) && (!dschar))
					{
						(void)func_ptr('0');
						(void)func_ptr(use_caps ? 'X' : 'x');
						count += 2;
						vlen += 2;
					}
#endif /* PRINTF_ADVANCED_ENABLE */
				}
				if ((c == 'o') || (c == 'b') || (c == 'p') || (c == 'u'))
				{
					if ('p' == c)
					{
						/*
						 * Fix MISRA issue: CID 16209727 (#15 of 15): MISRA C-2012 Pointer Type Conversions (MISRA
						 * C-2012 Rule 11.6)
						 * 1. misra_c_2012_rule_11_6_violation: The expression va_arg (ap, void *) of type void * is
						 * cast to type unsigned int.
						 *
						 * Orignal code: uval = (unsigned int)va_arg(ap, void *);
						 */
						void *pval;
						pval = (void *)va_arg(ap, void *);
						(void)memcpy((void *)&uval, (void *)&pval, sizeof(void *));
					}
					else
					{
#if PRINTF_ADVANCED_ENABLE
						if (0U != (flags_used & (uint32_t)kPRINTF_LengthLongLongInt))
						{
							uval = (unsigned long long int)va_arg(ap, unsigned long long int);
						}
						else if (0U != (flags_used & (uint32_t)kPRINTF_LengthLongInt))
						{
							uval = (unsigned long int)va_arg(ap, unsigned long int);
						}
						else
#endif /* PRINTF_ADVANCED_ENABLE */
{
							uval = (unsigned int)va_arg(ap, unsigned int);
}
					}
					switch (c)
					{
					case 'o':
						radix = 8;
						break;
					case 'b':
						radix = 2;
						break;
					case 'p':
						radix = 16;
						break;
					case 'u':
						radix = 10;
						break;
					default:
						/* MISRA C-2012 Rule 16.4 */
						break;
					}
					vlen  = DebugConsole_ConvertRadixNumToString(vstr, &uval, 0, (int32_t)radix, use_caps);
					vstrp = &vstr[vlen];
#if PRINTF_ADVANCED_ENABLE
					if (0U != (flags_used & (uint32_t)kPRINTF_Zero))
					{
						DebugConsole_PrintfPaddingCharacter('0', vlen, (int32_t)field_width, &count, func_ptr);
						vlen = (int32_t)field_width;
					}
					else
					{
						if (0U == (flags_used & (uint32_t)kPRINTF_Minus))
						{
							DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
						}
					}
#endif /* PRINTF_ADVANCED_ENABLE */
				}
#if !PRINTF_ADVANCED_ENABLE
				DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
#endif /* !PRINTF_ADVANCED_ENABLE */
				if (vstrp != NULL)
				{
					while ('\0' != *vstrp)
					{
						(void)func_ptr(*vstrp--);
						count++;
					}
				}
#if PRINTF_ADVANCED_ENABLE
				if (0U != (flags_used & (uint32_t)kPRINTF_Minus))
				{
					DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
				}
#endif /* PRINTF_ADVANCED_ENABLE */
			}
			else if (c == 'c')
			{
				cval = (int32_t)va_arg(ap, unsigned int);
				(void)func_ptr(cval);
				count++;
			}
			else if (c == 's')
			{
				sval = (char *)va_arg(ap, char *);
				if (NULL != sval)
				{
#if PRINTF_ADVANCED_ENABLE
					if (valid_precision_width)
					{
						vlen = (int32_t)precision_width;
					}
					else
					{
						vlen = (int32_t)strlen(sval);
					}
#else
	vlen = (int32_t)strlen(sval);
#endif /* PRINTF_ADVANCED_ENABLE */
#if PRINTF_ADVANCED_ENABLE
	if (0U == (flags_used & (uint32_t)kPRINTF_Minus))
#endif /* PRINTF_ADVANCED_ENABLE */
	{
		DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
	}

#if PRINTF_ADVANCED_ENABLE
	if (valid_precision_width)
	{
		while (('\0' != *sval) && (vlen > 0))
		{
			(void)func_ptr(*sval++);
			count++;
			vlen--;
		}
		/* In case that vlen sval is shorter than vlen */
		vlen = (int32_t)precision_width - vlen;
	}
	else
	{
#endif /* PRINTF_ADVANCED_ENABLE */
		while ('\0' != *sval)
		{
			(void)func_ptr(*sval++);
			count++;
		}
#if PRINTF_ADVANCED_ENABLE
	}
#endif /* PRINTF_ADVANCED_ENABLE */

#if PRINTF_ADVANCED_ENABLE
	if (0U != (flags_used & (uint32_t)kPRINTF_Minus))
	{
		DebugConsole_PrintfPaddingCharacter(' ', vlen, (int32_t)field_width, &count, func_ptr);
	}
#endif /* PRINTF_ADVANCED_ENABLE */
				}
			}
			else
			{
				(void)func_ptr(c);
				count++;
			}
		}
		p++;
	}
	return count;
}

#endif /* SDK_DEBUGCONSOLE */

