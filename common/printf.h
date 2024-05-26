///////////////////////////////////////////////////////////////////////////////
// \author (c) Marco Paland (info@paland.com)
//             2014-2018, PALANDesign Hannover, Germany
//
// \license The MIT License (MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// \brief Tiny printf, sprintf and snprintf implementation, optimized for speed on
//        embedded systems with a very limited resources.
//        Use this instead of bloated standard/newlib printf.
//        These routines are thread safe and reentrant!
//
///////////////////////////////////////////////////////////////////////////////

#ifndef _PRINTF_H_
#define _PRINTF_H_

#include <stdarg.h>
#include <stddef.h>


#ifdef __cplusplus
extern "C" {
#endif


/**
 * Output a character to a custom device like UART, used by the printf() function
 * This function is declared here only. You have to write your custom implementation somewhere
 * \param character Character to output
 */
void _putchar(char character);


/**
 * Tiny printf implementation
 * You have to implement _putchar if you use printf()
 * \param format A string that specifies the format of the output
 * \return The number of characters that are written into the array, not counting the terminating null character
 */
int __printf(const char* format, ...);
#ifdef  USE_OPTIMIZE_PRINTF
#define printf(fmt, ...) do {	\
	static const char flash_str[] ICACHE_RODATA_ATTR STORE_ATTR = fmt;	\
	__printf(flash_str, ##__VA_ARGS__);	\
	} while(0)
#else
#define printf   __printf
#endif

/**
 * Tiny sprintf implementation
 * Due to security reasons (buffer overflow) YOU SHOULD CONSIDER USING (V)SNPRINTF INSTEAD!
 * \param buffer A pointer to the buffer where to store the formatted string. MUST be big enough to store the output!
 * \param format A string that specifies the format of the output
 * \return The number of characters that are WRITTEN into the buffer, not counting the terminating null character
 */
int __sprintf(char* buffer, const char* format, ...);
#ifdef  USE_OPTIMIZE_PRINTF
#define sprintf(buf, fmt, ...) do {	\
	static const char flash_str[] ICACHE_RODATA_ATTR STORE_ATTR = fmt;	\
	__sprintf(buf, flash_str, ##__VA_ARGS__);	\
	} while(0)
#else
#define sprintf   __sprintf
#endif

/**
 * Tiny snprintf/vsnprintf implementation
 * \param buffer A pointer to the buffer where to store the formatted string
 * \param count The maximum number of characters to store in the buffer, including a terminating null character
 * \param format A string that specifies the format of the output
 * \return The number of characters that are WRITTEN into the buffer, not counting the terminating null character
 *         If the formatted string is truncated the buffer size (count) is returned
 */
int  __snprintf(char* buffer, unsigned int count, const char* format, ...);
#ifdef  USE_OPTIMIZE_PRINTF
#define snprintf(buf, size, fmt, ...) do {	\
	static const char flash_str[] ICACHE_RODATA_ATTR STORE_ATTR = fmt;	\
	__snprintf(buf, size, flash_str, ##__VA_ARGS__);	\
	} while(0)
#else
#define _snprintf   snprintf
#endif

int __vsnprintf(char* buffer, unsigned int count, const char* format, va_list va);
#ifdef  USE_OPTIMIZE_PRINTF
#define vsnprintf(buf, size, fmt, va) do {	\
	static const char flash_str[] ICACHE_RODATA_ATTR STORE_ATTR = fmt;	\
	__vsnprintf(buf, size, flash_str, va);	\
	} while(0)
#else
#define vsnprintf   __vsnprintf
#endif

/**
 * printf with output function
 * You may use this as dynamic alternative to printf() with its fixed _putchar() output
 * \param out An output function which takes one character and an argument pointer
 * \param arg An argument pointer for user data passed to output function
 * \param format A string that specifies the format of the output
 * \return The number of characters that are sent to the output function, not counting the terminating null character
 */
int __fctprintf(void (*out)(char character, void* arg), void* arg, const char* format, ...);
#ifdef  USE_OPTIMIZE_PRINTF
#define fctprintf(fct, buf, fmt, ...) do {	\
	static const char flash_str[] ICACHE_RODATA_ATTR STORE_ATTR = fmt;	\
	__fctprintf(fct, buf, flash_str, ##__VA_ARGS__);	\
	} while(0)
#else
#define fctprintf   __fctprintf
#endif

int __vfctprintf(void (*out)(char character, void* arg), void* arg, const char* format, va_list va);
#ifdef  USE_OPTIMIZE_PRINTF
#define vfctprintf(fct, buf, fmt, va) do {	\
	static const char flash_str[] ICACHE_RODATA_ATTR STORE_ATTR = fmt;	\
	__vfctprintf(fct, buf, flash_str, va);	\
	} while(0)
#else
#define vfctprintf   __vfctprintf
#endif

#ifdef __cplusplus
}
#endif


#endif  // _PRINTF_H_
