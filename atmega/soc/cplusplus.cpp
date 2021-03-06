/*
 * ESP8266 C++ implementations.
 * 
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <lwiot.h>

#include <lwiot/types.h>

__extension__ typedef int __guard __attribute__((mode (__DI__)));

void *operator new(size_t num)
{
	auto ptr = lwiot_mem_alloc(num);

	memset(ptr, 0, sizeof(num));
	return ptr;
}

void *operator new[](size_t num)
{
	auto ptr = lwiot_mem_alloc(num);

	memset(ptr, 0, sizeof(num));
	return ptr;
}

void operator delete(void *ptr)
{
	if(ptr == nullptr)
		return;

	free(ptr);
}

void operator delete[](void *ptr)
{
	if(ptr == nullptr)
		return;

	free(ptr);
}

#if __cplusplus >= 201402L
void __attribute__((weak)) operator delete(void *ptr, size_t size) noexcept
{
	::operator delete(ptr);
}

void __attribute__((weak)) operator delete[](void *ptr, size_t size) noexcept
{
	::operator delete(ptr);
}
#endif

extern "C" {
	int atexit(void (*func)(void))
	{
		return -1;
	}

	int __cxa_guard_acquire(__guard *g)
	{
		return !*(char *) (g);
	}

	void __cxa_guard_release(__guard *g)
	{
		*(char *) g = 1;
	}

	extern void __cxa_pure_virtual(void) __attribute__((weak));
	void __cxa_pure_virtual(void)
	{
		printf("Virtual function not implemented (%s:%i)", __FILE__, __LINE__);
		for(;;);
	}

	void __cxa_guard_abort(__guard *g)
	{
		for(;;);
	}

	extern void __gxx_personality_v0(void) __attribute__((weak));
	void __gxx_personality_v0(void)
	{
	}

	}

