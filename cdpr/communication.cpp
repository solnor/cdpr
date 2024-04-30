#include <iostream>
#include "communication.h"

#include <stdlib.h>
#include <Windows.h>
#include <string>
#include <fileapi.h>
#include <cassert>
#include <cstdlib>


int com_init(HANDLE handles[n_odrv], LPCSTR odrv_ports[n_odrv]) {

	DCB serialParams;
	COMMTIMEOUTS serialTimeouts;

	for (uint8_t i = 0; i < n_odrv; i++) {

		handles[i] = CreateFileA(odrv_ports[i],
			GENERIC_READ | GENERIC_WRITE,
			0, NULL, OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL, NULL);
		assert(handles[i] != INVALID_HANDLE_VALUE, "Unable to open serial line");

		GetCommState(handles[i], &serialParams);
		serialParams.BaudRate = 115200;
		serialParams.ByteSize = 8;
		serialParams.StopBits = ONESTOPBIT;
		serialParams.Parity = NOPARITY;
		SetCommState(handles[i], &serialParams);

		GetCommTimeouts(handles[i], &serialTimeouts);
		serialTimeouts.ReadIntervalTimeout = MAXDWORD;
		serialTimeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
		serialTimeouts.ReadTotalTimeoutConstant = 20; // 5 ms to respond after trying to read from the device
		serialTimeouts.WriteTotalTimeoutMultiplier = 0;
		serialTimeouts.WriteTotalTimeoutConstant = 0;
		SetCommTimeouts(handles[i], &serialTimeouts);
		std::cout << handles[i] << std::endl;
	}
	return 1;
}

int com_read_ln(HANDLE handle, char* c) {
	LPDWORD n = 0;
	int i = 0;
	while (1) {
		int r = ReadFile(handle, &c[i], 1, n, NULL);
		if (!r) {
			std::cout << "Failed to read line.\n" << std::endl;
		}
		std::cout << "Reading line: " << c << std::endl;
		if (*c == 0x0a) {
			break;
		}
		i++;
	}
	return (int)n;
}

int com_write_ln(HANDLE handle, char c[]) {
	LPDWORD n = 0;
	int r = WriteFile(handle, c, strlen(c), n, NULL);
	if (!r) {
		std::cout << "Failed to write line.\n" << std::endl;
	}
	//FlushFileBuffers(handle);
	return (int)n;
}

//int main() {
//	//std::string port = "\\\\.\\COM1";
//	LPCSTR port = "\\\\.\\COM4";
//	HANDLE handle = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
//	assert(handle != INVALID_HANDLE_VALUE, "Unable to open serial line");
//
//	DCB serialParams;
//	GetCommState(handle, &serialParams);
//	serialParams.BaudRate = 115200;
//	serialParams.ByteSize = 8;
//	serialParams.StopBits = ONESTOPBIT;
//	serialParams.Parity = NOPARITY;
//	SetCommState(handle, &serialParams);
//
//	COMMTIMEOUTS serialTimeouts;
//	GetCommTimeouts(handle, &serialTimeouts);
//	serialTimeouts.ReadIntervalTimeout = MAXDWORD;
//	serialTimeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
//	serialTimeouts.ReadTotalTimeoutConstant = 100; // 100 ms to respond after trying to read from the device
//	serialTimeouts.WriteTotalTimeoutMultiplier = 0;
//	serialTimeouts.WriteTotalTimeoutConstant = 0;
//
//	SetCommTimeouts(handle, &serialTimeouts);
//
//	LPDWORD n = 0;
//	char c[] = "f 0\n";
//	std::cout << strlen(c) << std::endl;
//	int r = WriteFile(handle, &c, strlen(c), n, NULL);
//	printf("n: %i\n", n);
//	printf("r : %i\n", r);
//	r = ReadFile(handle, &c, 10, n, NULL);
//	printf("n: %i\n", n);
//	printf("r : %i\n", r);
//	DWORD nRead;
//	//r = ReadFile(handle, buff2, 40, &nRead, NULL);
//
//	// Timeouts, se
//		// https ://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-commtimeouts
//	//std::cout << std::isprint(c) << std::endl;
//	std::cout << "Response:" << c << std::endl;
//	//std::cout << static_cast<unsigned>(c) << std::endl;
//	//std::cout << "Hello World!" << std::endl;
//	system("pause");
//	return 0;
//}



