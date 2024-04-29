#pragma once
#include <Windows.h>

LPCSTR odrv0_port = "\\\\.\\COM4";
LPCSTR odrv1_port = "\\\\.\\COM4";
LPCSTR odrv2_port = "\\\\.\\COM4";
LPCSTR odrv3_port = "\\\\.\\COM4";

constexpr int n_odrv = 4;
HANDLE handles[n_odrv];
LPCSTR odrv_ports[n_odrv] = { odrv0_port, odrv1_port, odrv2_port, odrv3_port };

int com_init();
int com_read_ln(HANDLE handle, char *c, int n_bytes_to_read);
int com_write_ln(HANDLE handle, char c[]);