// SPDX-License-Identifier: MIT
// Dependency-free PNG writer (stored deflate blocks). Fine for screenshots; files are uncompressed-sized.
#pragma once

#include <stdint.h>

bool WritePng( const char* path, const uint8_t* rgb, int width, int height );
