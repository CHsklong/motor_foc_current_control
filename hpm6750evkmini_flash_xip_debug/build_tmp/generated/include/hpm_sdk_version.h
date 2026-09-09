/*
 * Copyright (c) 2022 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_SDK_VERSION_H
#define HPM_SDK_VERSION_H

/* #undef SDK_VERSION_CODE */
#define SDK_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))

#define SDKVERSION          0x10C0100
#define SDK_VERSION_NUMBER  0x10C01
#define SDK_VERSION_MAJOR   1
#define SDK_VERSION_MINOR   12
#define SDK_PATCHLEVEL      1
#define SDK_VERSION_STRING  "1.12.1"

#define BUILD_VERSION          


#endif /* HPM_SDK_VERSION_H */
