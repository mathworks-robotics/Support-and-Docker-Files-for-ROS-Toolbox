/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOTRANSPORT_UTIL_HPP
#define GAZEBOTRANSPORT_UTIL_HPP

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifdef BUILDING_LIBMWGAZEBOTRANSPORT

/* This header is being included by files inside this module */
#include "package.h"

# define GAZEBOTRANSPORT_EXPORT_CLASS    DLL_EXPORT_SYM
# define GAZEBOTRANSPORT_EXPORT_FRIEND   DLL_EXPORT_SYM
# define GAZEBOTRANSPORT_EXPORT_FCN      DLL_EXPORT_SYM
# define GAZEBOTRANSPORT_EXPORT_TEMPLATE DLL_EXPORT_TEMPLATE

# define GAZEBOTRANSPORT_EXPORT_VAR      extern
# define GAZEBOTRANSPORT_EXPORT_VAR_DEF  DLL_EXPORT_SYM

# define GAZEBOTRANSPORT_EXPORT          DLL_EXPORT_SYM

#else

/* This file is being include by external modules */

#ifdef _MSC_VER
#define DLL_EXPORT_TEMPLATE
#define DLL_IMPORT_SYM __declspec(dllimport)
#elif __GNUC__ >= 4
#define DLL_EXPORT_TEMPLATE __attribute__ ((visibility("default")))
#define DLL_IMPORT_SYM __attribute__ ((visibility("default")))
#else
#define DLL_EXPORT_TEMPLATE
#define DLL_IMPORT_SYM
#endif

# define GAZEBOTRANSPORT_EXPORT_CLASS              DLL_IMPORT_SYM
# define GAZEBOTRANSPORT_EXPORT_FRIEND             DLL_IMPORT_SYM
# define GAZEBOTRANSPORT_EXPORT_FCN                DLL_IMPORT_SYM
# define GAZEBOTRANSPORT_EXPORT_TEMPLATE           DLL_EXPORT_TEMPLATE
# define GAZEBOTRANSPORT_EXPORT_VAR                extern DLL_IMPORT_SYM
# define GAZEBOTRANSPORT_EXPORT                    DLL_IMPORT_SYM

#endif

#endif // GAZEBOTRANSPORT_UTIL_HPP
