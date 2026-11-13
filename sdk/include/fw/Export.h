#pragma once
#include <QtCore/qglobal.h>
#ifdef FW_SDK_BUILD
#define FW_API Q_DECL_EXPORT
#else
#define FW_API Q_DECL_IMPORT
#endif
