#ifndef UTIL_H
#define UTIL_H

typedef enum {
    UTIL_INFO,
    UTIL_WARN,
    UTIL_FATL,
} UtilLogLvl;

#define UNIMPL \
    do { \
        util_log(UTIL_WARN, "%s:%d %s unimplemented", __FILE__, __LINE__, __func__); \
    } while (0)

#define UNUSED(x) ((void)(x))

void util_log(UtilLogLvl lvl, const char *fmt, ...);

#endif // UTIL_H


#ifdef UTIL_IMPL

#include <stdio.h>
#include <stdarg.h>

void
util_log(UtilLogLvl lvl, const char *fmt, ...)
{
    const char *tag;
    switch (lvl) {
    case UTIL_INFO: tag = "INFO"; break;
    case UTIL_WARN: tag = "WARN"; break;
    case UTIL_FATL: tag = "FATL"; break;
    default: break;
    }
    fprintf(stdout, "[%s] ", tag);

    va_list v;
    va_start(v, fmt);
    vfprintf(stdout, fmt, v);
    va_end(v);
}


#endif // UTIL_IMPL

