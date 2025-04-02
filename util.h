#ifndef UTIL_H
#define UTIL_H

#include <stdlib.h>

typedef enum {
    UINFO,
    UWARN,
    UFATL,
} UtilLogLvl;

#ifndef UASSERT
#include <assert.h>
#include <stdlib.h>
#define UASSERT(c) \
    do { \
        if (!c) { \
            ulog(UFATL, "%s:%d %s assertion failed '%s'", \
                     __FILE__, __LINE__, __func__, #c); \
            abort(); \
        } \
    } while (0)
#endif

#define UNIMPL \
    do { \
        ulog(UWARN, "%s:%d %s unimplemented", __FILE__, __LINE__, __func__); \
    } while (0)

#define UNUSED(x) ((void)(x))

void ulog(UtilLogLvl lvl, const char *fmt, ...);
void *umalloc(size_t size);

#endif // UH


#ifdef UIMPL

#include <stdio.h>
#include <stdarg.h>

void
ulog(UtilLogLvl lvl, const char *fmt, ...)
{
    const char *tag;
    switch (lvl) {
    case UINFO: tag = "INFO"; break;
    case UWARN: tag = "WARN"; break;
    case UFATL: tag = "FATL"; break;
    default: break;
    }
    fprintf(stdout, "[%s] ", tag);

    va_list v;
    va_start(v, fmt);
    vfprintf(stdout, fmt, v);
    va_end(v);
}

void *
umalloc(size_t size)
{
    void *p = malloc(size);
    UASSERT(p);
    return p;
}

#endif // UTIL_IMPL

