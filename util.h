#ifndef UTIL_H
#define UTIL_H

#include <stdlib.h>

typedef enum {
    UNONE,
    UINFO,
    UWARN,
    UFATL,
} UtilLogLvl;

#define ARRLEN(arr) (sizeof(arr) / sizeof(*arr))

#ifndef UASSERT
#include <assert.h>
#include <stdlib.h>
#define UASSERT(c) \
    do { \
        if (!(c)) { \
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

#endif // UTIL_H


#ifdef UTIL_IMPL

#include <stdio.h>
#include <stdarg.h>

void
ulog(UtilLogLvl lvl, const char *fmt, ...)
{
    const char *tag;
    switch (lvl) {
    case UNONE: break;
    case UINFO: fprintf(stdout, "[INFO] "); break;
    case UWARN: fprintf(stdout, "[WARN] "); break;
    case UFATL: fprintf(stdout, "[FATL] "); break;
    default: break;
    }

    va_list v;
    va_start(v, fmt);
    vfprintf(stdout, fmt, v);
    va_end(v);

    fputc('\n', stdout);
}

void *
umalloc(size_t size)
{
    void *p = malloc(size);
    UASSERT(p);
    return p;
}

#endif // UTIL_IMPL

