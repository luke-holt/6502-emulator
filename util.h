#ifndef UTIL_H
#define UTIL_H

#include <stdlib.h>
#include <string.h>

typedef enum {
    UNONE,
    UINFO,
    UWARN,
    UFATL,
} UtilLogLvl;

void ulog(UtilLogLvl lvl, const char *fmt, ...);
void *umalloc(size_t size);
void *urealloc(void *p, size_t size);

#define ARRLEN(arr) (sizeof(arr) / sizeof(*arr))

#define da_init(da) \
    do { \
        (da)->capacity = 256; \
        (da)->items = umalloc(sizeof(*(da)->items)*(da)->capacity); \
    } while (0)
#define da_delete(da) \
    do { \
        UASSERT(da); \
        free((da)->items); \
        (da)->items = NULL; \
        (da)->count = 0; \
        (da)->capacity = 0; \
    } while (0)
#define da_resize(da, cap) \
    do { \
        (da)->capacity = (cap); \
        (da)->items = urealloc((da)->items, sizeof(*(da)->items)*(da)->capacity); \
    } while (0)
#define da_append(da, item) \
    do { \
        if ((da)->count >= (da)->capacity) da_resize((da), (da)->capacity*2); \
        (da)->items[(da)->count++] = (item); \
    } while (0)
#define da_append_many(da, list, len) \
    do { \
        while ((da)->capacity <= ((da)->count + (len))) (da)->capacity *= 2; \
        da_resize((da), (da)->capacity); \
        memcpy(&(da)->items[(da)->count], (list), (len)*sizeof(*(da)->items)); \
    } while (0)

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

#endif // UTIL_H


#ifdef UTIL_IMPL

#include <stdio.h>
#include <stdarg.h>

void
ulog(UtilLogLvl lvl, const char *fmt, ...)
{
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

void *
urealloc(void *p, size_t size)
{
    p = realloc(p, size);
    UASSERT(p);
    return p;
}

#endif // UTIL_IMPL

