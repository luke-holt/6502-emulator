#include <string.h>

#define UTIL_IMPL
#include "util.h"

int
main(int argc, char *argv[])
{
    const char *msg = "test";
    ulog(UINFO, "msg <- \"%s\"", msg);
    ulog(UINFO, "len <- %d", strlen(msg));
    return 0;
}
