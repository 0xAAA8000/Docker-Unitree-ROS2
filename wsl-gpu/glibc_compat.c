/*
 * Windows 側から提供される WSL 用 GPU ドライバ (Qualcomm Adreno 等) が
 * glibc 2.38 / libstdc++ (GCC 13) で追加されたシンボルを要求するため、
 * Ubuntu 22.04 (glibc 2.35) 上で代替実装を提供する互換ライブラリ。
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

long __isoc23_strtol(const char *s, char **e, int b) { return strtol(s, e, b); }
long long __isoc23_strtoll(const char *s, char **e, int b) { return strtoll(s, e, b); }
unsigned long __isoc23_strtoul(const char *s, char **e, int b) { return strtoul(s, e, b); }
unsigned long long __isoc23_strtoull(const char *s, char **e, int b) { return strtoull(s, e, b); }

int __isoc23_sscanf(const char *s, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int r = vsscanf(s, fmt, ap);
    va_end(ap);
    return r;
}

int __isoc23_scanf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int r = vscanf(fmt, ap);
    va_end(ap);
    return r;
}

/* std::ios_base_library_init() (GLIBCXX_3.4.32)。iostream の初期化は旧 libstdc++ 側で行われるため何もしない */
void _ZSt21ios_base_library_initv(void) {}
