#ifndef UTRACE__H
#define UTRACE__H

#ifdef __cplusplus
extern "C" {
#endif

void utraceTime(const unsigned long long t, const char* msg);
void utrace(const char* msg);

#ifdef __cplusplus
}
#endif

#endif // UTRACE__H
