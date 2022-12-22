#pragma once

#define INLINE 

#include <stdint.h>

enum Text {text1 = 0, text2, text3, _ntext};
typedef enum Text text_t;

enum Language {de = 0, en, _nlang};
typedef enum Language lang_t;

#ifdef INLINE
// nur wegen inline tr() hier global, andernfalls können diese auch TU-global definiert bleiben 
extern const char* const data[_nlang][_ntext];
extern lang_t lang;

inline const char* tr(const text_t id) {
    return data[lang][id];
}
#else
const char* tr(const text_t id);
#endif

void setLanguage(const lang_t lang);
