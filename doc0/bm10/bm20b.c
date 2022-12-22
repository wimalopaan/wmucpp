#include "bm20.h"

#ifndef INLINE
static
#endif 
const char* const data[_nlang][_ntext] = {
    [de] = {
        [text1] = "bla",  
        [text2] = "blubb",  
    },
    [en] = {
        [text1] = "enbla",  
        [text2] = "enblubb",  
    }
};

#ifndef INLINE
static
#endif 
lang_t lang = de;

void setLanguage(const lang_t l) {
    lang = l;
}

#ifndef INLINE
const char* tr(const text_t id) {
    return data[lang][id];
}
#endif
