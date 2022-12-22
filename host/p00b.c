#include "p00.h"

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

lang_t lang = de;

void setLanguage(const lang_t l) {
    lang = l;
}
