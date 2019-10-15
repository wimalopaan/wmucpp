#pragma once

#ifndef NDEBUG

namespace Trace {
    char file[2];
    unsigned int line;
    
    inline void trace(const char* name, unsigned int l) {
        file[0] = name[0];
        file[1] = name[1];
        line = l;
    }
}

#define TRACE Trace::trace(TRACE_FILENAME, __LINE__)

#else 
# define TRACE
#endif
