/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "container/pgmstring.h"

#define CAT2(X, Y) X ## Y
#define CAT(X, Y) CAT2(X, Y)

#ifndef SIMPLETESTPREFIX
# define SIMPLETESTPREFIX x
#endif

#define SIMPLETEST(name) static SimpleTest::SimpleTestCase CAT(SIMPLETESTPREFIX, CAT(SimpleTestCaseOnLine, __LINE__)) = \
    SimpleTest::SimpleTestCase(name, __FILE__, __LINE__) = []

namespace SimpleTest {

class SimpleTestCase final {
public:
    SimpleTestCase(const char* name, const char* file, uint16_t line) : mName(name), mFile(file), mLine(line) {}

    template <typename T>
    SimpleTestCase& operator=(T lambda) {
        std::cout << "Test(" << mName << "," << mFile << "," << mLine << ")";
        ++testCount();
        if (lambda()) {
            std::cout << " ok " << "(" << testCount() << "/" << failureCount() << ")" << std::endl;
        }
        else {
            ++failureCount();
            std::cout << " failed " << "(" << testCount() << "/" << failureCount() << ")" << std::endl;
        }
        return *this;
    }
private:
    const char* mName{};
    const char* mFile{};
    uint16_t    mLine = 0;

    static size_t& testCount() {
        static size_t mTestCount;
        return mTestCount;
    }

    static size_t& failureCount() {
        static size_t mFailureCount;
        return mFailureCount;
    }
};


}

