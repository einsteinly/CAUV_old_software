/*
 *
 * Copyright 2003-2005 Daniel F. Savarese
 * Copyright 2006 Savarese Software Research Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.savarese.com/software/ApacheLicense-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __SSRCSPREAD_TEST_COMMON_H
#define __SSRCSPREAD_TEST_COMMON_H

#include <ssrc/spread.h>

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/TextTestRunner.h>

using namespace NS_SSRC_SPREAD;
using CppUnit::TestCase;
using CppUnit::TestFactoryRegistry;
using CppUnit::TextTestRunner;

#define LIBSSRCSPREAD_TEST_MAIN(init_hook) \
int main(int argc, char **argv) { \
  TextTestRunner runner; \
  TestFactoryRegistry & registry = TestFactoryRegistry::getRegistry();\
\
  init_hook; \
\
  runner.addTest(registry.makeTest());\
\
  return (runner.run() ? 0 : -1);\
}

#endif
