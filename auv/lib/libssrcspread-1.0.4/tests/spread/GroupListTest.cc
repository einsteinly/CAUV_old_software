/*
 *
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

#include <tests/TestCommon.h>

using namespace std::rel_ops;

class GroupListTest : public TestCase {

  GroupList *groups;

public:

  virtual void setUp() {
    groups = new GroupList();
  }

  virtual void tearDown() {
    delete groups;
    groups = 0;
  }

  void test_add() {
    CPPUNIT_ASSERT_EQUAL(0u, groups->size());
    groups->add("foo");
    CPPUNIT_ASSERT_EQUAL(1u, groups->size());
    CPPUNIT_ASSERT((*groups)[0] == "foo");
    groups->add("bar");
    CPPUNIT_ASSERT_EQUAL(2u, groups->size());
    CPPUNIT_ASSERT(groups->group(1) == "bar");
  }

  void test_add_group_list() {
    groups->add("foo");
    groups->add("bar");

    GroupList g;

    g.add("foobar");
    g.add("blah");
    g.add("blargh");

    groups->add(g);

    CPPUNIT_ASSERT_EQUAL(5u, groups->size());
    CPPUNIT_ASSERT(groups->group(4) == "blargh" && groups->group(1) == "bar");

    CPPUNIT_ASSERT(g != *groups);

    groups->clear();
    groups->add(g);

    CPPUNIT_ASSERT(g == *groups);
  }

  void test_split_private_group() {
    std::pair<string,string> name = split_private_group("#foo#bar");
    CPPUNIT_ASSERT("foo" == name.first);
    CPPUNIT_ASSERT("bar" == name.second);
  }

  CPPUNIT_TEST_SUITE(GroupListTest);
  CPPUNIT_TEST(test_add);
  CPPUNIT_TEST(test_add_group_list);
  CPPUNIT_TEST(test_split_private_group);
  CPPUNIT_TEST_SUITE_END();
};

CPPUNIT_TEST_SUITE_REGISTRATION(GroupListTest);
LIBSSRCSPREAD_TEST_MAIN()
