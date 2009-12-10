/*
 * Copyright 2006,2007 Savarese Software Research Corporation
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
#include <cstdlib>

class MessageTest : public TestCase {

  Message *message;

public:

  virtual void setUp() {
    message = new Message();
  }

  virtual void tearDown() {
    delete message;
    message = 0;
  }

  void test_set_self_discard() {
    CPPUNIT_ASSERT(!message->is_self_discard());
    message->set_self_discard();
    CPPUNIT_ASSERT(message->is_self_discard());
    message->set_self_discard(false);
    CPPUNIT_ASSERT(!message->is_self_discard());
  }

  void test_service() {
    Message::service_type service = Message::ReliableSelfDiscard;
    message->set_service(service);
    CPPUNIT_ASSERT_EQUAL(service, message->service());
  }

  void test_write() {
    message->write("test_scatter_send", 17);
    CPPUNIT_ASSERT_EQUAL(0,
                         std::memcmp("test_scatter_send",&(*message)[0],17));
    CPPUNIT_ASSERT_EQUAL(17u, message->size());
    message->clear();
    message->write("test_scatter_send2", 18);
    CPPUNIT_ASSERT_EQUAL(18u, message->size());
    CPPUNIT_ASSERT_EQUAL(0,
                         std::memcmp("test_scatter_send2",&(*message)[0],18));
  }

  void test_resize() {
    Message m(16);
    m.write("test_scatter_send2", 18);
    CPPUNIT_ASSERT_EQUAL(18u, m.size());
    CPPUNIT_ASSERT_EQUAL(0, std::memcmp("test_scatter_send2", &m[0], 18));

    const unsigned int capacity = 1048576;
    Message m2(capacity);

    while(m2.offset() < capacity) {
      const int j = std::rand();
      m2.write(&j, sizeof(j));
    }

    // Test non-destructive resize.
    const unsigned int chunk_size = 131072;
    m.clear();

    CPPUNIT_ASSERT_EQUAL(0u, m.offset());

    m.write(&m2[0], chunk_size);

    CPPUNIT_ASSERT_EQUAL(chunk_size, m.offset());

    m.resize(m2.size() / 2);
    CPPUNIT_ASSERT_EQUAL(m2.size() / 2, m.size());
    CPPUNIT_ASSERT_EQUAL(m2.size() / 2, m.capacity());

    for(unsigned int i = chunk_size; i < capacity; i+=chunk_size) {
      m.write(&m2[i], chunk_size);
    }

    CPPUNIT_ASSERT_EQUAL(capacity, m2.size());
    CPPUNIT_ASSERT_EQUAL(m2.size(), m.size());
    CPPUNIT_ASSERT_EQUAL(0, std::memcmp(&m[0], &m2[0], capacity));
  }

  CPPUNIT_TEST_SUITE(MessageTest);
  CPPUNIT_TEST(test_set_self_discard);
  CPPUNIT_TEST(test_service);
  CPPUNIT_TEST(test_write);
  CPPUNIT_TEST(test_resize);
  CPPUNIT_TEST_SUITE_END();
};

CPPUNIT_TEST_SUITE_REGISTRATION(MessageTest);
LIBSSRCSPREAD_TEST_MAIN()
