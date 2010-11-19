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
#include <utility>
#include <cstdlib>

using namespace std::rel_ops;

const char *TestGroupName = "MailboxTestGroup";
const char *TestGroupName2 = "MailboxTestGroup2";

class MailboxTest : public TestCase {

  Mailbox *mbox;
  Mailbox *rmbox;
  Message *message;

public:

  virtual void setUp() {
    try {
      mbox  = new Mailbox(LIBSSRCSPREAD_TEST_DAEMON);
      rmbox = new Mailbox(LIBSSRCSPREAD_TEST_DAEMON, "", false);
    } catch(const Error & e) {
      e.print();
      throw e;
    }
    message = new Message();
  }

  virtual void tearDown() {
    delete mbox;
    delete rmbox;
    delete message;
    mbox = 0;
    message = 0;
  }

  void join() {
    mbox->join(TestGroupName);
    mbox->join(TestGroupName2);
  }

  void leave() {
    mbox->leave(TestGroupName);
    mbox->leave(TestGroupName2);
  }

  void test_join_and_leave() {
    CPPUNIT_ASSERT_NO_THROW(join());
    CPPUNIT_ASSERT_NO_THROW(leave());
  }

  void test_send() {
    join();
    message->write("test_send", 9);
    CPPUNIT_ASSERT_NO_THROW(mbox->send(*message, TestGroupName));
    message->clear();
    message->write("test_send2", 10);
    CPPUNIT_ASSERT_NO_THROW(mbox->send(*message, TestGroupName));
    leave();
  }

  void test_scatter_send() {
    ScatterMessage sm;
    Message m;

    join();
    message->write("test_scatter_send", 17);
    m.write("test_scatter_send2", 18);
    CPPUNIT_ASSERT(sm.add(*message));
    CPPUNIT_ASSERT(sm.add(m));
    CPPUNIT_ASSERT_NO_THROW(mbox->send(sm, TestGroupName));

    mbox->clear_groups();
    mbox->add_group(TestGroupName);
    mbox->clear_message_parts();
    CPPUNIT_ASSERT(mbox->add_message_part("test_scatter_send3", 18));
    CPPUNIT_ASSERT(mbox->add_message_part("test_scatter_send4", 18));
    CPPUNIT_ASSERT_NO_THROW(mbox->send());
    leave();
  }

  void test_group_send() {
    GroupList groups;

    groups.add(TestGroupName);
    groups.add(TestGroupName2);

    join();
    message->write("test_group_send", 15);
    CPPUNIT_ASSERT_NO_THROW(mbox->send(*message, groups));
    message->clear();
    message->write("test_group_send2", 16);
    CPPUNIT_ASSERT_NO_THROW(mbox->send(*message, groups));
    leave();
  }

  void test_scatter_group_send() {
    GroupList groups;
    ScatterMessage sm;
    Message m;

    groups.add(TestGroupName);
    groups.add(TestGroupName2);

    join();
    message->write("test_scatter_group_send", 23);
    m.write("test_scatter_group_send2", 24);
    sm.add(*message);
    sm.add(m);
    CPPUNIT_ASSERT_NO_THROW(mbox->send(sm, groups));
    leave();
  }

  void test_receive_message() {
    const char *data = "foobar";
    unsigned int data_len = strlen(data) + 1;
    unsigned int bytes;
    GroupList groups;
    Message padding;

    mbox->clear_groups();
    mbox->clear_message_parts();
    mbox->add_group(rmbox->private_group());
    mbox->add_message_part(data, data_len);
    mbox->send();

    rmbox->clear_groups();
    rmbox->clear_message_parts();
    rmbox->add_message_part(*message);
    rmbox->add_message_part(padding);

    padding.resize(padding.capacity());
    CPPUNIT_ASSERT(padding.size() > 0);

    bytes = rmbox->receive();
    rmbox->copy_groups(groups);

    CPPUNIT_ASSERT_EQUAL(0u, padding.size());
    CPPUNIT_ASSERT_EQUAL(data_len, bytes);
    CPPUNIT_ASSERT_EQUAL(data_len, message->size());
    CPPUNIT_ASSERT_EQUAL(0, std::memcmp(data, &((*message)[0]), data_len));
    CPPUNIT_ASSERT(rmbox->private_group() == groups[0]);
    CPPUNIT_ASSERT(mbox->private_group() == message->sender());
  }

  void test_receive_message_buffer_too_short() {
    const unsigned int capacity = 131072;
    Message short_buffer(0);

    message->resize(capacity);

    for(unsigned int i = 0; i < capacity; i+=sizeof(int)) {
      const int j = std::rand();
      message->write(&j, sizeof(j));
    }

    mbox->clear_message_parts();
    mbox->add_message_part(*message);
    CPPUNIT_ASSERT_NO_THROW(mbox->send(rmbox->private_group()));

    CPPUNIT_ASSERT_EQUAL(0u, short_buffer.size());

    const unsigned int bytes = rmbox->receive(short_buffer);

    CPPUNIT_ASSERT_EQUAL(capacity, bytes);
    CPPUNIT_ASSERT_EQUAL(capacity, short_buffer.size());
    CPPUNIT_ASSERT_EQUAL(0, std::memcmp(&((*message)[0]), &short_buffer[0],
                                        capacity));
    CPPUNIT_ASSERT(rmbox->private_group() == rmbox->group(0));
    CPPUNIT_ASSERT(mbox->private_group() == short_buffer.sender());
  }

  void test_receive_membership_message() {
    join();

    // mbox join message
    GroupList groups;
    mbox->receive(*message, groups);

    CPPUNIT_ASSERT(message->is_membership());
    CPPUNIT_ASSERT(TestGroupName == message->sender());

#ifdef LIBSSRCSPREAD_ENABLE_MEMBERSHIP_INFO
    MembershipInfo info;

    message->get_membership_info(info);

    CPPUNIT_ASSERT(info.is_regular_membership());
    CPPUNIT_ASSERT(info.caused_by_join());
    CPPUNIT_ASSERT(mbox->private_group() == info.changed_member());

    GroupList mbox_group, local_members, non_local_members, all_members;

    mbox_group.add(mbox->private_group());
    info.get_local_members(local_members);
    info.get_non_local_members(non_local_members);
    info.get_all_members(all_members);

    CPPUNIT_ASSERT(mbox_group == local_members);
    CPPUNIT_ASSERT(mbox_group != non_local_members);
    CPPUNIT_ASSERT_EQUAL(0u, non_local_members.size());
    CPPUNIT_ASSERT(mbox_group == all_members);
    CPPUNIT_ASSERT(groups == all_members);
#endif

    // mbox join message
    mbox->receive(*message);
    CPPUNIT_ASSERT(message->is_membership());
    CPPUNIT_ASSERT(TestGroupName2 == message->sender());

    // rmbox join message
    rmbox->join(TestGroupName2);

    mbox->receive(*message);
    CPPUNIT_ASSERT(message->is_membership());
    CPPUNIT_ASSERT(TestGroupName2 == message->sender());

    // rmbox leave message
    rmbox->leave(TestGroupName2);

    mbox->receive(*message);
    CPPUNIT_ASSERT(message->is_membership());

#ifdef LIBSSRCSPREAD_ENABLE_MEMBERSHIP_INFO
    message->get_membership_info(info);

    CPPUNIT_ASSERT(info.caused_by_leave());
#endif

    CPPUNIT_ASSERT(TestGroupName2 == message->sender());

    leave();
  }

  void test_self_leave_message() {
#ifdef LIBSSRCSPREAD_ENABLE_MEMBERSHIP_INFO
    mbox->join(TestGroupName);
    mbox->receive(*message);
    mbox->leave(TestGroupName);
    mbox->receive(*message);

    MembershipInfo info;

    message->get_membership_info(info);

    CPPUNIT_ASSERT(info.caused_by_leave());
    CPPUNIT_ASSERT(info.is_self_leave());
#endif
  }

  CPPUNIT_TEST_SUITE(MailboxTest);
  CPPUNIT_TEST(test_join_and_leave);
  CPPUNIT_TEST(test_send);
  CPPUNIT_TEST(test_scatter_send);
  CPPUNIT_TEST(test_group_send);
  CPPUNIT_TEST(test_scatter_group_send);
  CPPUNIT_TEST(test_receive_message);
  CPPUNIT_TEST(test_receive_message_buffer_too_short);
  CPPUNIT_TEST(test_receive_membership_message);
  CPPUNIT_TEST(test_self_leave_message);
  CPPUNIT_TEST_SUITE_END();
};

CPPUNIT_TEST_SUITE_REGISTRATION(MailboxTest);
LIBSSRCSPREAD_TEST_MAIN()
