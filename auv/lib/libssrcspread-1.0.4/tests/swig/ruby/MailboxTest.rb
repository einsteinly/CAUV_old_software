#!/usr/bin/env ruby
#--
#
# Copyright 2006 Savarese Software Research Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.savarese.com/software/ApacheLicense-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#++

require 'test/unit'
require 'ssrc/spread'

class MailboxTest < Test::Unit::TestCase

  @@TestDaemon     = ENV["LIBSSRCSPREAD_TEST_DAEMON"].gsub('"','')
  @@TestGroupName  = "MailboxTestGroup"
  @@TestGroupName2 = "MailboxTestGroup2"

  def setup
    begin
      @mbox  = Ssrc::Spread::Mailbox.new(@@TestDaemon, "", true, 0)
      @rmbox = Ssrc::Spread::Mailbox.new(@@TestDaemon)
    rescue Ssrc::Spread::Error => error
      error.print
      raise error
    end

    @message = Ssrc::Spread::Message.new
  end

  def teardown
    @mbox = nil
    @rmbox = nil
  end

  def join
    @mbox.join(@@TestGroupName)
    @mbox.join(@@TestGroupName2)
  end

  def leave
    @mbox.leave(@@TestGroupName)
    @mbox.leave(@@TestGroupName2)
  end

  def test_join_and_leave
    assert_nothing_raised(Ssrc::Spread::Error) { join }
    assert_nothing_raised(Ssrc::Spread::Error) { leave }
  end

  def test_send
    join

    test = lambda do |s,g|
      assert_nothing_raised(Ssrc::Spread::Error) do
        @message.clear
        @message.write(s)
        @mbox.send(@message, g)
      end
    end

    test.call("test_send", @@TestGroupName)
    test.call("test_send2", @@TestGroupName2)

    leave
  end

  def test_scatter_send
    sm = Ssrc::Spread::ScatterMessage.new
    m  = Ssrc::Spread::Message.new

    join

    @message.write("test_scatter_send")
    m.write("test_scatter_send2")

    assert(sm.add(@message))
    assert(sm.add(m))
    assert_nothing_raised(Ssrc::Spread::Error) do
      @mbox.send(sm, @@TestGroupName)
    end

    @mbox.clear_groups
    @mbox.add_group(@@TestGroupName)
    @mbox.clear_message_parts

    # If you add a straight data message part, you must make sure
    # it does not get garbage-collected before the send.
    s1 = "test_scatter_send3"
    assert(@mbox.add_message_part(s1))
    s2 = "test_scatter_send4"
    assert(@mbox.add_message_part(s2))
    assert_nothing_raised(Ssrc::Spread::Error) { @mbox.send }

    leave
  end

  def test_group_send
    groups = Ssrc::Spread::GroupList.new

    groups.add(@@TestGroupName)
    groups.add(@@TestGroupName2)

    join

    test = lambda do |s|
      assert_nothing_raised(Ssrc::Spread::Error) do
        @message.clear
        @message.write(s)
        @mbox.send(@message, groups)
      end
    end

    test.call("test_group_send")
    test.call("test_group_send2")

    leave
  end

  def test_scatter_group_send
    groups = Ssrc::Spread::GroupList.new
    sm = Ssrc::Spread::ScatterMessage.new
    m = Ssrc::Spread::Message.new

    groups.add(@@TestGroupName)
    groups.add(@@TestGroupName2)

    join
    @message.write("test_scatter_group_send")
    m.write("test_scatter_group_send2")
    sm.add(@message)
    sm.add(m)
    assert_nothing_raised(Ssrc::Spread::Error) { @mbox.send(sm, groups) }
    leave
  end

  def test_receive_message
    data = "foobar"

    groups = Ssrc::Spread::GroupList.new
    padding = Ssrc::Spread::Message.new

    @mbox.clear_groups
    @mbox.clear_message_parts
    @mbox.add_group(@rmbox.private_group)
    @mbox.add_message_part(data)
    @mbox.send

    @rmbox.clear_groups
    @rmbox.clear_message_parts
    @rmbox.add_message_part(@message)
    @rmbox.add_message_part(padding)

    padding.resize(padding.capacity)
    assert(padding.size > 0)

    bytes = @rmbox.receive
    @rmbox.copy_groups(groups)

    assert_equal(0, padding.size)
    assert_equal(data.length + 1, bytes)
    assert_equal(data.length + 1, @message.size)
    assert_equal(data, @message.read(data.length + 1))
    assert_equal(@rmbox.private_group, groups.group(0))
    assert_equal(@mbox.private_group, @message.sender)
  end

  if Ssrc::Spread::const_defined? "MembershipInfo":
      def test_receive_membership_message
        info = Ssrc::Spread::MembershipInfo.new

        join

        # mbox join message
        groups = Ssrc::Spread::GroupList.new
        @mbox.receive(@message, groups)

        assert(@message.is_membership)
        assert_equal(@@TestGroupName, @message.sender)

        @message.get_membership_info(info)

        assert(info.caused_by_join)
        assert_equal(@mbox.private_group, info.changed_member)

        mbox_group = Ssrc::Spread::GroupList.new
        local_members = Ssrc::Spread::GroupList.new
        all_members = Ssrc::Spread::GroupList.new

        mbox_group.add(@mbox.private_group)
        info.get_local_members(local_members)
        info.get_all_members(all_members)

        assert_equal(mbox_group, local_members)
        assert_equal(mbox_group, all_members)
        assert_equal(groups, all_members)

        # mbox join message
        @mbox.receive(@message)
        assert(@message.is_membership)
        assert_equal(@@TestGroupName2, @message.sender)

        # rmbox join message
        @rmbox.join(@@TestGroupName2)

        @mbox.receive(@message)
        assert(@message.is_membership)
        assert_equal(@@TestGroupName2, @message.sender)

        # rmbox leave message
        @rmbox.leave(@@TestGroupName2)

        @mbox.receive(@message)
        assert(@message.is_membership)

        @message.get_membership_info(info)

        assert(info.caused_by_leave)
        assert_equal(@@TestGroupName2, @message.sender)

        leave
      end

      def test_self_leave_message
        @mbox.join(@@TestGroupName)
        @mbox.receive(@message)
        @mbox.leave(@@TestGroupName)
        @mbox.receive(@message)

        info = Ssrc::Spread::MembershipInfo.new

        @message.get_membership_info(info)

        assert(info.caused_by_leave)
        assert(info.is_self_leave)
      end
    end

  def test_error
    begin
      @mbox.join("####")
      assert(false)
    rescue Ssrc::Spread::Error
      assert_equal(Ssrc::Spread::Error::IllegalGroup, $!.error)
    end
  end

end
