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

class MessageTest < Test::Unit::TestCase

  def setup
    @message = Ssrc::Spread::Message.new
  end

  def teardown
    @message = nil
  end

  def test_capacity
    assert_equal(Ssrc::Spread::Message::DefaultCapacity, @message.capacity)
  end

  def test_self_discard
    assert(!@message.is_self_discard)
    @message.set_self_discard
    assert(@message.is_self_discard)
    @message.set_self_discard(false)
    assert(!@message.is_self_discard)
  end

  def test_service
    service = 
      Ssrc::Spread::Message::Reliable | Ssrc::Spread::Message::SelfDiscard
    @message.set_service(service)
    assert_equal(service, @message.service)
    assert(@message.is_self_discard)
    assert(@message.is_reliable)
    assert(!@message.is_fifo)
  end

  def test_write
    @message.writen("test_scatter_send", 17)
    assert_equal(17, @message.size)
    @message.clear
    @message.write("test_scatter_send2")
    # Includes null terminator
    assert_equal(18 + 1, @message.size)
  end

  def test_read
    @message.write("foobar")
    @message.rewind
    foo = @message.read(3)
    assert_equal("foo", foo)
    foo = "   "
    @message.readn(foo, 3)
    assert_equal("bar", foo)
  end

end
