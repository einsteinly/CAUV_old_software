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

class GroupListTest < Test::Unit::TestCase

  def setup
    @groups = Ssrc::Spread::GroupList.new
  end

  def teardown
    @groups = nil
  end

  def test_add
    assert_equal(0, @groups.size)
    @groups.add("foo")
    assert_equal(1, @groups.size)
    assert_equal("foo", @groups.group(0))
    @groups.add("bar")
    assert_equal(2, @groups.size)
    assert_equal("bar", @groups.group(1))
  end

  def test_add_group_list
    @groups.add("foo")
    @groups.add("bar")

    g = Ssrc::Spread::GroupList.new

    g.add("foobar")
    g.add("blah")
    g.add("blargh")

    @groups.add(g)

    assert_equal(5, @groups.size)
    assert_equal("blargh", @groups.group(4))
    assert_equal("bar", @groups.group(1))

    assert_not_equal(g, @groups)

    @groups.clear
    @groups.add(g)

    assert_equal(g, @groups)
  end

  def test_copy
    g2 = Ssrc::Spread::GroupList.new
    g2.add("foo")
    assert_not_equal(g2, @groups)
    @groups.copy(g2)
    assert_equal("foo", @groups.group(0))
    assert_equal(g2, @groups)
  end

  def test_split_private_group
    private_name, proc_name = Ssrc::Spread.split_private_group("#foo#bar")
    assert_equal("foo", private_name)
    assert_equal("bar", proc_name)
  end

end
