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

class ErrorTest < Test::Unit::TestCase

  def test_error
    error = Ssrc::Spread::Error.new(Ssrc::Spread::Error::ConnectionClosed)
    assert_equal(Ssrc::Spread::Error::ConnectionClosed, error.error)
  end

  def test_buffer_size_error
    error =
      Ssrc::Spread::BufferSizeError.new(Ssrc::Spread::Error::BufferTooShort,10)
    assert_equal(Ssrc::Spread::Error::BufferTooShort, error.error)
    assert_equal(10, error.size)
  end

end
