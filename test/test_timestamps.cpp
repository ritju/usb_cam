// Copyright 2026 Evan Flynn
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cstdlib>
#include <ctime>

#include "usb_cam/timestamps.hpp"

using usb_cam::timestamps::pts_to_timespec;
using usb_cam::timestamps::wallclock_now;

TEST(test_timestamps, pts_microseconds)
{
  const AVRational tb{1, 1000000};
  // 2020-01-01 00:00:00 UTC = 1577836800 s
  const int64_t pts = 1577836800LL * 1000000LL + 250000LL;
  const timespec ts = pts_to_timespec(pts, tb);
  EXPECT_EQ(ts.tv_sec, 1577836800);
  EXPECT_EQ(ts.tv_nsec, 250000000L);
}

TEST(test_timestamps, pts_fractions)
{
  const AVRational tb{1, 30};
  const int64_t pts = 1577836800LL * 30 + 15;  // .5 s
  const timespec ts = pts_to_timespec(pts, tb);
  EXPECT_EQ(ts.tv_sec, 1577836800);
  EXPECT_GE(ts.tv_nsec, 499000000L);
  EXPECT_LE(ts.tv_nsec, 501000000L);
}

TEST(test_timestamps, nopts_falls_back_to_wallclock)
{
  const timespec now = wallclock_now();
  const timespec ts = pts_to_timespec(AV_NOPTS_VALUE, AVRational{1, 1000000});
  const int64_t dt_ns =
    (ts.tv_sec - now.tv_sec) * 1000000000LL + (ts.tv_nsec - now.tv_nsec);
  EXPECT_LT(std::llabs(dt_ns), 1000000000LL);
}

TEST(test_timestamps, invalid_timebase_falls_back)
{
  const timespec now = wallclock_now();
  const timespec ts = pts_to_timespec(12345, AVRational{0, 1});
  const int64_t dt_ns =
    (ts.tv_sec - now.tv_sec) * 1000000000LL + (ts.tv_nsec - now.tv_nsec);
  EXPECT_LT(std::llabs(dt_ns), 1000000000LL);
}

TEST(test_timestamps, pre2000_falls_back)
{
  const timespec now = wallclock_now();
  const timespec ts = pts_to_timespec(1000, AVRational{1, 1});
  const int64_t dt_ns =
    (ts.tv_sec - now.tv_sec) * 1000000000LL + (ts.tv_nsec - now.tv_nsec);
  EXPECT_LT(std::llabs(dt_ns), 1000000000LL);
}
