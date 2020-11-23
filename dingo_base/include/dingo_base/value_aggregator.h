/**
 *  \file       value_aggregator.cpp
 *  \brief      Utility class for aggregating multiple sensor readings (e.g. battery voltage)
 *              and presenting them as an average, median, or instanteous reading
 *  \copyright  Copyright (c) 2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */


#ifndef VALUE_AGGREGATOR_H_INCLUDED
#define VALUE_AGGREGATOR_H_INCLUDED

#include <list>

namespace dingo_base
{
/**
 * Keeps a rolling-window of n samples, allows querying the latest single value
 * as well as the average and median.
 * 
 * Calling add_sample will force the average and median to be recalculated, making
 * the query methods O(1)
 **/
class ValueAggregator
{
public:
  /**
   * Constructor, specifies the maximum number of samples we store
   **/
  ValueAggregator(const size_t max_samples);

  /**
   * Add a new sample. The oldest sample may be dropped.
   * 
   * Adding a new sample will force the average and median to be recalculated
   **/
  void add_sample(const float x);

  /**
   * How many samples are currently stored?
   **/
  const size_t current_samples();

  /**
   * Returns the latest single sample recorded
   **/
  const float latest();

  /**
   * Returns the average of all stored samples
   **/
  const float average();

  /**
   * Returns the median of all stored samples
   **/
  const float median();

private:
  std::list<float> samples_;
  const int max_samples_;

  float average_;
  float median_;

  float calculate_average();
  float calculate_median();
};

}  // namespace dingo_base
#endif  //VALUE_AGGREGATOR_H_INCLUDED