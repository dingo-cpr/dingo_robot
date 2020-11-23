#include "dingo_base/value_aggregator.h"
#include <algorithm>
#include <list>
#include <vector>

using namespace std;
using namespace dingo_base;

ValueAggregator::ValueAggregator(const size_t max_samples) :
    samples_(),
    max_samples_(max_samples),
    average_(0.0f),
    median_(0.0f)
{

}

void ValueAggregator::add_sample(const float x)
{
  samples_.push_back(x);
  if (samples_.size() > max_samples_)
  {
    samples_.pop_front();
  }

  average_ = calculate_average();
  median_ = calculate_median();
}

const size_t ValueAggregator::current_samples()
{
  return samples_.size();
}

const float ValueAggregator::latest()
{
  if (samples_.size() > 0)
    return samples_.back();
  else
    return 0.0f;
}

const float ValueAggregator::average()
{
  return average_;
}

const float ValueAggregator::median()
{
  return median_;
}

float ValueAggregator::calculate_average()
{
  float total = 0.0f;
  std::for_each(samples_.begin(), samples_.end(), [&](const float &x) {
    total += x;
  });
  return total / samples_.size();
}

float ValueAggregator::calculate_median()
{
  if (samples_.size() > 0)
  {
    std::vector<float> cpy(samples_.size());
    std::for_each(samples_.begin(), samples_.end(), [&](const float &x){
      cpy.push_back(x);
    });
    std::sort(cpy.begin(), cpy.end());
    return cpy[cpy.size()/2];
  }
  else
    return 0.0;
}