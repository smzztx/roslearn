#include "simple_filter_tutorial/increment.h"
#include "pluginlib/class_list_macros.h"
using namespace filters;

template <typename T>
IncrementFilter<T>::IncrementFilter()
{}

template <typename T>
bool IncrementFilter<T>::configure()
{  return true;}

template <typename T>
IncrementFilter<T>::~IncrementFilter()
{}

template <typename T>
bool IncrementFilter<T>::update(const T & data_in, T& data_out)
{
  data_out = data_in + 1;
  return true;
};

PLUGINLIB_REGISTER_CLASS(IncrementFilterInt, filters::IncrementFilter<int>, filters::FilterBase<int>)