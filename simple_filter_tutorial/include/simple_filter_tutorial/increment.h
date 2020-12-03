#ifndef FILTERS_INCREMENT_H
#define FILTERS_INCREMENT_H
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include "filters/filter_base.h"

namespace filters {

template <typename T>
class IncrementFilter: public FilterBase <T>
{
public:
  IncrementFilter();
  ~IncrementFilter();
  virtual bool configure();
  virtual bool update( const T & data_in, T& data_out);
};

}

#endif