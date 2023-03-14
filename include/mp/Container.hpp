#ifndef Containers_h_
#define Containers_h_



#ifdef MP_USE_ETL
#include <etl/vector.h>
namespace mp {
    template <typename T>
    using vector = etl::vector<T, 150>;
}
#else
#include <vector>
namespace mp { 
    template <typename T>
    using vector = std::vector<T>;
}
#endif

#endif


