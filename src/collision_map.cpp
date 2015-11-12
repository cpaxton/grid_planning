#include <grid/collision_map.h>

namespace grid {

  std::size_t CollisionMapHash::operator()(Position_t const& Position_t)  {
    std::size_t hash = 0;
    for (double &q: Position_t) {
      hash ^= (int)q;
    }

    return hash;
  }
}
