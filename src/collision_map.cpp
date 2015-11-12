#include <grid/collision_map.h>

namespace grid {

  std::size_t CollisionMapHash::operator()(Position_t const& pos) const {
    std::size_t hash = 0;
    for (const double &q: pos) {
      hash ^= (int)q;
    }

    return hash;
  }

  void CollisionMap::reset() {
    
  }
}
