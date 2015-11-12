#ifndef _COLLISION_MAP
#define _COLLISION_MAP

#include <vector>
#include <unordered_map>

using namespace grid {

  typedef Position_t std::vector<double>; // store a set of joint positions

  /**
   * CollisionMap
   * Enable fast trajectory search by hashing collision detection calls within some scaling factor.
   *
   */
  class CollisionMap {
    private:
      std::unordered_map<Position_t,bool,CollisionMapHash> map;

  };

  template<>
  class CollisionMapHash<Position_t>
  {
  public:
      std::size_t operator()(Position_t const& Position_t) const;
  };

}

using namespace std {

}

#endif
