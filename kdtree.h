#ifndef _KDTREE_H_
#define _KDTREE_H_

#include <cstdlib>
#include <vector>
#include <algorithm>
#include <mutex>
#include "boundingbox.h"
#include "photon.h"

// ==================================================================
// A hierarchical spatial data structure to store photons.  This data
// struture allows for fast nearby neighbor queries for use in photon
// mapping.

class KDTree {
 public:

  // ========================
  // CONSTRUCTOR & DESTRUCTOR
  KDTree(const BoundingBox &_bbox, int _depth=0) {
    bbox = _bbox;
    depth = _depth;
    child1=NULL;
    child2=NULL;
  }
  ~KDTree();

  // =========
  // ACCESSORS
  // boundingbox
  const glm::vec3& getMin() const { return bbox.getMin(); }
  const glm::vec3& getMax() const { return bbox.getMax(); }
  bool overlaps(const BoundingBox &bb) const;
  // hierarchy
  int getDepth() const { return depth; }
  bool isLeaf() const {
    if (child1==NULL&&child2==NULL) return true;
    assert (child1 != NULL && child2 != NULL);
    return false; }
  const KDTree* getChild1() const { assert (!isLeaf()); assert (child1 != NULL); return child1; }
  const KDTree* getChild2() const { assert (!isLeaf()); assert (child2 != NULL); return child2; }
  // photons
  const std::vector<Photon>& getPhotons() const { return photons; }
  int CountPhotonsInBox(const BoundingBox &bb) const;
  void CollectPhotonsInBox(const BoundingBox &bb, std::vector<Photon> &photons) const;
  void UpdatePhoton(const Photon& p);

  // =========
  // MODIFIERS
  void AddPhoton(const Photon &p);
  bool PhotonInCell(const Photon &p);

 private:

  // HELPER FUNCTION
  void SplitCell();

  // REPRESENTATION
  BoundingBox bbox;
  KDTree* child1;
  KDTree* child2;
  int split_axis;
  float split_value;
  std::vector<Photon> photons;
  int depth;
  std::mutex nodeMutex;
};

bool sortByX(const Photon &a, const Photon &b);
bool sortByY(const Photon &a, const Photon &b);
bool sortByZ(const Photon &a, const Photon &b);
#endif
