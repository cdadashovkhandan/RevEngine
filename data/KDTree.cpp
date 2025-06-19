/*
    Copyright:
    2018 Christoph Dalitz and Jens Wilberg
    Niederrhein University of Applied Sciences,
    Institute for Pattern Recognition,
    Reinarzstr. 49, 47805 Krefeld, Germany
    <http://www.hsnr.de/ipattern/>

    Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "KDTree.h"
#include <math.h>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <QVector3D>

//--------------------------------------------------------------
// function object for comparing only dimension d of two vecotrs
//--------------------------------------------------------------

#define POINT_SIZE 3

class compare_dimension {
 public:
  compare_dimension(size_t dim) { d = dim; }
  bool operator()(const KDNode& p, const KDNode& q) {
    return (p.point[d] < q.point[d]);
  }
  size_t d;
};

//--------------------------------------------------------------
// internal node structure used by KDTree
//--------------------------------------------------------------
class KDTree_node {
 public:
  KDTree_node() {
    dataindex = cutdim = 0;
    loson = hison = (KDTree_node*)NULL;
  }
  ~KDTree_node() {
    if (loson) delete loson;
    if (hison) delete hison;
  }
  // index of node data in KDTree array "allnodes"
  size_t dataindex;
  // cutting dimension
  size_t cutdim;
  // value of point
  // float cutval; // == point[cutdim]
  QVector3D point;
  //  roots of the two subtrees
  KDTree_node *loson, *hison;
  // bounding rectangle of this node's subtree
  QVector3D lobound, upbound;
};

//--------------------------------------------------------------
// different distance metrics
//--------------------------------------------------------------
class DistanceMeasure {
 public:
  DistanceMeasure() {}
  virtual ~DistanceMeasure() {}
  virtual float distance(const QVector3D& p, const QVector3D& q) = 0;
  virtual float coordinate_distance(float x, float y, size_t dim) = 0;
};
// Maximum distance (Linfinite norm)
class DistanceL0 : virtual public DistanceMeasure {
  FloatVector* w;

 public:
  DistanceL0(const FloatVector* weights = NULL) {
    if (weights)
      w = new FloatVector(*weights);
    else
      w = (FloatVector*)NULL;
  }
  ~DistanceL0() {
    if (w) delete w;
  }
  float distance(const QVector3D& p, const QVector3D& q) {
    size_t i;
    float dist, test;
    if (w) {
      dist = (*w)[0] * fabs(p[0] - q[0]);
      for (i = 1; i < POINT_SIZE; i++) {
        test = (*w)[i] * fabs(p[i] - q[i]);
        if (test > dist) dist = test;
      }
    } else {
      dist = fabs(p[0] - q[0]);
      for (i = 1; i < POINT_SIZE; i++) {
        test = fabs(p[i] - q[i]);
        if (test > dist) dist = test;
      }
    }
    return dist;
  }
  float coordinate_distance(float x, float y, size_t dim) {
    if (w)
      return (*w)[dim] * fabs(x - y);
    else
      return fabs(x - y);
  }
};
// Manhatten distance (L1 norm)
class DistanceL1 : virtual public DistanceMeasure {
  FloatVector* w;

 public:
  DistanceL1(const FloatVector* weights = NULL) {
    if (weights)
      w = new FloatVector(*weights);
    else
      w = (FloatVector*)NULL;
  }
  ~DistanceL1() {
    if (w) delete w;
  }
  float distance(const QVector3D& p, const QVector3D& q) {
    size_t i;
    float dist = 0.0;
    if (w) {
      for (i = 0; i < POINT_SIZE; i++) dist += (*w)[i] * fabs(p[i] - q[i]);
    } else {
      for (i = 0; i < POINT_SIZE; i++) dist += fabs(p[i] - q[i]);
    }
    return dist;
  }
  float coordinate_distance(float x, float y, size_t dim) {
    if (w)
      return (*w)[dim] * fabs(x - y);
    else
      return fabs(x - y);
  }
};
// Euklidean distance (L2 norm) (squared)
class DistanceL2 : virtual public DistanceMeasure {
  FloatVector* w;

 public:
  DistanceL2(const FloatVector* weights = NULL) {
    if (weights)
      w = new FloatVector(*weights);
    else
      w = (FloatVector*)NULL;
  }
  ~DistanceL2() {
    if (w) delete w;
  }
  float distance(const QVector3D& p, const QVector3D& q) {
    size_t i;
    float dist = 0.0;
    if (w) {
      for (i = 0; i < POINT_SIZE; i++)
        dist += (*w)[i] * (p[i] - q[i]) * (p[i] - q[i]);
    } else {
      for (i = 0; i < POINT_SIZE; i++) dist += (p[i] - q[i]) * (p[i] - q[i]);
    }
    return dist;
  }
  float coordinate_distance(float x, float y, size_t dim) {
    if (w)
      return (*w)[dim] * (x - y) * (x - y);
    else
      return (x - y) * (x - y);
  }
};

//--------------------------------------------------------------
// destructor and constructor of KDTree
//--------------------------------------------------------------
KDTree::~KDTree() {
  if (root) delete root;
  delete distance;
}
// distance_type can be 0 (Maximum), 1 (Manhatten), or 2 (Euklidean [squared])
KDTree::KDTree(const KDNodeVector* nodes, int distance_type /*=2*/) {
  size_t i, j;
  float val;
  // copy over input data
  if (!nodes || nodes->empty())
    throw std::invalid_argument(
        "KDTree::KDTree(): argument nodes must not be empty");
  dimension = 3;
  allnodes = *nodes;
  // initialize distance values
  distance = NULL;
  this->distance_type = -1;
  set_distance(distance_type);
  // compute global bounding box
  lobound = nodes->begin()->point;
  upbound = nodes->begin()->point;
  for (i = 1; i < nodes->size(); i++) {
    for (j = 0; j < dimension; j++) {
      val = allnodes[i].point[j];
      if (lobound[j] > val) lobound[j] = val;
      if (upbound[j] < val) upbound[j] = val;
    }
  }
  // build tree recursively
  root = build_tree(0, 0, allnodes.size());
}

// distance_type can be 0 (Maximum), 1 (Manhatten), or 2 (Euklidean [squared])
void KDTree::set_distance(int distance_type,
                          const FloatVector* weights /*=NULL*/) {
  if (distance) delete distance;
  this->distance_type = distance_type;
  if (distance_type == 0) {
    distance = (DistanceMeasure*)new DistanceL0(weights);
  } else if (distance_type == 1) {
    distance = (DistanceMeasure*)new DistanceL1(weights);
  } else {
    distance = (DistanceMeasure*)new DistanceL2(weights);
  }
}

//--------------------------------------------------------------
// recursive build of tree
// "a" and "b"-1 are the lower and upper indices
// from "allnodes" from which the subtree is to be built
//--------------------------------------------------------------
KDTree_node* KDTree::build_tree(size_t depth, size_t a, size_t b) {
  size_t m;
  float temp, cutval;
  KDTree_node* node = new KDTree_node();
  node->lobound = lobound;
  node->upbound = upbound;
  node->cutdim = depth % dimension;
  if (b - a <= 1) {
    node->dataindex = a;
    node->point = allnodes[a].point;
  } else {
    m = (a + b) / 2;
    std::nth_element(allnodes.begin() + a, allnodes.begin() + m,
                     allnodes.begin() + b, compare_dimension(node->cutdim));
    node->point = allnodes[m].point;
    cutval = allnodes[m].point[node->cutdim];
    node->dataindex = m;
    if (m - a > 0) {
      temp = upbound[node->cutdim];
      upbound[node->cutdim] = cutval;
      node->loson = build_tree(depth + 1, a, m);
      upbound[node->cutdim] = temp;
    }
    if (b - m > 1) {
      temp = lobound[node->cutdim];
      lobound[node->cutdim] = cutval;
      node->hison = build_tree(depth + 1, m + 1, b);
      lobound[node->cutdim] = temp;
    }
  }
  return node;
}

//--------------------------------------------------------------
// k nearest neighbor search
// returns the *k* nearest neighbors of *point* in O(log(n))
// time. The result is returned in *result* and is sorted by
// distance from *point*.
// The optional search predicate is a callable class (aka "functor")
// derived from KDNodePredicate. When Null (default, no search
// predicate is applied).
//--------------------------------------------------------------
void KDTree::k_nearest_neighbors(const QVector3D& point, size_t k,
                                 KDNodeVector* result,
                                 KDNodePredicate* pred /*=NULL*/) {
  size_t i;
  KDNode temp;
  searchpredicate = pred;

  result->clear();
  if (k < 1) return;
  if (dimension != POINT_SIZE)
    throw std::invalid_argument(
        "KDTree::k_nearest_neighbors(): point must be of same dimension as "
        "KDTree");

  // collect result of k values in neighborheap
  //std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>*
  //neighborheap = new std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>();
  SearchQueue* neighborheap = new SearchQueue();
  if (k > allnodes.size()) {
    // when more neighbors asked than nodes in tree, return everything
    k = allnodes.size();
    for (i = 0; i < k; i++) {
      if (!(searchpredicate && !(*searchpredicate)(allnodes[i])))
        neighborheap->push(
            nn4heap(i, distance->distance(allnodes[i].point, point)));
    }
  } else {
    neighbor_search(point, root, k, neighborheap);
  }

  // copy over result sorted by distance
  // (we must revert the vector for ascending order)
  while (!neighborheap->empty()) {
    i = neighborheap->top().dataindex;
    neighborheap->pop();
    result->push_back(allnodes[i]);
  }
  // beware that less than k results might have been returned
  k = result->size();
  for (i = 0; i < k / 2; i++) {
    temp = (*result)[i];
    (*result)[i] = (*result)[k - 1 - i];
    (*result)[k - 1 - i] = temp;
  }
  delete neighborheap;
}

//--------------------------------------------------------------
// range nearest neighbor search
// returns the nearest neighbors of *point* in the given range
// *r*. The result is returned in *result* and is sorted by
// distance from *point*.
//--------------------------------------------------------------
void KDTree::range_nearest_neighbors(const QVector3D& point, float r,
                                     KDNodeVector* result) {
  KDNode temp;

  result->clear();
  if (dimension != POINT_SIZE)
    throw std::invalid_argument(
        "KDTree::k_nearest_neighbors(): point must be of same dimension as "
        "KDTree");
  if (this->distance_type == 2) {
    // if euclidien distance is used the range must be squared because we
    // get squared distances from this implementation
    r *= r;
  }

  // collect result in range_result
  std::vector<size_t> range_result;
  range_search(point, root, r, &range_result);

  // copy over result
  for (std::vector<size_t>::iterator i = range_result.begin();
       i != range_result.end(); ++i) {
    result->push_back(allnodes[*i]);
  }

  // clear vector
  range_result.clear();
}

//--------------------------------------------------------------
// recursive function for nearest neighbor search in subtree
// under *node*. Stores result in *neighborheap*.
// returns "true" when no nearer neighbor elsewhere possible
//--------------------------------------------------------------
bool KDTree::neighbor_search(const QVector3D& point, KDTree_node* node,
                             size_t k, SearchQueue* neighborheap) {
  float curdist, dist;

  curdist = distance->distance(point, node->point);
  if (!(searchpredicate && !(*searchpredicate)(allnodes[node->dataindex]))) {
    if (neighborheap->size() < k) {
      neighborheap->push(nn4heap(node->dataindex, curdist));
    } else if (curdist < neighborheap->top().distance) {
      neighborheap->pop();
      neighborheap->push(nn4heap(node->dataindex, curdist));
    }
  }
  // first search on side closer to point
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->loson)
      if (neighbor_search(point, node->loson, k, neighborheap)) return true;
  } else {
    if (node->hison)
      if (neighbor_search(point, node->hison, k, neighborheap)) return true;
  }
  // second search on farther side, if necessary
  if (neighborheap->size() < k) {
    dist = std::numeric_limits<float>::max();
  } else {
    dist = neighborheap->top().distance;
  }
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->hison && bounds_overlap_ball(point, dist, node->hison))
      if (neighbor_search(point, node->hison, k, neighborheap)) return true;
  } else {
    if (node->loson && bounds_overlap_ball(point, dist, node->loson))
      if (neighbor_search(point, node->loson, k, neighborheap)) return true;
  }

  if (neighborheap->size() == k) dist = neighborheap->top().distance;
  return ball_within_bounds(point, dist, node);
}

//--------------------------------------------------------------
// recursive function for range search in subtree under *node*.
// Stores result in *range_result*.
//--------------------------------------------------------------
void KDTree::range_search(const QVector3D& point, KDTree_node* node,
                          float r, std::vector<size_t>* range_result) {
  float curdist = distance->distance(point, node->point);
  if (curdist <= r) {
    range_result->push_back(node->dataindex);
  }
  if (node->loson != NULL && this->bounds_overlap_ball(point, r, node->loson)) {
    range_search(point, node->loson, r, range_result);
  }
  if (node->hison != NULL && this->bounds_overlap_ball(point, r, node->hison)) {
    range_search(point, node->hison, r, range_result);
  }
}

// returns true when the bounds of *node* overlap with the
// ball with radius *dist* around *point*
bool KDTree::bounds_overlap_ball(const QVector3D& point, float dist,
                                 KDTree_node* node) {
  if (distance_type != 0) {
    float distsum = 0.0;
    size_t i;
    for (i = 0; i < dimension; i++) {
      if (point[i] < node->lobound[i]) {  // lower than low boundary
        distsum += distance->coordinate_distance(point[i], node->lobound[i], i);
        if (distsum > dist) return false;
      } else if (point[i] > node->upbound[i]) {  // higher than high boundary
        distsum += distance->coordinate_distance(point[i], node->upbound[i], i);
        if (distsum > dist) return false;
      }
    }
    return true;
  } else { // maximum distance needs different treatment
    float max_dist = 0.0;
    float curr_dist = 0.0;
    size_t i;
    for (i = 0; i < dimension; i++) {
      if (point[i] < node->lobound[i]) {  // lower than low boundary
        curr_dist = distance->coordinate_distance(point[i], node->lobound[i], i);
      } else if (point[i] > node->upbound[i]) {  // higher than high boundary
        curr_dist = distance->coordinate_distance(point[i], node->upbound[i], i);
      }
      if(curr_dist > max_dist) {
        max_dist = curr_dist;
      }
      if (max_dist > dist) return false;
    }
    return true;
  }
}

// returns true when the bounds of *node* completely contain the
// ball with radius *dist* around *point*
bool KDTree::ball_within_bounds(const QVector3D& point, float dist,
                                KDTree_node* node) {
  size_t i;
  for (i = 0; i < dimension; i++)
    if (distance->coordinate_distance(point[i], node->lobound[i], i) <= dist ||
        distance->coordinate_distance(point[i], node->upbound[i], i) <= dist)
      return false;
  return true;
}
