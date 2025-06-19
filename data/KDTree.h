#ifndef __KDTree_H
#define __KDTree_H

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

#include <QVector>
#include <QVector3D>
#include <cstdlib>
#include <queue>
typedef std::vector<float> FloatVector;

// for passing points to the constructor of KDTree
struct KDNode {
  QVector3D point;
  void* data;
  int index;
  KDNode(const QVector3D& p, void* d = NULL, int i = -1) {
    point = p;
    data = d;
    index = i;
  }
  KDNode() { data = NULL; }
};
typedef QVector<KDNode> KDNodeVector;

// base function object for search predicate in knn search
// returns true when the given KDNode is an admissible neighbor
// To define an own search predicate, derive from this class
// and overwrite the call operator operator()
struct KDNodePredicate {
  virtual ~KDNodePredicate() {}
  virtual bool operator()(const KDNode&) const { return true; }
};

//--------------------------------------------------------
// private helper classes used internally by KDTree
//
// the internal node structure used by KDTree
class KDTree_node;
// base class for different distance computations
class DistanceMeasure;
// helper class for priority queue in k nearest neighbor search
class nn4heap {
 public:
  size_t dataindex;  // index of actual KDNode in *allnodes*
  float distance;   // distance of this neighbor from *point*
  nn4heap(size_t i, float d) {
    dataindex = i;
    distance = d;
  }
};
class compare_nn4heap {
 public:
  bool operator()(const nn4heap& n, const nn4heap& m) {
    return (n.distance < m.distance);
  }
};
  typedef std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap> SearchQueue;
//--------------------------------------------------------

// KDTree class
class KDTree {
 private:
  // recursive build of tree
  KDTree_node* build_tree(size_t depth, size_t a, size_t b);
  // helper variable for keeping track of subtree bounding box
  QVector3D lobound, upbound;
  // helper variable to check the distance method
  int distance_type;
  bool neighbor_search(const QVector3D& point, KDTree_node* node, size_t k, SearchQueue* neighborheap);
  void range_search(const QVector3D& point, KDTree_node* node, float r, std::vector<size_t>* range_result);
  bool bounds_overlap_ball(const QVector3D& point, float dist,
                           KDTree_node* node);
  bool ball_within_bounds(const QVector3D& point, float dist,
                          KDTree_node* node);
  // class implementing the distance computation
  DistanceMeasure* distance;
  // search predicate in knn searches
  KDNodePredicate* searchpredicate;

 public:
  KDNodeVector allnodes;
  size_t dimension;
  KDTree_node* root;
  // distance_type can be 0 (max), 1 (city block), or 2 (euklid [squared])
  KDTree(const KDNodeVector* nodes, int distance_type = 2);
  ~KDTree();
  void set_distance(int distance_type, const FloatVector* weights = NULL);
  void k_nearest_neighbors(const QVector3D& point, size_t k,
                           KDNodeVector* result, KDNodePredicate* pred = NULL);
  void range_nearest_neighbors(const QVector3D& point, float r,
                               KDNodeVector* result);
};

#endif
