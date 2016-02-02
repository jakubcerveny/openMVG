
// Copyright (c) 2016 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_UNION_FIND_DISJOINT_SET_HPP_
#define OPENMVG_UNION_FIND_DISJOINT_SET_HPP_

#include <vector>
#include <numeric>

namespace openMVG  {

// Union-Find/Disjoint-Set data structure
//--
// A disjoint-set data structure also called a union–find data structure
// or merge–find set, is a data structure that keeps track of a set of elements
// partitioned into a number of disjoint (nonoverlapping) subsets.
// It supports two operations:
// - Find: Determine which subset a particular element is in.
//   - It returns an item from this set that serves as its "representative";
// - Union: Join two subsets into a single subset.
// Sometime a Connected method is implemented:
// - Connected:
//   - By comparing the result of two Find operations, one can determine whether
//      two elements are in the same subset.
//--
struct UnionFind
{
  // Represent the DS/UF forest thanks to two array:
  // A parent 'pointer tree' where each node holds a reference to its parent node
  std::vector<unsigned int> m_cc_parent;
  // A 'rank/size array' to know the size of each connected component
  std::vector<unsigned int> m_cc_size;

  // Init the UF structure with num_cc nodes
  void InitSets
  (
    const unsigned int num_cc
  )
  {
    // all set size are 1 (independent nodes)
    m_cc_size.resize(num_cc, 1);
    // Parents id have their own CC id {0,n}
    m_cc_parent.resize(num_cc);
    std::iota(m_cc_parent.begin(), m_cc_parent.end(), 0);
  }

  // Return the number of nodes that have been initialized in the UF tree
  unsigned int GetNumNodes() const
  {
    return m_cc_size.size();
  }

  // Return the representative set id of I nth component
  unsigned int Find
  (
    unsigned int i
  )
  {
    while (i != m_cc_parent[i])
    {
      m_cc_parent[i] = m_cc_parent[m_cc_parent[i]]; // Path compression
      i = m_cc_parent[i];
    }
    return i;
  }

  // Replace sets containing I and J with their union
  void Union
  (
    unsigned int i,
    unsigned int j
  )
  {
    i = Find(i);
    j = Find(j);
    if (i==j) return;
    if (m_cc_size[i] < m_cc_size[j])
    {
      m_cc_parent[i] = j;
      m_cc_size[j] += m_cc_size[i];
    }
    else
    {
      m_cc_parent[j] = i;
      m_cc_size[i] += m_cc_size[j];
    }
  }
};

} // namespace openMVG

#endif // OPENMVG_UNION_FIND_DISJOINT_SET_HPP_