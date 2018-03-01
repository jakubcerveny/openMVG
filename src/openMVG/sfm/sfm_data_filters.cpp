// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data_filters.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/tracks/union_find.hpp"

#include <utility>

namespace openMVG {
namespace sfm {

/// List the view indexes that have valid camera intrinsic and pose.
std::set<IndexT> Get_Valid_Views
(
  const SfM_Data & sfm_data
)
{
  std::set<IndexT> valid_idx;
  for (const auto & view_it : sfm_data.GetViews())
  {
    const View * v = view_it.second.get();
    if (sfm_data.IsPoseAndIntrinsicDefined(v))
    {
      valid_idx.insert(v->id_view);
    }
  }
  return valid_idx;
}

// Remove tracks that have a small angle (tracks with tiny angle leads to instable 3D points)
// Return the number of removed tracks
IndexT RemoveOutliers_PixelResidualError
(
  SfM_Data & sfm_data,
  const double dThresholdPixel,
  const unsigned int minTrackLength
)
{
  IndexT outlier_count = 0;
  Landmarks::iterator iterTracks = sfm_data.structure.begin();
  while (iterTracks != sfm_data.structure.end())
  {
    Observations & obs = iterTracks->second.obs;
    Observations::iterator itObs = obs.begin();
    while (itObs != obs.end())
    {
      const View * view = sfm_data.views.at(itObs->first).get();
      const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view);
      const cameras::IntrinsicBase * intrinsic = sfm_data.intrinsics.at(view->id_intrinsic).get();
      const Vec2 residual = intrinsic->residual(pose(iterTracks->second.X), itObs->second.x);
      if (residual.norm() > dThresholdPixel)
      {
        ++outlier_count;
        itObs = obs.erase(itObs);
      }
      else
        ++itObs;
    }
    if (obs.empty() || obs.size() < minTrackLength)
      iterTracks = sfm_data.structure.erase(iterTracks);
    else
      ++iterTracks;
  }
  return outlier_count;
}

void CalculateResidualError(SfM_Data &sfm_data,
                            double &meanError, double &stddevError)
{
   meanError = stddevError = 0.0;
   std::size_t count = 0;

   Landmarks::iterator iterTracks = sfm_data.structure.begin();
   while (iterTracks != sfm_data.structure.end())
   {
     Observations & obs = iterTracks->second.obs;
     Observations::iterator itObs = obs.begin();
     while (itObs != obs.end())
     {
       const View * view = sfm_data.views.at(itObs->first).get();
       const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view);
       const cameras::IntrinsicBase * intrinsic = sfm_data.intrinsics.at(view->id_intrinsic).get();
       double residual = intrinsic->residual(pose(iterTracks->second.X), itObs->second.x).norm();

       meanError += residual;
       stddevError += residual*residual;
       count++;

       ++itObs;
     }
     ++iterTracks;
   }

   meanError /= count;
   stddevError = std::sqrt(stddevError / count - meanError*meanError);
}

void DecoupleViews(SfM_Data &sfm_data)
{
   Poses unique_poses;
   Intrinsics unique_intrin;

   for (Views::iterator
        iterViews = sfm_data.views.begin();
        iterViews != sfm_data.views.end();
        ++iterViews)
   {
      View *view = iterViews->second.get();
      if (!sfm_data.IsPoseAndIntrinsicDefined(view)) {
         continue;
      }

      const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view);
      const cameras::IntrinsicBase *intrinsic =
            sfm_data.intrinsics.at(view->id_intrinsic).get();

      unique_poses[view->id_view] = pose;
      unique_intrin[view->id_view] =
            std::shared_ptr<cameras::IntrinsicBase>(intrinsic->clone());

      view->id_pose = view->id_view;
      view->id_intrinsic = view->id_view;
   }

   sfm_data.poses.swap(unique_poses);
   sfm_data.intrinsics.swap(unique_intrin);
}

// Remove tracks that have a small angle (tracks with tiny angle leads to instable 3D points)
// Return the number of removed tracks
IndexT RemoveOutliers_AngleError
(
  SfM_Data & sfm_data,
  const double dMinAcceptedAngle
)
{
  IndexT removedTrack_count = 0;
  Landmarks::iterator iterTracks = sfm_data.structure.begin();
  while (iterTracks != sfm_data.structure.end())
  {
    Observations & obs = iterTracks->second.obs;
    double max_angle = 0.0;
    for (Observations::const_iterator itObs1 = obs.begin();
      itObs1 != obs.end(); ++itObs1)
    {
      const View * view1 = sfm_data.views.at(itObs1->first).get();
      const geometry::Pose3 pose1 = sfm_data.GetPoseOrDie(view1);
      const cameras::IntrinsicBase * intrinsic1 = sfm_data.intrinsics.at(view1->id_intrinsic).get();

      Observations::const_iterator itObs2 = itObs1;
      ++itObs2;
      for (; itObs2 != obs.end(); ++itObs2)
      {
        const View * view2 = sfm_data.views.at(itObs2->first).get();
        const geometry::Pose3 pose2 = sfm_data.GetPoseOrDie(view2);
        const cameras::IntrinsicBase * intrinsic2 = sfm_data.intrinsics.at(view2->id_intrinsic).get();

        const double angle = AngleBetweenRay(
          pose1, intrinsic1, pose2, intrinsic2,
          itObs1->second.x, itObs2->second.x);
        max_angle = std::max(angle, max_angle);
      }
    }
    if (max_angle < dMinAcceptedAngle)
    {
      iterTracks = sfm_data.structure.erase(iterTracks);
      ++removedTrack_count;
    }
    else
      ++iterTracks;
  }
  return removedTrack_count;
}

bool eraseMissingPoses
(
  SfM_Data & sfm_data,
  const IndexT min_points_per_pose
)
{
  IndexT removed_elements = 0;
  const Landmarks & landmarks = sfm_data.structure;

  // Count the observation poses occurrence
  Hash_Map<IndexT, IndexT> map_PoseId_Count;
  // Init with 0 count (in order to be able to remove non referenced elements)
  for (const auto & pose_it : sfm_data.GetPoses())
  {
    map_PoseId_Count[pose_it.first] = 0;
  }

  // Count occurrence of the poses in the Landmark observations
  for (const auto & lanmark_it : landmarks)
  {
    const Observations & obs = lanmark_it.second.obs;
    for (const auto obs_it : obs)
    {
      const IndexT ViewId = obs_it.first;
      const View * v = sfm_data.GetViews().at(ViewId).get();
      map_PoseId_Count[v->id_pose] += 1; // Default initialization is 0
    }
  }
  // If usage count is smaller than the threshold, remove the Pose
  for (const auto & it : map_PoseId_Count)
  {
    if (it.second < min_points_per_pose)
    {
      sfm_data.poses.erase(it.first);
      ++removed_elements;
    }
  }
  return removed_elements > 0;
}

bool eraseObservationsWithMissingPoses
(
  SfM_Data & sfm_data,
  const IndexT min_points_per_landmark
)
{
  IndexT removed_elements = 0;

  std::set<IndexT> pose_Index;
  std::transform(sfm_data.poses.cbegin(), sfm_data.poses.cend(),
    std::inserter(pose_Index, pose_Index.begin()), stl::RetrieveKey());

  // For each landmark:
  //  - Check if we need to keep the observations & the track
  Landmarks::iterator itLandmarks = sfm_data.structure.begin();
  while (itLandmarks != sfm_data.structure.end())
  {
    Observations & obs = itLandmarks->second.obs;
    Observations::iterator itObs = obs.begin();
    while (itObs != obs.end())
    {
      const IndexT ViewId = itObs->first;
      const View * v = sfm_data.GetViews().at(ViewId).get();
      if (pose_Index.count(v->id_pose) == 0)
      {
        itObs = obs.erase(itObs);
        ++removed_elements;
      }
      else
        ++itObs;
    }
    if (obs.empty() || obs.size() < min_points_per_landmark)
      itLandmarks = sfm_data.structure.erase(itLandmarks);
    else
      ++itLandmarks;
  }
  return removed_elements > 0;
}

/// Remove unstable content from analysis of the sfm_data structure
bool eraseUnstablePosesAndObservations
(
  SfM_Data & sfm_data,
  const IndexT min_points_per_pose,
  const IndexT min_points_per_landmark
)
{
  // First remove orphan observation(s) (observation using an undefined pose)
  eraseObservationsWithMissingPoses(sfm_data, min_points_per_landmark);
  // Then iteratively remove orphan poses & observations
  IndexT remove_iteration = 0;
  bool bRemovedContent = false;
  do
  {
    bRemovedContent = false;
    if (eraseMissingPoses(sfm_data, min_points_per_pose))
    {
      bRemovedContent = eraseObservationsWithMissingPoses(sfm_data, min_points_per_landmark);
      // Erase some observations can make some Poses index disappear so perform the process in a loop
    }
    remove_iteration += bRemovedContent ? 1 : 0;
  }
  while (bRemovedContent);

  return remove_iteration > 0;
}

/// Tell if the sfm_data structure is one CC or not
bool IsTracksOneCC
(
  const SfM_Data & sfm_data
)
{
  // Compute the Connected Component from the tracks

  // Build a table to have contiguous view index in [0,n]
  // (Use only the view index used in the observations)
  Hash_Map<IndexT, IndexT> view_renumbering;
  IndexT cpt = 0;
  const Landmarks & landmarks = sfm_data.structure;
  for (const auto & Landmark_it : landmarks)
  {
    const Observations & obs = Landmark_it.second.obs;
    for (const auto & obs_it : obs)
    {
      if (view_renumbering.count(obs_it.first) == 0)
      {
        view_renumbering[obs_it.first] = cpt++;
      }
    }
  }

  UnionFind uf_tree;
  uf_tree.InitSets(view_renumbering.size());

  // Link track observations in connected component
  for (const auto & Landmark_it : landmarks)
  {
    const Observations & obs = Landmark_it.second.obs;
    std::set<IndexT> id_to_link;
    for (const auto & obs_it : obs)
    {
      id_to_link.insert(view_renumbering.at(obs_it.first));
    }
    std::set<IndexT>::const_iterator iterI = id_to_link.cbegin();
    std::set<IndexT>::const_iterator iterJ = id_to_link.cbegin();
    std::advance(iterJ, 1);
    while (iterJ != id_to_link.cend())
    {
      // Link I => J
      uf_tree.Union(*iterI, *iterJ);
      ++iterJ;
    }
  }
  // Count the number of CC
  const std::set<unsigned int> parent_id(uf_tree.m_cc_parent.cbegin(), uf_tree.m_cc_parent.cend());
  return parent_id.size() == 1;
}

/// Keep the largest connected component of tracks from the sfm_data structure
void KeepLargestViewCCTracks
(
  SfM_Data & sfm_data
)
{
  // Compute the Connected Component from the tracks

  // Build a table to have contiguous view index in [0,n]
  // (Use only the view index used in the observations)
  Hash_Map<IndexT, IndexT> view_renumbering;
  {
    IndexT cpt = 0;
    const Landmarks & landmarks = sfm_data.structure;
    for (const auto & Landmark_it : landmarks)
    {
      const Observations & obs = Landmark_it.second.obs;
      for (const auto & obs_it : obs)
      {
        if (view_renumbering.count(obs_it.first) == 0)
        {
          view_renumbering[obs_it.first] = cpt++;
        }
      }
    }
  }

  UnionFind uf_tree;
  uf_tree.InitSets(view_renumbering.size());

  // Link track observations in connected component
  Landmarks & landmarks = sfm_data.structure;
  for (const auto & Landmark_it : landmarks)
  {
    const Observations & obs = Landmark_it.second.obs;
    std::set<IndexT> id_to_link;
    for (const auto & obs_it : obs)
    {
      id_to_link.insert(view_renumbering.at(obs_it.first));
    }
    std::set<IndexT>::const_iterator iterI = id_to_link.cbegin();
    std::set<IndexT>::const_iterator iterJ = id_to_link.cbegin();
    std::advance(iterJ, 1);
    while (iterJ != id_to_link.cend())
    {
      // Link I => J
      uf_tree.Union(*iterI, *iterJ);
      ++iterJ;
    }
  }

  // Count the number of CC
  const std::set<unsigned int> parent_id(uf_tree.m_cc_parent.cbegin(), uf_tree.m_cc_parent.cend());
  if (parent_id.size() > 1)
  {
    // There is many CC, look the largest one
    // (if many CC have the same size, export the first that have been seen)
    std::pair<IndexT, unsigned int> max_cc( UndefinedIndexT, std::numeric_limits<unsigned int>::min());
    {
      for (const unsigned int parent_id_it : parent_id)
      {
        if (uf_tree.m_cc_size[parent_id_it] > max_cc.second) // Update the component parent id and size
        {
          max_cc = {parent_id_it, uf_tree.m_cc_size[parent_id_it]};
        }
      }
    }
    // Delete track ids that are not contained in the largest CC
    if (max_cc.first != UndefinedIndexT)
    {
      const unsigned int parent_id_largest_cc = max_cc.first;
      Landmarks::iterator itLandmarks = landmarks.begin();
      while (itLandmarks != landmarks.end())
      {
        // Since we built a view 'track' graph thanks to the UF tree,
        //  checking the CC of each track is equivalent to check the CC of any observation of it.
        // So we check only the first
        const Observations & obs = itLandmarks->second.obs;
        Observations::const_iterator itObs = obs.begin();
        if (!obs.empty())
        {
          if (uf_tree.Find(view_renumbering.at(itObs->first)) != parent_id_largest_cc)
          {
            itLandmarks = landmarks.erase(itLandmarks);
          }
          else
          {
            ++itLandmarks;
          }
        }
      }
    }
  }
}

} // namespace sfm
} // namespace openMVG
