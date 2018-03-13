// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2016 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_transform.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"

#include "openMVG/geometry/Similarity3_Kernel.hpp"

namespace openMVG {
namespace sfm {

/** Triangulate control points and find best rigid fit of the scene to the
*   provided control point 3D positions.
*/
bool FindControlPointsRigidFit
(
  const SfM_Data &sfm_data,
  Similarity3 &similarity
)
{
  if (sfm_data.control_points.size() < 3)
  {
    std::cout << sfm_data.control_points.size() << " control points "
                 "is not enough to perform a rigid fit." << std::endl;
    return false;
  }

  // triangulate control points
  SfM_Data sfm_dummy = sfm_data;
  sfm_dummy.structure = sfm_data.control_points;
  sfm_dummy.control_points.clear();

  SfM_Data_Structure_Computation_Blind structure_estimator;
  structure_estimator.triangulate(sfm_dummy);

  if (sfm_dummy.structure.size() != sfm_data.control_points.size())
  {
    std::cout << "Only " << sfm_dummy.structure.size() << " out of "
              << sfm_data.control_points.size() << " control points could be "
              << "triangulated." << std::endl;
    return false;
  }

  // get the two sets of 3D points
  Mat X(3, sfm_dummy.structure.size());
  int index = 0;
  for (const auto &iter : sfm_dummy.structure)
  {
    X.col(index++) = iter.second.X;
  }

  Mat Y(3, sfm_data.control_points.size());
  index = 0;
  for (const auto &iter : sfm_data.control_points)
  {
    Y.col(index++) = iter.second.X;
  }

  // calculate similarity transformation
  openMVG::geometry::kernel::Similarity3Solver similarity_solver;
  std::vector<geometry::Similarity3> sims;
  similarity_solver.Solve(X, Y, &sims);

  if (sims.size() == 0)
  {
    std::cout << "Could solve for the rigid fit transformation." << std::endl;
    return false;
  }

  similarity = sims.back();
  return true;
}

/// Apply a similarity to the SfM_Data scene (transform landmarks & camera poses)
void ApplySimilarity
(
  const geometry::Similarity3 & sim,
  SfM_Data & sfm_data,
  bool transform_priors
)
{
  // Transform the landmark positions
  for (auto & iterLandMark : sfm_data.structure)
  {
    iterLandMark.second.X = sim(iterLandMark.second.X);
  }

  // Transform the camera positions
  for (auto & iterPose : sfm_data.poses)
  {
    iterPose.second = sim(iterPose.second);
  }

  if (transform_priors)
  {
    for (auto & iterView : sfm_data.views)
    {
      // Transform the camera position priors
      if (sfm::ViewPriors * prior = dynamic_cast<sfm::ViewPriors*>(iterView.second.get()))
      {
        prior->pose_center_ = sim(prior->pose_center_);
      }
    }

    // Transform the control points
    for (auto & iterControlPoint : sfm_data.control_points)
    {
      iterControlPoint.second.X = sim(iterControlPoint.second.X);
    }
  }
}

} // namespace sfm
} // namespace openMVG
