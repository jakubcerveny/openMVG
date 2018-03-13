// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2016 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_SFM_DATA_TRANSFORM_HPP
#define OPENMVG_SFM_SFM_DATA_TRANSFORM_HPP

#include "openMVG/geometry/Similarity3.hpp"

namespace openMVG {

namespace sfm {

struct SfM_Data;

/** Triangulate control points and find best rigid fit of the scene to the
*   provided control point 3D positions.
*/
bool FindControlPointsRigidFit
(
  const SfM_Data & sfm_data,
  geometry::Similarity3 & similarity
);

/// Apply a similarity to the SfM_Data scene (transform landmarks & camera poses)
void ApplySimilarity
(
  const geometry::Similarity3 & sim,
  SfM_Data & sfm_data,
  bool transform_priors = false
);

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_SFM_DATA_TRANSFORM_HPP
