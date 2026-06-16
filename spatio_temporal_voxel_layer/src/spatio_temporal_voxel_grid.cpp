/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *********************************************************************/

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"

namespace volume_grid
{

/*****************************************************************************/
SpatioTemporalVoxelGrid::SpatioTemporalVoxelGrid(rclcpp::Clock::SharedPtr clock, const float& voxel_size,
                                                 const double& background_value, const int& decay_model,
                                                 const double& voxel_decay, const bool& pub_voxels)
  : _clock(clock)
  , _decay_model(decay_model)
  , _background_value(background_value)
  , _voxel_size(voxel_size)
  , _voxel_decay(voxel_decay)
  , _pub_voxels(pub_voxels)
  , _grid_points(std::make_unique<std::vector<geometry_msgs::msg::Point32>>())
  , _cost_map(new std::unordered_map<occupany_cell, uint>)
/*****************************************************************************/
{
  this->InitializeGrid();
}

/*****************************************************************************/
SpatioTemporalVoxelGrid::~SpatioTemporalVoxelGrid(void)
/*****************************************************************************/
{
  // pcl pointclouds free themselves
  if (_cost_map)
  {
    delete _cost_map;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::InitializeGrid(void)
/*****************************************************************************/
{
  // initialize the OpenVDB Grid volume
  openvdb::initialize();

  // make it default to background value
  _grid = openvdb::DoubleGrid::create(_background_value);

  // setup scale and tranform
  openvdb::Mat4d m = openvdb::Mat4d::identity();
  m.preScale(openvdb::Vec3d(_voxel_size, _voxel_size, _voxel_size));
  m.preTranslate(openvdb::Vec3d(0, 0, 0));
  m.preRotate(openvdb::math::Z_AXIS, 0);

  // setup transform and other metadata
  _grid->setTransform(openvdb::math::Transform::createLinearTransform(m));
  _grid->setName("SpatioTemporalVoxelLayer");
  _grid->insertMeta("Voxel Size", openvdb::FloatMetadata(_voxel_size));
  _grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ClearFrustums(const std::vector<observation::MeasurementReading>& clearing_readings,
                                            std::unordered_set<occupany_cell>& cleared_cells)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // accelerate the decay of voxels interior to the frustum
  if (this->IsGridEmpty())
  {
    _grid_points->clear();
    _cost_map->clear();
    return;
  }

  _grid_points->clear();
  _cost_map->clear();

  std::vector<frustum_model> obs_frustums;

  if (clearing_readings.size() == 0)
  {
    TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
    return;
  }

  obs_frustums.reserve(clearing_readings.size());

  std::vector<observation::MeasurementReading>::const_iterator it = clearing_readings.begin();
  for (; it != clearing_readings.end(); ++it)
  {
    geometry::Frustum* frustum;
    if (it->_model_type == DEPTH_CAMERA)
    {
      frustum = new geometry::DepthCameraFrustum(it->_vertical_fov_in_rad, it->_horizontal_fov_in_rad, it->_min_z_in_m,
                                                 it->_max_z_in_m);
    }
    else if (it->_model_type == THREE_DIMENSIONAL_LIDAR)
    {
      frustum =
          new geometry::ThreeDimensionalLidarFrustum(it->_vertical_fov_in_rad, it->_vertical_fov_padding_in_m,
                                                     it->_horizontal_fov_in_rad, it->_min_z_in_m, it->_max_z_in_m);
    }
    else
    {
      // add else if statement for each implemented model
      delete frustum;
      continue;
    }

    frustum->SetPosition(it->_origin);
    frustum->SetOrientation(it->_orientation);
    frustum->TransformModel();
    obs_frustums.emplace_back(frustum, it->_decay_acceleration);
  }
  TemporalClearAndGenerateCostmap(obs_frustums, cleared_cells);
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::TemporalClearAndGenerateCostmap(std::vector<frustum_model>& frustums,
                                                              std::unordered_set<occupany_cell>& cleared_cells)
/*****************************************************************************/
{
  // sample time once for all clearing readings
  const double cur_time = _clock->now().seconds();

  // check each point in the grid for inclusion in a frustum
  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (; cit_grid.test(); ++cit_grid)
  {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    std::vector<frustum_model>::iterator frustum_it = frustums.begin();
    bool frustum_cycle = false;
    bool cleared_point = false;

    const double time_since_marking = cur_time - cit_grid.getValue();
    const double base_duration_to_decay = GetTemporalClearingDuration(time_since_marking);

    for (; frustum_it != frustums.end(); ++frustum_it)
    {
      if (!frustum_it->frustum)
      {
        continue;
      }

      if (frustum_it->frustum->IsInside(pose_world))
      {
        frustum_cycle = true;

        const double frustum_acceleration = GetFrustumAcceleration(time_since_marking, frustum_it->accel_factor);

        const double time_until_decay = base_duration_to_decay - frustum_acceleration;
        if (time_until_decay < 0.)
        {
          // expired by acceleration
          cleared_point = true;
          if (!this->ClearGridPoint(pt_index))
          {
            std::cout << "Failed to clear point." << std::endl;
          }
          break;
        }
        else
        {
          const double updated_mark = cit_grid.getValue() - frustum_acceleration;
          if (!this->MarkGridPoint(pt_index, updated_mark))
          {
            std::cout << "Failed to update mark." << std::endl;
          }
          break;
        }
      }
    }

    // if not inside any, check against nominal decay model
    if (!frustum_cycle)
    {
      if (base_duration_to_decay < 0.)
      {
        // expired by temporal clearing
        cleared_point = true;
        if (!this->ClearGridPoint(pt_index))
        {
          std::cout << "Failed to clear point." << std::endl;
        }
      }
    }

    if (cleared_point)
    {
      cleared_cells.insert(occupany_cell(pose_world[0], pose_world[1]));
    }
    else
    {
      // if here, we can add to costmap and PC2
      PopulateCostmapAndPointcloud(pt_index);
    }
  }

  // free memory taken by expired voxels
  _grid->pruneGrid();
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::PopulateCostmapAndPointcloud(const openvdb::Coord& pt)
/*****************************************************************************/
{
  // add pt to the pointcloud and costmap
  openvdb::Vec3d pose_world = this->IndexToWorld(pt);

  if (_pub_voxels)
  {
    geometry_msgs::msg::Point32 point;
    point.x = pose_world[0];
    point.y = pose_world[1];
    point.z = pose_world[2];
    _grid_points->push_back(point);
  }

  std::unordered_map<occupany_cell, uint>::iterator cell;
  cell = _cost_map->find(occupany_cell(pose_world[0], pose_world[1]));
  if (cell != _cost_map->end())
  {
    cell->second += 1;
  }
  else
  {
    _cost_map->insert(std::make_pair(occupany_cell(pose_world[0], pose_world[1]), 1));
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::Mark(const std::vector<observation::MeasurementReading>& marking_readings,
                                   const double& robot_x, const double& robot_y)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // Pre-select active ignore rects once before processing any points
  SelectActiveIgnoreRects(robot_x, robot_y);

  // mark the grid
  if (marking_readings.size() > 0)
  {
    for (uint i = 0; i != marking_readings.size(); i++)
    {
      (*this)(marking_readings.at(i));
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::operator()(const observation::MeasurementReading& obs) const
/*****************************************************************************/
{
  if (obs._marking)
  {
    float mark_range_2 = obs._obstacle_range_in_m * obs._obstacle_range_in_m;
    float mark_min_range_2 = obs._min_obstacle_range_in_m * obs._min_obstacle_range_in_m;
    const double cur_time = _clock->now().seconds();

    const sensor_msgs::msg::PointCloud2& cloud = *(obs._cloud);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      float distance_2 = (*iter_x - obs._origin.x) * (*iter_x - obs._origin.x) +
                         (*iter_y - obs._origin.y) * (*iter_y - obs._origin.y) +
                         (*iter_z - obs._origin.z) * (*iter_z - obs._origin.z);
      if (distance_2 > mark_range_2 || distance_2 < mark_min_range_2)
      {
        continue;
      }
      if (obs._enable_cut && *iter_z > obs._cut_min_z_in_m && *iter_z < obs._cut_max_z_in_m)
      {
        geometry_msgs::msg::Point point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        bool is_point_in_cut_volume = IsPointInRectangle(obs._cut_inside, obs._cut_outside, obs._cut_extend_outside,
                                                         obs._cut_extend_inside, point);
        if (is_point_in_cut_volume)
        {
          continue;
        }
      }

      if (IsPointInAnyIgnorePolygon(static_cast<double>(*iter_x), static_cast<double>(*iter_y)))
      {
        continue;
      }

      double x = *iter_x < 0 ? *iter_x - _voxel_size : *iter_x;
      double y = *iter_y < 0 ? *iter_y - _voxel_size : *iter_y;
      double z = *iter_y < 0 ? *iter_z - _voxel_size : *iter_z;

      openvdb::Vec3d mark_grid(this->WorldToIndex(openvdb::Vec3d(x, y, z)));

      if (!this->MarkGridPoint(openvdb::Coord(mark_grid[0], mark_grid[1], mark_grid[2]), cur_time))
      {
        std::cout << "Failed to mark point." << std::endl;
      }
    }
  }
}

/*****************************************************************************/
std::unordered_map<occupany_cell, uint>* SpatioTemporalVoxelGrid::GetFlattenedCostmap()
/*****************************************************************************/
{
  return _cost_map;
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetTemporalClearingDuration(const double& time_delta)
/*****************************************************************************/
{
  // use configurable model to get desired decay time
  if (_decay_model == 0)
  {  // Linear
    return _voxel_decay - time_delta;
  }
  else if (_decay_model == 1)
  {  // Exponential
    return _voxel_decay * std::exp(-time_delta);
  }
  return _voxel_decay;  // PERSISTENT
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetFrustumAcceleration(const double& time_delta, const double& acceleration_factor)
/*****************************************************************************/
{
  const double acceleration = 1. / 6. * acceleration_factor * (time_delta * time_delta * time_delta);
  return acceleration;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::GetOccupancyPointCloud(std::unique_ptr<sensor_msgs::msg::PointCloud2>& pc2)
/*****************************************************************************/
{
  // convert the grid points stored in a PointCloud2
  pc2->width = _grid_points->size();
  pc2->height = 1;
  pc2->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*pc2);

  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2, "z");

  for (std::vector<geometry_msgs::msg::Point32>::iterator it = _grid_points->begin(); it != _grid_points->end(); ++it)
  {
    const geometry_msgs::msg::Point32& pt = *it;
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ResetGrid(void)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  // clear the voxel grid
  try
  {
    _grid->clear();
    if (this->IsGridEmpty())
    {
      return true;
    }
  }
  catch (...)
  {
    std::cout << "Failed to reset costmap, please try again." << std::endl;
  }
  return false;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::ResetGridArea(const occupany_cell& start, const occupany_cell& end, bool invert_area)
/*****************************************************************************/
{
  boost::unique_lock<boost::mutex> lock(_grid_lock);

  openvdb::DoubleGrid::ValueOnCIter cit_grid = _grid->cbeginValueOn();
  for (cit_grid; cit_grid.test(); ++cit_grid)
  {
    const openvdb::Coord pt_index(cit_grid.getCoord());
    const openvdb::Vec3d pose_world = this->IndexToWorld(pt_index);

    const bool in_x_range = pose_world.x() > start.x && pose_world.x() < end.x;
    const bool in_y_range = pose_world.y() > start.y && pose_world.y() < end.y;
    const bool in_range = in_x_range && in_y_range;

    if (in_range != invert_area)
    {
      ClearGridPoint(pt_index);
    }
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::MarkGridPoint(const openvdb::Coord& pt, const double& value) const
/*****************************************************************************/
{
  // marking the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  accessor.setValueOn(pt, value);
  return accessor.getValue(pt) == value;
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::ClearGridPoint(const openvdb::Coord& pt) const
/*****************************************************************************/
{
  // clearing the OpenVDB set
  openvdb::DoubleGrid::Accessor accessor = _grid->getAccessor();

  if (accessor.isValueOn(pt))
  {
    accessor.setValueOff(pt, _background_value);
  }
  return !accessor.isValueOn(pt);
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::IndexToWorld(const openvdb::Coord& coord) const
/*****************************************************************************/
{
  // Applies tranform stored in getTransform.
  openvdb::Vec3d pose_world = _grid->indexToWorld(coord);

  // Using the center for world coordinate
  const double& center_offset = _voxel_size / 2.0;
  pose_world[0] += center_offset;
  pose_world[1] += center_offset;
  pose_world[2] += center_offset;

  return pose_world;
}

/*****************************************************************************/
openvdb::Vec3d SpatioTemporalVoxelGrid::WorldToIndex(const openvdb::Vec3d& vec) const
/*****************************************************************************/
{
  // Applies inverse tranform stored in getTransform.
  return _grid->worldToIndex(vec);
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::IsGridEmpty(void) const
/*****************************************************************************/
{
  // Returns grid's population status
  return _grid->empty();
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::SaveGrid(const std::string& file_name, double& map_size_bytes)
/*****************************************************************************/
{
  try
  {
    openvdb::io::File file(file_name + ".vdb");
    openvdb::GridPtrVec grids = { _grid };
    file.write(grids);
    file.close();
    map_size_bytes = _grid->memUsage();
    return true;
  }
  catch (...)
  {
    map_size_bytes = 0.;
    return false;
  }
  return false;  // best offense is a good defense
}

double SpatioTemporalVoxelGrid::CrossProduct(const geometry_msgs::msg::Point& first_point,
                                             const geometry_msgs::msg::Point& second_point,
                                             const geometry_msgs::msg::Point& point) const
{
  double cross_product =
      (first_point.x - point.x) * (second_point.y - point.y) - (first_point.y - point.y) * (second_point.x - point.x);

  return cross_product;
}

bool SpatioTemporalVoxelGrid::IsPointInRectangle(const geometry_msgs::msg::Point& inside,
                                                 const geometry_msgs::msg::Point& outside,
                                                 const geometry_msgs::msg::Point& extend_outside,
                                                 const geometry_msgs::msg::Point& extend_inside,
                                                 const geometry_msgs::msg::Point& point) const
{
  // 步骤1: 根据对角点计算矩形的四个顶点 (按顺时针顺序)
  std::vector<geometry_msgs::msg::Point> rectvertices(4);
  rectvertices[0] = inside;
  rectvertices[1] = outside;
  rectvertices[2] = extend_outside;
  rectvertices[3] = extend_inside;

  // 步骤2: 计算点与每条边的叉积符号
  int sign = 0;
  for (int i = 0; i < 4; i++)
  {
    int j = (i + 1) % 4;
    double cross = CrossProduct(rectvertices[i], rectvertices[j], point);

    // 初始化符号 (跳过零值)
    if (cross != 0)
    {
      if (sign == 0)
      {
        sign = (cross > 0) ? 1 : -1;
      }
      else
      {
        // 符号不一致则点在外侧
        if ((sign == 1 && cross < 0) || (sign == -1 && cross > 0))
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool SpatioTemporalVoxelGrid::AreVerticesOrdered(const std::vector<geometry_msgs::msg::Point>& rectvertices)
{
  // 检查输入顶点数量
  if (rectvertices.size() != 4)
  {
    std::cout << "Vertices size is less than 4, current size: " << rectvertices.size() << " !" << std::endl;
    return false;
  }
  const double epsilon = 1e-9;
  int prev_sign = 0;
  for (int i = 0; i < 4; ++i)
  {
    int j = (i + 1) % 4;
    int k = (i + 2) % 4;
    // 计算向量 (v_i -> v_j) 和向量 (v_i -> v_k) 的叉积
    double cross = CrossProduct(rectvertices[i], rectvertices[k], rectvertices[j]);

    // 确定当前叉积的符号
    int current_sign = 0;
    if (cross > epsilon)
    {
      current_sign = 1;
    }
    else if (cross < -epsilon)
    {
      current_sign = -1;
    }
    else
    {
      return false;
    }
    if (prev_sign == 0)
    {
      prev_sign = current_sign;
    }
    else
    {
      if (prev_sign != current_sign)
      {
        std::cout << "Vertices are not ordered for stvl!" << std::endl;
        return false;
      }
    }
  }
  return true;  // 所有连续的叉积符号一致，顶点按顺序排列
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::SetIgnorePolygons(const std::vector<nav_msgs::msg::Path>& paths)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(_ignore_polygons_lock);
  _ignore_paths = paths;
}

/*****************************************************************************/
std::vector<nav_msgs::msg::Path> SpatioTemporalVoxelGrid::GetIgnorePolygons() const
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(_ignore_polygons_lock);
  return _ignore_paths;
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::SetIgnoreWidth(int width)
/*****************************************************************************/
{
  _ignore_width.store(width);
}

/*****************************************************************************/
int SpatioTemporalVoxelGrid::GetIgnoreWidth() const
/*****************************************************************************/
{
  return _ignore_width.load();
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::SetIgnoreRange(double range)
/*****************************************************************************/
{
  _ignore_range.store(range);
}

/*****************************************************************************/
double SpatioTemporalVoxelGrid::GetIgnoreRange() const
/*****************************************************************************/
{
  return _ignore_range.load();
}

/*****************************************************************************/
void SpatioTemporalVoxelGrid::SelectActiveIgnoreRects(const double& robot_x, const double& robot_y)
/*****************************************************************************/
{
  const int width = _ignore_width.load();
  if (width == 0)
  {
    boost::mutex::scoped_lock lock(_ignore_polygons_lock);
    _active_ignore_rects.clear();
    return;
  }

  const double offset = std::abs(static_cast<double>(width)) / 100.0;  // convert cm to meters
  const double range = _ignore_range.load();
  const double range_sq = range * range;

  boost::mutex::scoped_lock lock(_ignore_polygons_lock);
  _active_ignore_rects.clear();

  for (const auto& path : _ignore_paths)
  {
    if (path.poses.size() < 2)
    {
      continue;
    }

    // Step 1: For each segment, check if it intersects the range circle (point-to-segment distance)
    std::vector<size_t> nearby_segments;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i)
    {
      const double ax = path.poses[i].pose.position.x;
      const double ay = path.poses[i].pose.position.y;
      const double bx = path.poses[i + 1].pose.position.x;
      const double by = path.poses[i + 1].pose.position.y;

      const double seg_dx = bx - ax;
      const double seg_dy = by - ay;
      const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

      double min_dist_sq;
      if (seg_len_sq < 1e-18)
      {
        // Degenerate segment, treat as point
        const double dx = ax - robot_x;
        const double dy = ay - robot_y;
        min_dist_sq = dx * dx + dy * dy;
      }
      else
      {
        // Project robot onto segment line, clamp to [0,1]
        const double to_ax = robot_x - ax;
        const double to_ay = robot_y - ay;
        double t = (to_ax * seg_dx + to_ay * seg_dy) / seg_len_sq;
        t = std::max(0.0, std::min(1.0, t));

        // Closest point on segment to robot
        const double closest_x = ax + t * seg_dx;
        const double closest_y = ay + t * seg_dy;
        const double dx = closest_x - robot_x;
        const double dy = closest_y - robot_y;
        min_dist_sq = dx * dx + dy * dy;
      }

      if (min_dist_sq <= range_sq)
      {
        nearby_segments.push_back(i);
      }
    }

    if (nearby_segments.empty())
    {
      continue;
    }

    // Step 2: For each nearby segment, generate a fine-grained rectangle
    for (const auto i : nearby_segments)
    {
      const size_t j = i + 1;

      const auto& p0 = path.poses[i].pose.position;
      const auto& p1 = path.poses[j].pose.position;

      // Compute tangent direction (average of tangents at i and j)
      double dx0, dy0;
      if (i == 0)
      {
        dx0 = path.poses[1].pose.position.x - p0.x;
        dy0 = path.poses[1].pose.position.y - p0.y;
      }
      else
      {
        dx0 = p0.x - path.poses[i - 1].pose.position.x;
        dy0 = p0.y - path.poses[i - 1].pose.position.y;
      }

      double dx1, dy1;
      if (j == path.poses.size() - 1)
      {
        dx1 = p1.x - path.poses[j - 1].pose.position.x;
        dy1 = p1.y - path.poses[j - 1].pose.position.y;
      }
      else
      {
        dx1 = path.poses[j + 1].pose.position.x - p1.x;
        dy1 = path.poses[j + 1].pose.position.y - p1.y;
      }

      const double len0 = std::sqrt(dx0 * dx0 + dy0 * dy0);
      const double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
      if (len0 < 1e-9 || len1 < 1e-9)
      {
        continue;
      }

      // Normal vectors (rotated 90° counter-clockwise)
      const double nx0 = -dy0 / len0;
      const double ny0 = dx0 / len0;
      const double nx1 = -dy1 / len1;
      const double ny1 = dx1 / len1;

      // Determine robot side via cross product at midpoint
      const double mid_x = (p0.x + p1.x) * 0.5;
      const double mid_y = (p0.y + p1.y) * 0.5;
      const double dir_x = p1.x - p0.x;
      const double dir_y = p1.y - p0.y;
      const double to_robot_x = robot_x - mid_x;
      const double to_robot_y = robot_y - mid_y;
      const double cross = dir_x * to_robot_y - dir_y * to_robot_x;

      // If cross > 0, robot is on left (positive normal direction), ignore left side
      // If cross < 0, robot is on right (negative normal direction), ignore right side
      const double sign = (cross >= 0) ? 1.0 : -1.0;

      // Generate offset points on the robot's side
      const double ox0 = p0.x + sign * nx0 * offset;
      const double oy0 = p0.y + sign * ny0 * offset;
      const double ox1 = p1.x + sign * nx1 * offset;
      const double oy1 = p1.y + sign * ny1 * offset;

      // Build AABB rectangle from path points and offset points
      double min_x = std::min({ p0.x, p1.x, ox0, ox1 });
      double max_x = std::max({ p0.x, p1.x, ox0, ox1 });
      double min_y = std::min({ p0.y, p1.y, oy0, oy1 });
      double max_y = std::max({ p0.y, p1.y, oy0, oy1 });

      geometry_msgs::msg::PolygonStamped rect;
      rect.header = path.header;

      geometry_msgs::msg::Point32 c0, c1, c2, c3;
      c0.x = static_cast<float>(min_x);
      c0.y = static_cast<float>(min_y);
      c0.z = 0.0f;
      c1.x = static_cast<float>(max_x);
      c1.y = static_cast<float>(min_y);
      c1.z = 0.0f;
      c2.x = static_cast<float>(max_x);
      c2.y = static_cast<float>(max_y);
      c2.z = 0.0f;
      c3.x = static_cast<float>(min_x);
      c3.y = static_cast<float>(max_y);
      c3.z = 0.0f;

      rect.polygon.points = { c0, c1, c2, c3 };
      _active_ignore_rects.push_back(rect);
    }
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelGrid::IsPointInAnyIgnorePolygon(const double& px, const double& py) const
/*****************************************************************************/
{
  // No lock needed — _active_ignore_rects is set by SelectActiveIgnoreRects under _grid_lock,
  // and operator() is also called under _grid_lock (from Mark).
  for (const auto& polygon_stamped : _active_ignore_rects)
  {
    const auto& pts = polygon_stamped.polygon.points;
    if (pts.size() < 4)
      continue;

    geometry_msgs::msg::Point inside, outside, extend_outside, extend_inside;
    inside.x = pts[0].x;
    inside.y = pts[0].y;
    outside.x = pts[2].x;
    outside.y = pts[2].y;
    extend_outside.x = pts[3].x;
    extend_outside.y = pts[3].y;
    extend_inside.x = pts[1].x;
    extend_inside.y = pts[1].y;

    geometry_msgs::msg::Point point;
    point.x = px;
    point.y = py;

    if (IsPointInRectangle(inside, outside, extend_outside, extend_inside, point))
    {
      return true;
    }
  }
  return false;
}

}  // namespace volume_grid
