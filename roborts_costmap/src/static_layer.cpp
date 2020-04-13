/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *********************************************************************/
#include "static_layer_setting.pb.h"
#include "static_layer.h"

namespace roborts_costmap {

void StaticLayer::OnInitialize() {
  ros::NodeHandle nh;
  is_current_ = true;
  ParaStaticLayer para_static_layer;

  std::string config_dir;
  nh.getParam("config_dir", config_dir);
  std::string static_map = ros::package::getPath("roborts_costmap") + \
      "/config/" + config_dir + "/static_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(static_map.c_str(), &para_static_layer);
  global_frame_ = layered_costmap_->GetGlobalFrameID();
  first_map_only_ = para_static_layer.first_map_only();
  subscribe_to_updates_ = para_static_layer.subscribe_to_updates();
  track_unknown_space_ = para_static_layer.track_unknown_space();
  use_maximum_ = para_static_layer.use_maximum();
  int temp_threshold = para_static_layer.lethal_threshold();
  lethal_threshold_ = std::max(std::min(100, temp_threshold), 0);
  trinary_costmap_ = para_static_layer.trinary_map();
  unknown_cost_value_ = para_static_layer.unknown_cost_value();
  robot_color_ = para_static_layer.robot_color();
  buff_zone_smaller_ = para_static_layer.buff_zone_smaller();

  map_received_ = false;
  bool is_debug_ = para_static_layer.is_debug();
  map_topic_ = para_static_layer.topic_name();
  map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &StaticLayer::InComingMap, this);

  buff_zone_sub_ = nh.subscribe("/buff_zone_status", 1, &StaticLayer::UpdateBuffZoneStatus, this);

  ros::Rate temp_rate(10);
  while (!map_received_) {
    ros::spinOnce();
    temp_rate.sleep();
  }

  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  is_enabled_ = true;
  has_updated_data_ = true;

  // double min_x = -0.215523916483;
  // double min_y = 1.259628582;
  // double max_x = 0.532082033157;
  // double max_y = 2.0255279541;
  BuffZone buff_zone{};

  buff_zone.max_x_ = -3.10139417648;
  buff_zone.max_y_ = 0.828392624855;
  buff_zone.min_x_ = -3.89651417732;
  buff_zone.min_y_ = 0.290447890759;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(1, buff_zone));

  buff_zone.max_x_ = -1.79103124142;
  buff_zone.max_y_ = -0.226469397545;
  buff_zone.min_x_ = -2.53039693832;
  buff_zone.min_y_ = -0.718370676041;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(2, buff_zone));

  buff_zone.max_x_ = 0.339022964239;
  buff_zone.max_y_ = 2.08031368256;
  buff_zone.min_x_ = -0.413164108992;
  buff_zone.min_y_ = 1.41502892971;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(3, buff_zone));

  buff_zone.max_x_ = 3.88713097572;
  buff_zone.max_y_ = -0.004487104416;
  buff_zone.min_x_ = 3.10159683228;
  buff_zone.min_y_ = -0.78013420105;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(4, buff_zone));

  buff_zone.max_x_ = 2.58001852036;
  buff_zone.max_y_ = 0.871861934662;
  buff_zone.min_x_ = 1.77950155735;
  buff_zone.min_y_ = 0.00792863965;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(5, buff_zone));

  buff_zone.max_x_ = 0.449780195951;
  buff_zone.max_y_ = -1.28938426971;
  buff_zone.min_x_ = -0.435091942549;
  buff_zone.min_y_ = -2.09911131859;
  map_buff_zones_.insert(std::map<int, BuffZone>::value_type(6, buff_zone));
}

void StaticLayer::MatchSize() {
  if (!layered_costmap_->IsRolling()) {
    Costmap2D* master = layered_costmap_->GetCostMap();
    ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), master->GetResolution(),
              master->GetOriginX(), master->GetOriginY());
  }
}

void StaticLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
  unsigned int temp_index = 0;
  unsigned char value = 0;
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  auto resolution = new_map->info.resolution;
  auto origin_x = new_map->info.origin.position.x;
  auto origin_y = new_map->info.origin.position.y;
  auto master_map = layered_costmap_->GetCostMap();
  if (!layered_costmap_->IsRolling() && (master_map->GetSizeXCell() != size_x || master_map->GetSizeYCell() != size_y ||
      master_map->GetResolution() != resolution || master_map->GetOriginX() != origin_x
      || master_map->GetOriginY() != origin_y ||
      !layered_costmap_->IsSizeLocked())) {
    layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y, true);
  } else if (size_x_ != size_x || size_y_ != size_y || resolution_ != resolution || origin_x_ != origin_x
      || origin_y_ != origin_y) {
    ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }

  for (auto i = 0; i < size_y; i++) {
    for (auto j = 0; j < size_x; j++) {
      value = new_map->data[temp_index];
      costmap_[temp_index] = InterpretValue(value);
      ++temp_index;
    }
  }

  map_received_ = true;
  has_updated_data_ = true;
  map_frame_ = new_map->header.frame_id;
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  if (first_map_only_) {
    map_sub_.shutdown();
  }
}

void StaticLayer::UpdateBuffZoneStatus(const roborts_msgs::BuffZoneStatus &new_buff_zone_status) {
  if (!map_received_) {
    return;
  }
  auto fill_buffer_zone = [&](BuffZone buff_zone, unsigned char value) {
    unsigned int map_min_x;
    unsigned int map_min_y;
    unsigned int map_max_x;
    unsigned int map_max_y;
    World2Map(buff_zone.min_x_ + buff_zone_smaller_, buff_zone.min_y_ + buff_zone_smaller_, map_min_x, map_min_y);
    World2Map(buff_zone.max_x_ - buff_zone_smaller_, buff_zone.max_y_ - buff_zone_smaller_, map_max_x, map_max_y);
    for (unsigned int i = map_min_x; i < map_max_x; ++i) {
      for (unsigned int j = map_min_y; j < map_max_y; ++j) {
        if (i < 0 || i >= size_x_) {
          continue;
        }
        if (j < 0 || j >= size_y_) {
          continue;
        }
        costmap_[GetIndex(i, j)] = value;
      }
    }
  };

  std::cout << "Robot color: " << robot_color_ << std::endl;
  std::vector<uint8_t> vec_buff_debuff_status{0};
  vec_buff_debuff_status.emplace_back(new_buff_zone_status.F1_zone_buff_debuff_status);
  vec_buff_debuff_status.emplace_back(new_buff_zone_status.F2_zone_buff_debuff_status);
  vec_buff_debuff_status.emplace_back(new_buff_zone_status.F3_zone_buff_debuff_status);
  vec_buff_debuff_status.emplace_back(new_buff_zone_status.F4_zone_buff_debuff_status);
  vec_buff_debuff_status.emplace_back(new_buff_zone_status.F5_zone_buff_debuff_status);
  vec_buff_debuff_status.emplace_back(new_buff_zone_status.F6_zone_buff_debuff_status);

  for (int i = 1; i < vec_buff_debuff_status.size(); ++i) {
    auto status = vec_buff_debuff_status.at(i);
    if (status == 1 || status == 2) {
      if (robot_color_ == "red") {
        fill_buffer_zone(map_buff_zones_.at(i), FREE_SPACE);
      }
      if (robot_color_ == "blue") {
        fill_buffer_zone(map_buff_zones_.at(i), LETHAL_OBSTACLE);
      }
    } else if (status == 3 || status == 4) {
      if (robot_color_ == "red") {
        fill_buffer_zone(map_buff_zones_.at(i), LETHAL_OBSTACLE);
      }
      if (robot_color_ == "blue") {
        fill_buffer_zone(map_buff_zones_.at(i), FREE_SPACE);
      }
    } else if (status == 5 || status == 6) {
      fill_buffer_zone(map_buff_zones_.at(i), LETHAL_OBSTACLE);
    }
  }
  std::cout << "Update buff zone status successfully!" << std::endl;
}

unsigned char StaticLayer::InterpretValue(unsigned char value) {
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::Activate() {
  OnInitialize();
}

void StaticLayer::Deactivate() {
//    delete cost_map_;
  //shut down the map topic message subscriber
  map_sub_.shutdown();
}

void StaticLayer::Reset() {
  if (first_map_only_) {
    has_updated_data_ = true;
  } else {
    OnInitialize();
  }
}

void StaticLayer::UpdateBounds(double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double* min_x,
                               double* min_y,
                               double* max_x,
                               double* max_y) {
  double wx, wy;
  if (!layered_costmap_->IsRollingWindow()) {
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }
  //just make sure the value is normal
  UseExtraBounds(min_x, min_y, max_x, max_y);
  Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  Map2World(staic_layer_x_ + width_, staic_layer_y_ + height_, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
  has_updated_data_ = false;
}

void StaticLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!map_received_) {
    return;
  }
  if (!layered_costmap_->IsRollingWindow()) {
    if (!use_maximum_) {
      UpdateOverwriteByAll(master_grid, min_i, min_j, max_i, max_j);
    } else {
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    unsigned int mx, my;
    double wx, wy;
    tf::StampedTransform temp_transform;
    try {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), temp_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    for (auto i = min_i; i < max_i; ++i) {
      for (auto j = min_j; j < max_j; ++j) {
        layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
        tf::Point p(wx, wy, 0);
        p = temp_transform(p);
        if (World2Map(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.SetCost(i, j, GetCost(mx, my));
          } else {
            master_grid.SetCost(i, j, std::max(master_grid.GetCost(i, j), GetCost(i, j)));
          }
        }
      }
    }
  }
}

} //namespace roborts_costmap

