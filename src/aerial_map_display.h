/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Institute for Autonomous Systems Technology
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
 *********************************************************************/

#pragma once

#include <boost/optional.hpp>

#include <ros/ros.h>

#include <rviz/display.h>
#include <rviz/ogre_primitives/map_tile.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#endif
#include <rviz/message_filter_display.h>

#include <OGRE/OgreMaterial.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>

#include <map_helpers/osm_tile_helper.h>
#include <map_helpers/tiff_tile_helper.h>
#include <tas_proj/geoid_converter.h>
#include <tas_proj/gps_utm_converter.h>

#include "vis_helpers/texture_cache.h"

namespace rviz
{
class FloatProperty;
class IntProperty;
class RosTopicProperty;
class StringProperty;
class BoolProperty;
class EnumProperty;
class TfFrameProperty;

class AerialImageDisplay : public rviz::MessageFilterDisplay<sensor_msgs::NavSatFix>
{
  Q_OBJECT
public:
  enum
  {
    MAP_TYPE_OSM,
    MAP_TYPE_TIFF,
  };

  enum
  {
    HEIGHT_TYPE_EGM96,
    HEIGHT_TYPE_WGS84,
    HEIGHT_TYPE_ZERO,
    HEIGHT_TYPE_FROM_TF_FRAME,
  };

public:
  AerialImageDisplay();


  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void propertyChanged();

protected:
  // overrides from MessageFilterDisplay
  void onEnable() override;
  void onInitialize() override;
  void processMessage(const sensor_msgs::NavSatFix::ConstPtr& msg) override;
  void fixedFrameChanged() override;

  void loadImagery();
  void assembleScene();
  void clear();
  void clearGeometry();
  void createGeometry();
  bool applyTransforms(bool force_new_ref = false);
  bool getAxisAlignedPoseInUtmFrame(geometry_msgs::Pose& out);
  float getHeightOfTfInUtmFrame(const std::string& tf);

  std::vector<std::shared_ptr<rviz::MapTile>> objects_;
  std::vector<Ogre::MaterialPtr> materials_;

  // general properties
  TfFrameProperty* utm_frame_property_;
  FloatProperty* alpha_property_;
  BoolProperty* draw_under_property_;
  EnumProperty* height_type_property_;
  TfFrameProperty* height_tf_frame_property_;
  FloatProperty* height_offset_property_;
  EnumProperty* map_type_property_;
  std::string utm_frame_;
  float alpha_{};
  bool draw_under_{};
  int height_type_{};
  std::string height_tf_frame_;
  float height_offset_{};
  int map_type_{};

  // osm specific properties
  StringProperty* tile_url_property_;
  IntProperty* zoom_property_;
  IntProperty* blocks_property_;
  std::string tile_url_;
  unsigned int zoom_{};
  int blocks_{};

  // tiff specific properties
  StringProperty* tile_uri_property_;
  FloatProperty* roi_property_;
  std::string tile_uri_;
  float roi_{};

  // tile management
  tas::proj::GpsUtmConverter gps_utm_converter_;
  tas::proj::GeoidConverter geoid_converter_;
  std::unique_ptr<tas::visualization::TiffTileHelper> tiff_tile_helper_;
  std::unique_ptr<tas::visualization::OsmTileHelper> osm_tile_helper_;
  geometry_msgs::PosePtr ref_pose_;
  Ogre::SceneNode* tile_node_ = nullptr;
  bool dirty_;
  sensor_msgs::NavSatFixConstPtr last_msg_;
  std::unique_ptr<TextureCache> texture_cache_;
  boost::optional<std::vector<tas::visualization::TileInfo>> tile_infos_;
};

}  // namespace rviz
