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

#ifndef MAP_HELPERS_OSM_TILE_HELPER_H
#define MAP_HELPERS_OSM_TILE_HELPER_H

#include <vector>
#include <string>

#include <tas_proj/gps_utm_converter.h>

#include <map_helpers/tile_info.h>

namespace tas
{
namespace visualization
{
class OsmTileHelper
{
public:
  std::vector<TileInfo> getTileInfo(const tas::proj::GpsCoord& gps_coord, unsigned int zoom,
                                    const std::string& url_template, int neighbors = 0);

  inline Point2d tileToPoint(int tile_x, int tile_y, unsigned int zoom);

  static std::string generateUrl(const std::string& url_template, int tile_x, int tile_y, unsigned int zoom);

  // Taken from https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#C.2FC.2B.2B
  static int longToTileX(double lon, unsigned int z);
  static int latToTileY(double lat, unsigned int z);
  static double tileXToLong(int x, int unsigned z);
  static double tileYToLat(int y, int unsigned z);

private:
  tas::proj::GpsUtmConverter gps_utm_converter_;
};
}  // namespace visualization
}  // namespace tas

#endif  // MAP_HELPERS_OSM_TILE_HELPER_H
