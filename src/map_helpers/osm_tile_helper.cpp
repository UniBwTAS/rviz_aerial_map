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

#include <boost/geometry.hpp>

#include <QtCore/QString>

#include <iostream>
#include <map_helpers/osm_tile_helper.h>

namespace tas
{
namespace visualization
{
std::vector<TileInfo> OsmTileHelper::getTileInfo(const tas::proj::GpsCoord& gps_coord, unsigned int zoom,
                                                 const std::string& url_template, int neighbors)
{
  std::vector<TileInfo> tile_infos;

  int center_tile_x = longToTileX(gps_coord.lon, zoom);
  int center_tile_y = latToTileY(gps_coord.lat, zoom);

  for (int x_shift = -neighbors; x_shift <= neighbors; x_shift++)
  {
    for (int y_shift = -neighbors; y_shift <= neighbors; y_shift++)
    {
      int cur_tile_x = center_tile_x + x_shift;
      int cur_tile_y = center_tile_y + y_shift;

      Polygon polygon;
      // top left
      bg::append(polygon.outer(), tileToPoint(cur_tile_x, cur_tile_y, zoom));
      // bottom left
      bg::append(polygon.outer(), tileToPoint(cur_tile_x, cur_tile_y + 1, zoom));
      // bottom right
      bg::append(polygon.outer(), tileToPoint(cur_tile_x + 1, cur_tile_y + 1, zoom));
      // top right
      bg::append(polygon.outer(), tileToPoint(cur_tile_x + 1, cur_tile_y, zoom));
      // close ring
      bg::append(polygon.outer(), tileToPoint(cur_tile_x, cur_tile_y, zoom));

      std::string url = generateUrl(url_template, cur_tile_x, cur_tile_y, zoom);
      tile_infos.emplace_back(std::move(url), std::move(polygon));
    }
  }

  return tile_infos;
}

Point2d OsmTileHelper::tileToPoint(int tile_x, int tile_y, unsigned int zoom)
{
  tas::proj::GpsCoord gps;
  gps.lat = tileYToLat(tile_y, zoom);
  gps.lon = tileXToLong(tile_x, zoom);

  tas::proj::UtmCoord utm;

  gps_utm_converter_.gpsToUtm(gps, utm);

  return { utm.east, utm.north };
}

std::string OsmTileHelper::generateUrl(const std::string& url_template, int tile_x, int tile_y, unsigned int zoom)
{
  QString url = QString::fromStdString(url_template);
  url.replace("{x}", QString::number(tile_x));
  url.replace("{y}", QString::number(tile_y));
  url.replace("{z}", QString::number(zoom));
  return url.toStdString();
}

int OsmTileHelper::longToTileX(double lon, unsigned int z)
{
  return (int)(floor((lon + 180.0) / 360.0 * (1u << z)));
}

int OsmTileHelper::latToTileY(double lat, unsigned int z)
{
  double latrad = lat * M_PI / 180.0;
  return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1u << z)));
}

double OsmTileHelper::tileXToLong(int x, int unsigned z)
{
  return x / (double)(1u << z) * 360.0 - 180;
}

double OsmTileHelper::tileYToLat(int y, int unsigned z)
{
  double n = M_PI - 2.0 * M_PI * y / (double)(1u << z);
  return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

}  // namespace visualization
}  // namespace tas
