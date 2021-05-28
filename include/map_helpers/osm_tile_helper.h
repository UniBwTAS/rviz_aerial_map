#pragma once

#include <vector>

#include <tas_proj/gps_utm_converter.h>

#include <map_helpers/tile_info.h>

namespace tas
{
namespace visualization
{
class OsmTileHelper
{
  public:
    std::vector<TileInfo> getTileInfo(const tas::proj::GpsCoord& gps_coord,
                                      unsigned int zoom,
                                      const std::string& url_template,
                                      int neighbors = 0);

    inline Point2d tileToPoint(int tile_x, int tile_y, unsigned int zoom);

    static std::string generateUrl(const std::string& url_template, int tile_x, int tile_y, unsigned int zoom);

    /** Taken from https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#C.2FC.2B.2B **/
    static int longToTileX(double lon, unsigned int z);
    static int latToTileY(double lat, unsigned int z);
    static double tileXToLong(int x, int unsigned z);
    static double tileYToLat(int y, int unsigned z);

  private:
    tas::proj::GpsUtmConverter gps_utm_converter_;
};
} // namespace visualization
} // namespace tas
