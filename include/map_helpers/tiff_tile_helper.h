#pragma once

#include <vector>

#include <tas_proj/gps_utm_converter.h>

#include <map_helpers/tile_info.h>

namespace tas
{
namespace visualization
{

class TiffTileHelper
{
  public:
    explicit TiffTileHelper(const std::vector<std::string>& search_paths);

    std::vector<TileInfo> getTileInfo(const tas::proj::GpsCoord& gps_coord, double region);

  private:
    void parseFolder(const std::vector<std::string>& search_paths);
    void storeTileData(const std::string& filename);
    static Polygon extractImagePolygonFromGeoTiff(const std::string& filename);

    static bool containsUtmCoord(const TileInfo& image_data, const tas::proj::UtmCoord& utm_coord, double region);

  private:
    std::vector<std::string> default_search_paths_;
    std::vector<TileInfo> tile_data_;
    tas::proj::GpsUtmConverter gps_utm_converter_;
};

} // namespace visualization
} // namespace tas
