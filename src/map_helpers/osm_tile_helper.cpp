#include <boost/geometry.hpp>

#include <QtCore/QString>

#include <iostream>
#include <map_helpers/osm_tile_helper.h>

namespace tas
{
namespace visualization
{

std::vector<TileInfo> OsmTileHelper::getTileInfo(const tas::proj::GpsCoord& gps_coord,
                                                 unsigned int zoom,
                                                 const std::string& url_template,
                                                 int neighbors)
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

    return {utm.east, utm.north};
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

} // namespace visualization
} // namespace tas
