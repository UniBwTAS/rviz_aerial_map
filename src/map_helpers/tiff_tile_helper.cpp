#include <QDirIterator>
#include <QStringList>

#include <Eigen/Core>

#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <boost/geometry.hpp>

#include <tas_proj/gps_converter.h>
#include <tas_proj/gps_utm_converter.h>

#include <map_helpers/tiff_tile_helper.h>

namespace tas
{
namespace visualization
{

TiffTileHelper::TiffTileHelper(const std::vector<std::string>& search_paths)
{
    parseFolder(search_paths);
}

void TiffTileHelper::parseFolder(const std::vector<std::string>& search_paths)
{
    for (const auto& path : search_paths)
    {
        QDirIterator it(QString::fromStdString(path),
                        QStringList() << "*.jpg"
                                      << "*.tif",
                        QDir::Files,
                        QDirIterator::Subdirectories);
        while (it.hasNext())
        {
            storeTileData(it.next().toStdString());
        }
    }
}

void TiffTileHelper::storeTileData(const std::string& filename)
{
    tile_data_.emplace_back(filename, extractImagePolygonFromGeoTiff(filename));
}

Polygon TiffTileHelper::extractImagePolygonFromGeoTiff(const std::string& filename)
{
    GDALAllRegister();

    auto* data_set = static_cast<GDALDataset*>(GDALOpen(filename.c_str(), GA_ReadOnly));
    if (data_set == nullptr)
    {
        return Polygon();
    }

    std::array<double, 6> adf_geo_transform{};
    data_set->GetGeoTransform(adf_geo_transform.data());

    GDALRasterBand* raster_band = data_set->GetRasterBand(1);
    if (!raster_band)
    {
        return Polygon();
    }
    raster_band->GetYSize();
    Eigen::Vector2d top_left(adf_geo_transform[0], adf_geo_transform[3]);
    Eigen::Vector2d top_right(adf_geo_transform[0] + adf_geo_transform[1] * raster_band->GetXSize(),
                              adf_geo_transform[3]);
    Eigen::Vector2d bottom_right(adf_geo_transform[0] + adf_geo_transform[1] * raster_band->GetXSize(),
                                 adf_geo_transform[3] + adf_geo_transform[5] * raster_band->GetYSize());
    Eigen::Vector2d bottom_left(adf_geo_transform[0],
                                adf_geo_transform[3] + adf_geo_transform[5] * raster_band->GetYSize());

    tas::proj::GpsCoord top_left_projected, bottom_left_projected, bottom_right_projected, top_right_projected;

    char* projection_reference = const_cast<char*>(data_set->GetProjectionRef());

    OGRSpatialReference ref;
    ref.importFromWkt(&projection_reference);

    char* proj_string = nullptr;
    if (ref.exportToProj4(&proj_string) == OGRERR_NONE)
    {
        tas::proj::GpsConverter gps_converter;
        if (gps_converter.init(std::string(proj_string)))
        {
            gps_converter.toGps(top_left_projected, top_left);
            gps_converter.toGps(bottom_left_projected, bottom_left);
            gps_converter.toGps(bottom_right_projected, bottom_right);
            gps_converter.toGps(top_right_projected, top_right);
        }
    }

    GDALClose(data_set);

    tas::proj::GpsUtmConverter gps_utm_converter;
    tas::proj::UtmCoord result;

    Polygon enclosure;

    gps_utm_converter.gpsToUtm(top_left_projected, result);
    bg::append(enclosure.outer(), Point2d(result.east, result.north));

    gps_utm_converter.gpsToUtm(bottom_left_projected, result);
    bg::append(enclosure.outer(), Point2d(result.east, result.north));

    gps_utm_converter.gpsToUtm(bottom_right_projected, result);
    bg::append(enclosure.outer(), Point2d(result.east, result.north));

    gps_utm_converter.gpsToUtm(top_right_projected, result);
    bg::append(enclosure.outer(), Point2d(result.east, result.north));

    // close ring
    gps_utm_converter.gpsToUtm(top_left_projected, result);
    bg::append(enclosure.outer(), Point2d(result.east, result.north));

    return enclosure;
}

std::vector<TileInfo> TiffTileHelper::getTileInfo(const tas::proj::GpsCoord& gps_coord, double region)
{
    std::vector<TileInfo> tile_infos;
    for (auto& element : tile_data_)
    {
        tas::proj::UtmCoord utm;
        gps_utm_converter_.gpsToUtm(gps_coord, utm);
        if (containsUtmCoord(element, utm, region))
        {
            tile_infos.push_back(element);
        }
    }
    return tile_infos;
}

bool TiffTileHelper::containsUtmCoord(const TileInfo& image_data, const tas::proj::UtmCoord& utm_coord, double region)
{
    Point2d point(utm_coord.east, utm_coord.north);

    Polygon enlarged_point;
    bg::append(enlarged_point.outer(), Point2d(point.x() - region, point.y() + region));
    bg::append(enlarged_point.outer(), Point2d(point.x() - region, point.y() - region));
    bg::append(enlarged_point.outer(), Point2d(point.x() + region, point.y() - region));
    bg::append(enlarged_point.outer(), Point2d(point.x() + region, point.y() + region));
    bg::append(enlarged_point.outer(), Point2d(point.x() - region, point.y() + region));

    return bg::intersects(enlarged_point, image_data.enclosure);
}

} // namespace visualization
} // namespace tas
