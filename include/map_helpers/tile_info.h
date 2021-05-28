#pragma once

#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace tas
{
namespace visualization
{

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> Point2d;
typedef bg::model::polygon<Point2d> Polygon;

struct TileInfo
{
    TileInfo() : filename(""), enclosure(Polygon())
    {
    }

    TileInfo(std::string filename, Polygon enclosure) : filename(std::move(filename)), enclosure(std::move(enclosure))
    {
    }

    std::string filename;
    Polygon enclosure;
};

} // namespace visualization
} // namespace tas
