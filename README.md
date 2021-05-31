# rviz_aerial_map

Plugin for rviz for displaying aerial images. It allows to visualize your own geo-referenced images as well as map tiles
from OpenStreetMap and similar.

![Alt text](.screenshot.png?raw=true "Example Image")

In order to use rviz_aerial_map, add this package to your catkin workspace. Additionally few more dependencies are 
required:

* GDAL: C++ library for extracting GPS position of geo-referenced images
  * `rosdep install --ignore-src rviz_aerial_map` or `sudo apt install libgdal-dev`
* [tas_proj](https://github.com/UniBwTAS/tas_proj): C++ library based on PROJ for converting between geodetic/geographic coordinate systems
  * clone it into your catkin workspace
* [ogre_primitives](https://github.com/UniBwTAS/ogre_primitives): C++ library for Ogre drawing primitives
  * clone it into your catkin workspace
* utm (optional): Python library to convert gps to utm
  * `pip install utm`

## Demo

The package contains a launch file for demonstration purposes.
Use it to verify your installation and to get started:

```
roslaunch rviz_aerial_image demo.launch
```

The launch file plays the sample rosbag containing NavSatFix messages and displays Rviz together with this plugin.

Check the Usage section below to learn how to use the position of your robot and a satellite map.

## Usage

Add an instance of `AerialMap` to your rviz config.

The `Topic` field must point to a publisher of `sensor_msgs/NavSatFix`. This plugin requires an UTM frame in the TF tree, e.g. `utm` -> `odom` -> `base_link`. If you don't have such a TF tree you can use:
```
rosrun rviz_aerial_map generate_tf_utm.py
```
it simply generates the TF transform utm -> base_link from `sensor_msgs/NavSatFix`.

Map tiles will be cached to `$HOME/.cache/rviz_aerial_image`.
At present the cache does not expire automatically - you should delete the files in the folder if you want the images to be reloaded.

Currently, we only support the [OpenStreetMap](http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames) convention for tile URLs.
This e.g. implies that only raster tiles (no vector tiles) are supported.

## Tile servers

You must provide a tile URL (Object URI) from which the satellite images are loaded.
The URL should have the form `http://server.tld/{z}/{x}/{y}.jpg`.
Where the tokens `{z}`, `{x}`, `{y}` represent the zoom level, x coordinate, and y coordinate respectively.
These will automatically be substituted by rviz_aerial_image when making HTTP requests.

rviz_aerial_image doesn't come with any preconfigured tile URL.
For example, you could use one of the following tile servers:

* OpenStreetMap: https://tile.openstreetmap.org/{z}/{x}/{y}.png
* TomTom: https://api.tomtom.com/map/1/tile/basic/main/{z}/{x}/{y}.png?tileSize=512&key=[TOKEN]
* Mapbox: https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}?access_token=[TOKEN]

For some of these, you have to request an access token first.
Please refer to the respective terms of service and copyrights.

## Options

- `Topic`: the topic of the GPS measurements.
- `UTM frame`: the name of the utm frame.
- `Alpha`: simply the display transparency.
- `Draw behind`: whether to draw map behind all other geometry.
- `Height type`: select at which height the map should be drawn.
- `Height offset`: allows to set a small height offset.
- `Map type`: whether to visualize geo-referenced images on local disk or to use a online service such as OpenStreetMap.
- `URL` / `Filepath`: path to file or tile URL (see above)
- `Zoom` is the zoom level of the map. Recommended values are 16-19, as anything smaller is _very_ low resolution. 22 is the current max.
- `Blocks` number of adjacent blocks to load. rviz_aerial_image will load the central block, and this many blocks around the center. 8 is the current max.

## Support and Contributions

In case of questions or problems, do not hesitate to open an issue.

Contributions are welcomed. Please add a summary of your changes to the [changelog](CHANGELOG.rst) under the section Forthcoming.