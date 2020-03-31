#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <OGRE/OgreImageCodec.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreVector3.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/grid.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>

#include "aerial_map_display.h"

#include <boost/geometry/algorithms/num_points.hpp>

namespace rviz
{
AerialMapDisplay::AerialMapDisplay() : Display(), messages_received_(0), dirty_(false)
{
    // general properties

    topic_property_ =
        new RosTopicProperty("Topic",
                             "",
                             QString::fromStdString(ros::message_traits::datatype<sensor_msgs::NavSatFix>()),
                             "sensor_msgs::NavSatFix topic to subscribe to.",
                             this,
                             SLOT(updateTopic()));

    map_frame_property_ = new TfFrameProperty(
        "Map frame", "map", "Specify any world fixed frame.", this, nullptr, false, SLOT(propertyChanged()));

    alpha_property_ =
        new FloatProperty("Alpha", 1, "Amount of transparency to apply to the map.", this, SLOT(propertyChanged()));
    alpha_property_->setMin(0);
    alpha_property_->setMax(1);

    draw_under_property_ = new BoolProperty("Draw behind",
                                            false,
                                            "Rendering option, controls whether or not the map is always"
                                            " drawn behind everything else.",
                                            this,
                                            SLOT(propertyChanged()));

    height_offset_property_ = new FloatProperty(
        "Height offset",
        0,
        "Elevation offset of the map. The map is drawn at the height defined above. However this is typically not "
        "the same height as the bottom of the robot.",
        this,
        SLOT(propertyChanged()));

    map_type_property_ = new EnumProperty("Map type",
                                          "Open Street Map",
                                          "Whether to use the images from Open Street Map (and compatible "
                                          "alternatives) or georeferenced images from local disk. For the latter all "
                                          "types can be used, which are supported by the GDAL library.",
                                          this,
                                          SLOT(propertyChanged()));
    map_type_property_->addOption("Open Street Map", MAP_TYPE_OSM);
    map_type_property_->addOption("Georeferenced Images", MAP_TYPE_TIFF);

    // osm specific properties

    QString const url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
    tile_url_property_ =
        new StringProperty("URL", url, "URL from which to retrieve map tiles.", this, SLOT(propertyChanged()));

    QString const zoom_desc = QString::fromStdString("Zoom level (0 - " + std::to_string(22) + ")");
    zoom_property_ = new IntProperty("Zoom", 18, zoom_desc, this, SLOT(propertyChanged()));
    zoom_property_->setMin(0);
    zoom_property_->setMax(22);

    QString const blocks_desc = QString::fromStdString("Adjacent blocks (0 - " + std::to_string(10) + ")");
    blocks_property_ = new IntProperty("Blocks", 3, blocks_desc, this, SLOT(propertyChanged()));
    blocks_property_->setMin(0);
    blocks_property_->setMax(10);

    // tiff speciffic properties
    tile_uri_property_ = new StringProperty(
        "Filepath", "", "Directory from which to retrieve map tiles.", this, SLOT(propertyChanged()));

    roi_property_ = new FloatProperty("ROI",
                                      30,
                                      "Side length of the region of interest (ROI) square. Its center is at the "
                                      "NavSatFix position and all tiles that intersect this square are displayed.",
                                      this,
                                      SLOT(propertyChanged()));
}

AerialMapDisplay::~AerialMapDisplay()
{
    unsubscribe();
    clear();
}

void AerialMapDisplay::onInitialize()
{
    Display::onInitialize();

    tile_node_ = scene_node_->createChildSceneNode();
    map_frame_property_->setFrameManager(context_->getFrameManager());
}

void AerialMapDisplay::onEnable()
{
    subscribe();
    propertyChanged();
}

void AerialMapDisplay::onDisable()
{
    unsubscribe();
    clear();
}

void AerialMapDisplay::subscribe()
{
    if (!isEnabled())
    {
        return;
    }

    if (!topic_property_->getTopic().isEmpty())
    {
        try
        {
            ROS_INFO("Subscribing to %s", topic_property_->getTopicStd().c_str());
            coord_sub_ =
                update_nh_.subscribe(topic_property_->getTopicStd(), 1, &AerialMapDisplay::navFixCallback, this);

            setStatus(StatusProperty::Ok, "Topic", "OK");
        }
        catch (ros::Exception& e)
        {
            setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
        }
    }
}

void AerialMapDisplay::unsubscribe()
{
    coord_sub_.shutdown();
    ROS_INFO("Unsubscribing.");
}

void AerialMapDisplay::propertyChanged()
{
    map_frame_ = map_frame_property_->getStdString();
    alpha_ = alpha_property_->getFloat();
    draw_under_ = draw_under_property_->getBool();
    height_offset_ = height_offset_property_->getFloat();
    map_type_ = map_type_property_->getOptionInt();

    tile_url_ = tile_url_property_->getStdString();
    zoom_ = zoom_property_->getInt();
    blocks_ = blocks_property_->getInt();

    tile_uri_ = tile_uri_property_->getStdString();
    roi_ = roi_property_->getFloat();

    if (map_type_ == MAP_TYPE_TIFF)
    {
        tile_url_property_->hide();
        zoom_property_->hide();
        blocks_property_->hide();

        tile_uri_property_->show();
        roi_property_->show();

        // TAS specific path
        if (tile_uri_.length() == 0)
        {
            const char* path = std::getenv("MUCAR_HOME");
            if (path != NULL)
            {
                tile_uri_ = std::string(path) + "/data/aerial_images";
            }
        }

        texture_cache_.reset(new TextureCache(false));
        osm_tile_helper_.reset();
        tiff_tile_helper_.reset(new tas::visualization::TiffTileHelper({tile_uri_}));
    }
    else if (map_type_ == MAP_TYPE_OSM)
    {
        tile_url_property_->show();
        zoom_property_->show();
        blocks_property_->show();

        tile_uri_property_->hide();
        roi_property_->hide();

        texture_cache_.reset(new TextureCache(true));
        osm_tile_helper_.reset(new tas::visualization::OsmTileHelper());
        tiff_tile_helper_.reset();
    }

    dirty_ = true;

    applyTransforms(true);
    loadImagery();
    assembleScene();

    context_->queueRender();
}

void AerialMapDisplay::updateTopic()
{
    unsubscribe();
    clear();
    subscribe();
}

void AerialMapDisplay::clear()
{
    setStatus(StatusProperty::Warn, "NavSatFix", "No NavSatFix received");
    setStatus(StatusProperty::Warn, "Tiles", "No image tiles loaded");
    clearGeometry();
    dirty_ = true;
}

void AerialMapDisplay::clearGeometry()
{
    if (tile_node_)
    {
        tile_node_->removeAndDestroyAllChildren();
    }
    objects_.clear();
    materials_.clear();
}

void AerialMapDisplay::createGeometry()
{
    for (int block = 0; block < (2 * blocks_ + 1) * (2 * blocks_ + 1); ++block)
    {
        // generate an unique name
        static int count = 0;
        std::string const name_suffix = std::to_string(count);
        ++count;

        // one material per texture
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
            "satellite_material_" + name_suffix, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->setReceiveShadows(false);
        material->getTechnique(0)->setLightingEnabled(false);
        material->setDepthBias(-16.0f, 0.0f);
        material->setCullingMode(Ogre::CULL_NONE);
        material->setDepthWriteEnabled(false);

        // create texture and initialize it
        Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
        tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);

        materials_.push_back(material);

        // create an object
        auto tile = std::make_shared<rviz::MapTile>();
        tile->setVisible(false);
        tile_node_->attachObject(tile.get());
        objects_.push_back(tile);

        assert(!material.isNull());
    }
}

void AerialMapDisplay::update(float, float)
{
    // create all geometry, if necessary
    assembleScene();

    // draw
    context_->queueRender();
}

void AerialMapDisplay::navFixCallback(sensor_msgs::NavSatFixConstPtr const& msg)
{
    ++messages_received_;
    setStatus(StatusProperty::Ok, "NavSatFix", QString::number(messages_received_) + " InstanceArrays received");

    last_msg_ = msg;

    // re-load imagery
    applyTransforms();
    loadImagery();
}

void AerialMapDisplay::loadImagery()
{
    if (!isEnabled() || !last_msg_)
    {
        return;
    }

    tas::proj::GpsCoord gps;
    gps.lat = last_msg_->latitude;
    gps.lon = last_msg_->longitude;

    if (map_type_ == MAP_TYPE_OSM)
    {
        tile_infos_ = osm_tile_helper_->getTileInfo(gps, zoom_, tile_url_, blocks_);
    }
    else if (map_type_ == MAP_TYPE_TIFF)
    {
        tile_infos_ = tiff_tile_helper_->getTileInfo(gps, roi_ / 2.);
    }

    std::vector<std::string> urls;
    std::transform((*tile_infos_).begin(),
                   (*tile_infos_).end(),
                   std::back_inserter(urls),
                   [](const tas::visualization::TileInfo& tile_info) -> std::string { return tile_info.filename; });

    dirty_ = texture_cache_->request(urls);
}

void AerialMapDisplay::assembleScene()
{
    if (!isEnabled() || !dirty_ || !tile_infos_ || !ref_pose_ || !last_msg_)
    {
        return;
    }
    dirty_ = false;

    clearGeometry();
    createGeometry();

    size_t num_tiles_found = 0;

    int tile_idx = 0;
    for (const tas::visualization::TileInfo& tile_info : *tile_infos_)
    {
        auto& obj = objects_[tile_idx];
        auto& material = materials_[tile_idx];
        assert(!material.isNull());

        Ogre::TexturePtr texture = texture_cache_->ready(tile_info.filename);
        if (texture.isNull())
        {
            // don't show tiles with old textures
            obj->setVisible(false);
            tile_idx++;
            continue;
        }
        num_tiles_found++;

        obj->setVisible(true);

        // update texture
        Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->getTextureUnitState(0);
        tex_unit->setTextureName(texture->getName());

        // configure depth & alpha properties
        if (alpha_ >= 0.9998)
        {
            material->setDepthWriteEnabled(!draw_under_);
            material->setSceneBlending(Ogre::SBT_REPLACE);
        }
        else
        {
            material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
            material->setDepthWriteEnabled(false);
        }

        if (draw_under_)
        {
            // render under everything else
            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
        }
        else
        {
            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
        }

        tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha_);

        obj->setMaterial(material->getName());

        tas::visualization::Polygon polygon = tile_info.enclosure;
        auto it = boost::begin(boost::geometry::exterior_ring(polygon));

        auto top_left_x = static_cast<float>(boost::geometry::get<0>(*it) - ref_utm_.east);
        auto top_left_y = static_cast<float>(boost::geometry::get<1>(*it++) - ref_utm_.north);
        auto bottom_left_x = static_cast<float>(boost::geometry::get<0>(*it) - ref_utm_.east);
        auto bottom_left_y = static_cast<float>(boost::geometry::get<1>(*it++) - ref_utm_.north);
        auto bottom_right_x = static_cast<float>(boost::geometry::get<0>(*it) - ref_utm_.east);
        auto bottom_right_y = static_cast<float>(boost::geometry::get<1>(*it++) - ref_utm_.north);
        auto top_right_x = static_cast<float>(boost::geometry::get<0>(*it) - ref_utm_.east);
        auto top_right_y = static_cast<float>(boost::geometry::get<1>(*it) - ref_utm_.north);

        obj->setParameters(top_left_x,
                           top_left_y,
                           bottom_left_x,
                           bottom_left_y,
                           bottom_right_x,
                           bottom_right_y,
                           top_right_x,
                           top_right_y,
                           0);

        tile_idx++;
    }

    // since not all tiles were loaded, this function has to be called again
    if (num_tiles_found == 0)
    {
        setStatus(StatusProperty::Warn, "Tiles", "No image tiles loaded");
        dirty_ = true;
    }
    else if (num_tiles_found < tile_infos_->size())
    {
        // some tiles were not found
        // lets print no warning as this happens frequently when a new tile is entered
        setStatus(StatusProperty::Ok, "Tiles", "Some image tiles missing");
        dirty_ = true;
    }
    else
    {
        setStatus(StatusProperty::Ok, "Tiles", "All image tiles loaded");
    }

    std::vector<std::string> urls;
    std::transform((*tile_infos_).begin(),
                   (*tile_infos_).end(),
                   std::back_inserter(urls),
                   [](const tas::visualization::TileInfo& tile_info) -> std::string { return tile_info.filename; });
}

bool AerialMapDisplay::applyTransforms(bool force_new_ref)
{
    if (!isEnabled() || !last_msg_)
    {
        return false;
    }

    geometry_msgs::Pose cur_pose;
    if (!getAxisAlignedPoseInMapFrame(cur_pose))
    {
        return false;
    }

    double distance = 0;
    if (ref_pose_)
    {
        tf2::Vector3 cur_pos, ref_pos;
        tf2::fromMsg(cur_pose.position, cur_pos);
        tf2::fromMsg(ref_pose_->position, ref_pos);

        distance = (cur_pos - ref_pos).length();
    }

    if (!ref_pose_ || distance > 5000 || force_new_ref)
    {
        gpsToUtm(*last_msg_, ref_utm_);

        ref_pose_.reset(new geometry_msgs::Pose());
        *ref_pose_ = cur_pose;

        dirty_ = true;
    }

    ref_pose_->position.z = cur_pose.position.z + height_offset_;

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->transform(map_frame_, ros::Time(), *ref_pose_, position, orientation))
    {
        setStatus(StatusProperty::Error,
                  "Transform",
                  "Could not transform from [" + QString::fromStdString(map_frame_) + "] to Fixed Frame [" +
                      fixed_frame_ + "]");
        return false;
    }
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");

    tile_node_->setPosition(position);
    tile_node_->setOrientation(orientation);

    return true;
}

void AerialMapDisplay::fixedFrameChanged()
{
    applyTransforms();
}

void AerialMapDisplay::reset()
{
    messages_received_ = 0;

    Display::reset();
    updateTopic();
}

void AerialMapDisplay::gpsToUtm(const sensor_msgs::NavSatFix& from, tas::proj::UtmCoord& to)
{
    tas::proj::GpsCoord gps;
    gps.lat = from.latitude;
    gps.lon = from.longitude;
    gps.altitude = from.altitude;

    gps_utm_converter_.gpsToUtm(gps, to);
}

bool AerialMapDisplay::getAxisAlignedPoseInMapFrame(geometry_msgs::Pose& out)
{
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = context_->getFrameManager()->getTF2BufferPtr();
    geometry_msgs::TransformStamped tf;
    try
    {
        tf = tf_buffer->lookupTransform(map_frame_, last_msg_->header.frame_id, ros::Time(0));
        out.position.x = tf.transform.translation.x;
        out.position.y = tf.transform.translation.y;
        out.position.z = tf.transform.translation.z;
        out.orientation.x = 0;
        out.orientation.y = 0;
        out.orientation.z = 0;
        out.orientation.w = 1;

        return true;
    }
    catch (tf2::TransformException& ex)
    {
        setStatus(StatusProperty::Error,
                  "Transform",
                  "Could not transform from [" + QString::fromStdString(map_frame_) + "] to NavSatFix Frame [" +
                      QString::fromStdString(last_msg_->header.frame_id) + "]");
        return false;
    }
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AerialMapDisplay, rviz::Display)
