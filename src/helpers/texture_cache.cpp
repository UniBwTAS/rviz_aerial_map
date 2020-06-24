#include <functional>
#include <OGRE/OgreTextureManager.h>
#include <QImage>
#include <ros/console.h>

#include "texture_cache.h"

TextureCache::TextureCache(bool uri_is_url)
    : uri_is_url_(uri_is_url),
      downloader_([this](const std::string& uri, const QImage& image) { imageLoadedGuarded(uri, image); })
{
}

TextureCache::~TextureCache()
{
    std::lock_guard<std::mutex> lock_guard(mutex);

    for (auto& cached_texture : cached_textures_)
    {
        Ogre::TexturePtr ptr = std::get<2>(cached_texture.second);
        if (!ptr.isNull())
        {
            Ogre::TextureManager::getSingleton().remove(ptr->getName());
        }
    }
}

bool TextureCache::request(const std::vector<std::string>& uris)
{
    std::lock_guard<std::mutex> lock_guard(mutex);

    // load images if not requested so far
    bool is_dirty = false;
    for (const std::string& uri : uris)
    {
        auto it = cached_textures_.find(uri);
        bool load_image = false;
        if (it == cached_textures_.end())
        {
            ROS_DEBUG_STREAM("LOADING [NOT LISTED]: " << uri);
            load_image = true;
        }
        else
        {
            int status = std::get<0>(it->second);
            double elapsed_time = static_cast<double>(clock() - std::get<1>(it->second)) / CLOCKS_PER_SEC;
            if ((status == STATUS_ERROR || status == STATUS_LOADING) && elapsed_time > 3.)
            {
                ROS_DEBUG_STREAM("LOADING [TIMEOUT]: " << uri << "(" << elapsed_time << ")");
                cached_textures_.erase(it);
                load_image = true;
            }
        }
        if (load_image)
        {
            is_dirty = true;
            int status = STATUS_LOADING;
            cached_textures_.insert({uri, {status, clock(), Ogre::TexturePtr()}});
            if (uri_is_url_)
            {
                downloader_.loadFile(uri);
            }
            else
            {
                QImage img(QString::fromStdString(uri), nullptr);
                imageLoaded(uri, img);
            }
        }
    }

    // purge remaining textures
    for (auto it = cached_textures_.begin(); it != cached_textures_.end();)
    {
        if (std::find(uris.begin(), uris.end(), it->first) == uris.end())
        {
            Ogre::TexturePtr ptr = std::get<2>(it->second);
            if (!ptr.isNull())
            {
                Ogre::TextureManager::getSingleton().remove(ptr->getName());
            }
            it = cached_textures_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    return is_dirty;
}

Ogre::TexturePtr TextureCache::ready(const std::string& uri) const
{
    std::lock_guard<std::mutex> lock_guard(mutex);

    auto const it = cached_textures_.find(uri);

    if (it == cached_textures_.cend())
    {
        return Ogre::TexturePtr();
    }

    return std::get<2>(it->second);
}

Ogre::TexturePtr TextureCache::textureFromImage(const QImage& image, const std::string& name)
{
    Ogre::DataStreamPtr data_stream;
    data_stream.bind(new Ogre::MemoryDataStream((void*)image.constBits(), image.byteCount()));

    Ogre::String const res_group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

    Ogre::TextureManager& texture_manager = Ogre::TextureManager::getSingleton();

    // swap byte order when going from QImage to Ogre
    return texture_manager.loadRawData(
        name, res_group, data_stream, image.width(), image.height(), Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);
}

QImage TextureCache::convertImage(const QImage& image)
{
    return image.convertToFormat(QImage::Format_RGB888).mirrored();
}

void TextureCache::imageLoaded(const std::string& uri, const QImage& image)
{
    auto it = cached_textures_.find(uri);
    if (it != cached_textures_.end())
    {
        if (!image.isNull())
        {
            ROS_DEBUG_STREAM("SUCCESS: " << uri);
            std::get<2>(it->second) = textureFromImage(convertImage(image), uri);
            std::get<0>(it->second) = STATUS_FINISHED;
        }
        else
        {
            ROS_DEBUG_STREAM("ERROR: " << uri << std::endl);
            std::get<0>(it->second) = STATUS_ERROR;
        }
    }
}

void TextureCache::imageLoadedGuarded(const std::string& uri, const QImage& image)
{
    std::lock_guard<std::mutex> lock_guard(mutex);
    imageLoaded(uri, image);
}
