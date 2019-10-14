#include <functional>
#include <thread>
#include <utility>

#include <QImage>

#include <OGRE/OgreTextureManager.h>

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
        if (!cached_texture.second.isNull())
        {
            Ogre::TextureManager::getSingleton().remove(cached_texture.second->getName());
        }
    }
}

bool TextureCache::request(const std::vector<std::string>& uris)
{
    std::lock_guard<std::mutex> lock_guard(mutex);

    bool is_dirty = false;

    for (const std::string& uri : uris)
    {
        if (cached_textures_.find(uri) == cached_textures_.end())
        {
            is_dirty = true;
            cached_textures_.emplace(std::make_pair(uri, Ogre::TexturePtr()));
            if (uri_is_url_)
            {
                downloader_.loadFile(uri);
            }
            else
            {
                /*std::thread([this, uri] {
                    QImage img(QString::fromStdString(uri), nullptr);
                    imageLoadedGuarded(uri, img);
                }).detach();*/
                QImage img(QString::fromStdString(uri), nullptr);
                imageLoaded(uri, img);
            }
        }
    }

    if (uris.size() != cached_textures_.size())
    {
        is_dirty = true;
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

    return it->second;
}

void TextureCache::purge(const std::vector<std::string>& except_uris)
{
    std::lock_guard<std::mutex> lock_guard(mutex);

    for (auto it = cached_textures_.begin(); it != cached_textures_.end();)
    {
        if (std::find(except_uris.begin(), except_uris.end(), it->first) == except_uris.end())
        {
            if (!it->second.isNull())
            {
                Ogre::TextureManager::getSingleton().remove(it->second->getName());
            }
            it = cached_textures_.erase(it);
        }
        else
        {
            ++it;
        }
    }
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

    if (cached_textures_.find(uri) != cached_textures_.end())
    {
        if (!image.isNull())
        {
            it->second = textureFromImage(convertImage(image), uri);
        }
        else
        {
            cached_textures_.erase(it);
        }
    }
}

void TextureCache::imageLoadedGuarded(const std::string& uri, const QImage& image)
{
    std::lock_guard<std::mutex> lock_guard(mutex);
    imageLoaded(uri, image);
}
