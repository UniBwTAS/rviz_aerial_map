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

#include <OGRE/OgreTextureManager.h>
#include <QImage>
#include <functional>
#include <ros/console.h>

#include "texture_cache.h"

TextureCache::TextureCache(bool uri_is_url)
  : uri_is_url_(uri_is_url)
  , downloader_([this](const std::string& uri, const QImage& image) { imageLoadedGuarded(uri, image); })
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
      Status status = std::get<0>(it->second);
      double elapsed_time = static_cast<double>(clock() - std::get<1>(it->second)) / CLOCKS_PER_SEC;
      if ((status == Status::Error || status == Status::Loading) && elapsed_time > 3.)
      {
        ROS_DEBUG_STREAM("LOADING [TIMEOUT]: " << uri << "(" << elapsed_time << ")");
        cached_textures_.erase(it);
        load_image = true;
      }
    }
    if (load_image)
    {
      is_dirty = true;
      cached_textures_.insert({ uri, { Status::Loading, clock(), Ogre::TexturePtr() } });
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
  return texture_manager.loadRawData(name, res_group, data_stream, image.width(), image.height(), Ogre::PF_B8G8R8,
                                     Ogre::TEX_TYPE_2D, 0);
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
      std::get<0>(it->second) = Status::Finished;
    }
    else
    {
      ROS_DEBUG_STREAM("ERROR: " << uri << std::endl);
      std::get<0>(it->second) = Status::Error;
    }
  }
}

void TextureCache::imageLoadedGuarded(const std::string& uri, const QImage& image)
{
  std::lock_guard<std::mutex> lock_guard(mutex);
  imageLoaded(uri, image);
}
