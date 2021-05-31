#pragma once

#include <map>
#include <mutex>

#include "image_downloader.h"

class TextureCache
{
public:
  enum class Status
  {
    Loading,
    Finished,
    Error
  };

public:
  explicit TextureCache(bool uri_is_url);
  ~TextureCache();

  bool request(const std::vector<std::string>& uris);
  Ogre::TexturePtr ready(const std::string& uri) const;

private:
  void imageLoaded(const std::string& uri, const QImage& image);
  void imageLoadedGuarded(const std::string& uri, const QImage& image);

  static Ogre::TexturePtr textureFromImage(const QImage& image, const std::string& name);
  /**
   * Convert any QImage to a 24bit RGB QImage
   */
  static QImage convertImage(const QImage& image);

private:
  mutable std::mutex mutex;
  bool uri_is_url_;
  std::map<std::string, std::tuple<Status, clock_t, Ogre::TexturePtr>> cached_textures_;
  ImageDownloader downloader_;
};
