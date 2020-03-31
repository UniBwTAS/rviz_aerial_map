#pragma once

#include <map>
#include <mutex>

#include "helpers/image_downloader.h"

class TextureCache
{
  public:
    static const int STATUS_LOADING = 0;
    static const int STATUS_FINISHED = 1;
    static const int STATUS_ERROR = 2;

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
    std::map<std::string, std::tuple<int, clock_t, Ogre::TexturePtr>> cached_textures_;
    ImageDownloader downloader_;
};
