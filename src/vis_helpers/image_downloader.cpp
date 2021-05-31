#include "image_downloader.h"

ImageDownloader::ImageDownloader(std::function<void(std::string, QImage)> callback)
  : manager(new QNetworkAccessManager(this)), callback(std::move(callback))
{
  connect(manager, SIGNAL(finished(QNetworkReply*)), SLOT(downloadFinished(QNetworkReply*)));

  auto* disk_cache = new QNetworkDiskCache(this);
  QString const cache_path =
      QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation)).filePath("rviz_aerial_map");
  disk_cache->setCacheDirectory(cache_path);
  // there is no option to disable maximum cache size
  disk_cache->setMaximumCacheSize(std::numeric_limits<qint64>::max());
  manager->setCache(disk_cache);
}

void ImageDownloader::loadFile(const std::string& url)
{
  // see:
  // https://foundation.wikimedia.org/wiki/Maps_Terms_of_Use#Using_maps_in_third-party_services
  QNetworkRequest request(QUrl(QString::fromStdString(url)));
  char constexpr agent[] = "rviz_aerial_map/" RVIZ_AERIAL_MAP_VERSION " (+https://github.com/UniBwTAS/rviz_aerial_map)";
  request.setHeader(QNetworkRequest::KnownHeaders::UserAgentHeader, agent);
  manager->get(request);
}

void ImageDownloader::downloadFinished(QNetworkReply* reply)
{
  QImage image = QImage();
  if (!reply->error())
  {
    QImageReader reader(reply);
    if (reader.canRead())
    {
      image = reader.read();
    }
  }

  callback(reply->url().toString().toStdString(), image);

  reply->deleteLater();
}