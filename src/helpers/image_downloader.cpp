#include "image_downloader.h"

ImageDownloader::ImageDownloader(std::function<void(std::string, QImage)> callback)
    : manager(new QNetworkAccessManager(this)), callback(std::move(callback))
{
    connect(manager, SIGNAL(finished(QNetworkReply*)), SLOT(downloadFinished(QNetworkReply*)));

    auto* disk_cache = new QNetworkDiskCache(this);
    QString const cache_path =
        QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation)).filePath("rviz_satellite");
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
    char constexpr agent[] = "rviz_satellite/" RVIZ_SATELLITE_VERSION " (+https://github.com/gareth-cross/"
                             "rviz_satellite)";
    request.setHeader(QNetworkRequest::KnownHeaders::UserAgentHeader, agent);
    manager->get(request);
}

void ImageDownloader::downloadFinished(QNetworkReply* reply)
{
    QUrl const url = reply->url();
    if (reply->error())
    {
        return;
    }

    QImageReader reader(reply);
    if (!reader.canRead())
    {
        return;
    }

    callback(url.toString().toStdString(), reader.read());

    reply->deleteLater();
}