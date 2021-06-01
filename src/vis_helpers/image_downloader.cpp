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