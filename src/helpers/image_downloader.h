#include <functional>

#include <QImageReader>
#include <QtNetwork>

class ImageDownloader : public QObject
{
    Q_OBJECT

  public:
    explicit ImageDownloader(std::function<void(std::string, QImage)> callback);

    void loadFile(const std::string& url);

  public Q_SLOTS:
    void downloadFinished(QNetworkReply* reply);

  private:
    QNetworkAccessManager* manager;
    std::function<void(std::string, QImage)> callback;
};
