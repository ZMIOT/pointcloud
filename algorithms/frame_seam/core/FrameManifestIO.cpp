#include "FrameManifestIO.h"
#include "PointCloudIO.h"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>

namespace weld {
QString findFrameManifest(const QString &inputPath) {
  QDir dir(QFileInfo(inputPath).absolutePath());
  for (const auto &relative :
       {QString("manifest.json"),
        QString("frame_data/20260907_001/manifest.json")}) {
    const auto candidate = dir.filePath(relative);
    if (QFileInfo::exists(candidate))
      return candidate;
  }
  return {};
}
bool loadFrameManifest(const QString &path, const CloudT::ConstPtr &input,
                       std::vector<ScanFrameRange> *frames, QString *error,
                       std::function<bool()> cancelled) {
  frames->clear();
  auto fail = [&](const QString &reason) {
    if (error)
      *error = reason;
    frames->clear();
    return false;
  };
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly))
    return fail(QStringLiteral("方案3需要匹配的 manifest.json：") + path);
  QJsonParseError parse;
  const auto document = QJsonDocument::fromJson(file.readAll(), &parse);
  if (parse.error != QJsonParseError::NoError || !document.isObject())
    return fail(QStringLiteral("帧索引 JSON 无效。"));
  const auto object = document.object();
  if (!input || object["inputPointCount"].toDouble() != input->size())
    return fail(
        QStringLiteral("帧索引总点数与输入不一致；请加载完整原始点云。"));
  const auto array = object["frames"].toArray();
  if (array.isEmpty())
    return fail(QStringLiteral("帧索引为空。"));
  std::size_t offset = 0;
  const QDir parent(QFileInfo(path).absolutePath());
  for (const auto &entry : array) {
    if (cancelled && cancelled()) return fail(QStringLiteral("已取消帧校验"));
    const auto f = entry.toObject();
    const int count = f["pointCount"].toInt(-1);
    if (count < 3 || f["startIndex"].toDouble() != offset + 1 ||
        f["endIndex"].toDouble() != offset + count ||
        offset + count > input->size())
      return fail(QStringLiteral("帧区间不连续、越界或点数不足。"));
    const auto name = f["file"].toString();
    if (name.isEmpty() || QFileInfo(name).fileName() != name)
      return fail(QStringLiteral("帧文件名无效。"));
    std::string loadError;
    const auto cloud = PointCloudIO::load(
        parent.filePath(name).toLocal8Bit().constData(), &loadError);
    if (!cloud || cloud->size() != static_cast<std::size_t>(count))
      return fail(QStringLiteral("无法核对原始帧：") + name);
    // 点数相同不足以证明是同一次扫描，逐点核对 XYZ，容许文本浮点舍入。
    for (int i = 0; i < count; ++i)
      if (!std::isfinite((*input)[offset+i].x) || !std::isfinite((*input)[offset+i].y) || !std::isfinite((*input)[offset+i].z) ||
          (toVector((*input)[offset + i]) - toVector((*cloud)[i]))
              .cwiseAbs()
              .maxCoeff() > 0.0001f)
        return fail(QStringLiteral("输入点序/坐标与原始帧不一致：") + name);
    frames->push_back(
        {offset, static_cast<std::size_t>(count), f["sdkSequence"].toInt()});
    offset += count;
  }
  if (offset != input->size())
    return fail(QStringLiteral("帧区间未覆盖全部输入点。"));
  return true;
}
} // namespace weld
