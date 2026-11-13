#include "LapEdge.h"
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    if (argc < 3) {
        std::cerr
            << "Usage: CarLapEdgeInspect input.pcd output-dir [neighborhood smoothing evidence]\n";
        return 2;
    }
    try {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile(argv[1], cloud) != 0)
            return 2;
        std::vector<lap::Point> points;
        for (std::size_t i = 0; i < cloud.size(); ++i)
            points.push_back({lap::Vec(cloud[i].x, cloud[i].y, cloud[i].z), std::uint32_t(i)});
        lap::Config cfg;
        if (argc > 3)
            cfg.neighborhoodScale = std::stod(argv[3]);
        if (argc > 4)
            cfg.smoothing = std::stod(argv[4]);
        if (argc > 5)
            cfg.evidenceScale = std::stod(argv[5]);
        QElapsedTimer timer;
        timer.start();
        const auto r = lap::detect(points, cfg);
        const auto elapsed = timer.elapsed();
        QDir().mkpath(argv[2]);
        QDir dir(argv[2]);
        QJsonObject reasons;
        int valid = 0, inserted = 0;
        for (const auto &c : r.candidates) {
            const QString name = lap::reasonName(c.reason);
            reasons[name] = reasons[name].toInt() + 1;
            if (c.eligible)
                ++valid;
        }
        for (const auto &s : r.trajectory)
            if (s.interpolated)
                ++inserted;
        QJsonObject report{
            {"ok", r.ok},
            {"message", QString::fromStdString(r.message)},
            {"elapsed_ms", double(elapsed)},
            {"input_points", int(points.size())},
            {"sections", int(r.sections.size())},
            {"point_spacing", r.pointSpacing},
            {"section_spacing", r.sectionSpacing},
            {"scan_normal", QJsonArray{r.scanNormal.x(), r.scanNormal.y(), r.scanNormal.z()}},
            {"measured_points", int(r.measured.size())},
            {"trajectory_points", int(r.trajectory.size())},
            {"interpolated_points", inserted},
            {"surface_validated", valid},
            {"measured_segments", r.measured.empty() ? 0 : int(r.measured.back().segment) + 1},
            {"trajectory_segments",
             r.trajectory.empty() ? 0 : int(r.trajectory.back().segment) + 1},
            {"reasons", reasons}};
        QFile json(dir.filePath("report.json"));
        if (!json.open(QIODevice::WriteOnly))
            throw std::runtime_error("Cannot write report");
        json.write(QJsonDocument(report).toJson());
        for (int mode = 0; mode < 2; ++mode) {
            std::ofstream file(
                dir.filePath(mode ? "trajectory.csv" : "measured.csv").toStdString());
            file << std::setprecision(12)
                 << "section_id,segment_id,source_index,x,y,z,confidence,interpolated\n";
            for (const auto &s : mode ? r.trajectory : r.measured)
                file << s.section << ',' << s.segment << ',' << s.source << ',' << s.xyz.x() << ','
                     << s.xyz.y() << ',' << s.xyz.z() << ',' << s.confidence << ','
                     << s.interpolated << '\n';
        }
        std::ofstream candidates(dir.filePath("candidates.csv").toStdString());
        candidates << std::setprecision(10)
                   << "section_id,source_index,side,score,step,gap,reason,radius,upper_surface_"
                      "sections,lower_surface_sections,surface_noise,surface_residual,raw_coarse_"
                      "source,raw_run_count,raw_run_span,raw_decision\n";
        for (const auto &c : r.candidates)
            candidates << c.section << ',' << r.input[c.point].source << ',' << c.side << ','
                       << c.score << ',' << c.step << ',' << c.gap << ','
                       << lap::reasonName(c.reason) << ',' << c.radius << ','
                       << c.upperSurfaceSections << ',' << c.lowerSurfaceSections << ','
                       << c.surfaceNoise << ',' << c.surfaceResidual << ','
                       << (c.rawCoarsePoint < 0 ? -1 : int(r.input[c.rawCoarsePoint].source)) << ','
                       << c.rawRunCount << ',' << c.rawRunSpan << ','
                       << static_cast<int>(c.rawDecision) << '\n';
        std::cout << QJsonDocument(report).toJson(QJsonDocument::Compact).constData() << '\n';
        return r.ok ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}
