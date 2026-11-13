#include "PointCloudIO.h"

#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace weld {
namespace {

std::string lowerExtension(const std::string& path) {
    const std::size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) {
        return std::string();
    }
    std::string result = path.substr(dot);
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return result;
}

void finalizeCloud(CloudT& cloud) {
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = false;
}

} // namespace

CloudT::Ptr PointCloudIO::load(const std::string& path, std::string* error) {
    const std::string extension = lowerExtension(path);
    CloudT::Ptr cloud(new CloudT);

    if (extension == ".txt" || extension == ".xyz" || extension == ".csv") {
        return loadText(path, error);
    }

    int status = -1;
    if (extension == ".pcd") {
        status = pcl::io::loadPCDFile<PointT>(path, *cloud);
    } else if (extension == ".ply") {
        status = pcl::io::loadPLYFile<PointT>(path, *cloud);
    } else {
        if (error) {
            *error = "Unsupported point-cloud format: " + extension;
        }
        return CloudT::Ptr(new CloudT);
    }

    if (status < 0 || cloud->empty()) {
        if (error) {
            *error = "Unable to load point cloud: " + path;
        }
        cloud->clear();
    }
    return cloud;
}

bool PointCloudIO::savePointCloud(const std::string& path,
                                  const CloudT& cloud,
                                  std::string* error) {
    if (cloud.empty()) {
        if (error) {
            *error = "Cannot save an empty point cloud: " + path;
        }
        return false;
    }

    const std::string extension = lowerExtension(path);
    int status = -1;
    if (extension == ".pcd") {
        status = pcl::io::savePCDFileBinary(path, cloud);
    } else if (extension == ".ply") {
        status = pcl::io::savePLYFileBinary(path, cloud);
    } else if (extension == ".xyz" || extension == ".txt") {
        std::ofstream output(path.c_str());
        if (output) {
            output << std::fixed << std::setprecision(6);
            for (const PointT& point : cloud) {
                output << point.x << ' ' << point.y << ' ' << point.z << '\n';
            }
            status = output.good() ? 0 : -1;
        }
    } else {
        if (error) {
            *error = "Unsupported output point-cloud format: " + extension;
        }
        return false;
    }

    if (status < 0) {
        if (error) {
            *error = "Unable to save point cloud: " + path;
        }
        return false;
    }
    return true;
}

CloudT::Ptr PointCloudIO::loadText(const std::string& path, std::string* error) {
    std::ifstream input(path.c_str());
    CloudT::Ptr cloud(new CloudT);
    if (!input) {
        if (error) {
            *error = "Unable to open text point cloud: " + path;
        }
        return cloud;
    }

    std::string line;
    std::size_t invalidLines = 0;
    int xyzColumns[3] = {0, 1, 2};
    bool layoutResolved = false;
    while (std::getline(input, line)) {
        if (line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
        std::replace(line.begin(), line.end(), ',', ' ');
        if (!layoutResolved) {
            std::istringstream header(line);
            std::vector<std::string> columns;
            std::string name;
            while (header >> name) {
                std::transform(name.begin(), name.end(), name.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                columns.push_back(name);
            }
            const auto x = std::find(columns.begin(), columns.end(), "x");
            const auto y = std::find(columns.begin(), columns.end(), "y");
            const auto z = std::find(columns.begin(), columns.end(), "z");
            if (x != columns.end() && y != columns.end() && z != columns.end()) {
                xyzColumns[0] = static_cast<int>(x - columns.begin());
                xyzColumns[1] = static_cast<int>(y - columns.begin());
                xyzColumns[2] = static_cast<int>(z - columns.begin());
                layoutResolved = true;
                continue;
            }
        }
        std::istringstream parser(line);
        PointT point;
        const int lastColumn = std::max(xyzColumns[0], std::max(xyzColumns[1], xyzColumns[2]));
        bool parsed = true;
        for (int column = 0; column <= lastColumn; ++column) {
            double value = 0;
            if (!(parser >> value)) { parsed = false; break; }
            if (column == xyzColumns[0]) point.x = static_cast<float>(value);
            if (column == xyzColumns[1]) point.y = static_cast<float>(value);
            if (column == xyzColumns[2]) point.z = static_cast<float>(value);
        }
        if (!parsed || !pcl::isFinite(point)) {
            ++invalidLines;
            continue;
        }
        layoutResolved = true;
        cloud->push_back(point);
    }
    finalizeCloud(*cloud);

    if (cloud->empty() && error) {
        std::ostringstream message;
        message << "No valid XYZ points were found in " << path
                << " (invalid lines: " << invalidLines << ")";
        *error = message.str();
    }
    return cloud;
}

bool PointCloudIO::saveSeamCsv(const std::string& path,
                               const CloudT& seam,
                               const std::vector<float>& confidence,
                               Axis axis,
                               std::string* error) {
    std::ofstream output(path.c_str());
    if (!output) {
        if (error) {
            *error = "Unable to create output file: " + path;
        }
        return false;
    }

    output << "station,x,y,z,confidence\n";
    output << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < seam.size(); ++i) {
        const float score = i < confidence.size() ? confidence[i] : 0.0f;
        output << coordinate(seam[i], axis) << ','
               << seam[i].x << ',' << seam[i].y << ',' << seam[i].z << ','
               << score << '\n';
    }
    return true;
}

bool PointCloudIO::saveDualSeamCsv(const std::string& path,
                                   const CloudT& leftSeam,
                                   const std::vector<float>& leftConfidence,
                                   const CloudT& rightSeam,
                                   const std::vector<float>& rightConfidence,
                                   Axis axis,
                                   std::string* error) {
    std::ofstream output(path.c_str());
    if (!output) {
        if (error) {
            *error = "Unable to create output file: " + path;
        }
        return false;
    }

    output << "side,station,x,y,z,confidence\n";
    output << std::fixed << std::setprecision(6);
    const auto writeSide = [&](char side,
                               const CloudT& seam,
                               const std::vector<float>& confidence) {
        for (std::size_t i = 0; i < seam.size(); ++i) {
            const float score = i < confidence.size() ? confidence[i] : 0.0f;
            output << side << ',' << coordinate(seam[i], axis) << ','
                   << seam[i].x << ',' << seam[i].y << ',' << seam[i].z << ','
                   << score << '\n';
        }
    };
    writeSide('L', leftSeam, leftConfidence);
    writeSide('R', rightSeam, rightConfidence);
    return true;
}

bool PointCloudIO::loadDualSeamCsv(const std::string& path,
                                   CloudT::Ptr* leftSeam,
                                   CloudT::Ptr* rightSeam,
                                   std::string* error) {
    if (!leftSeam || !rightSeam) {
        if (error) {
            *error = "Dual seam output pointers are null.";
        }
        return false;
    }
    std::ifstream input(path.c_str());
    if (!input) {
        if (error) {
            *error = "Unable to open dual seam file: " + path;
        }
        return false;
    }

    CloudT::Ptr left(new CloudT);
    CloudT::Ptr right(new CloudT);
    std::string line;
    while (std::getline(input, line)) {
        const std::size_t comma = line.find(',');
        if (comma == std::string::npos) {
            continue;
        }
        std::string side = line.substr(0, comma);
        std::transform(side.begin(), side.end(), side.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
        if (side != "L" && side != "LEFT" && side != "R" && side != "RIGHT") {
            continue;
        }
        std::string valuesText = line.substr(comma + 1);
        std::replace(valuesText.begin(), valuesText.end(), ',', ' ');
        std::istringstream parser(valuesText);
        double station = 0.0;
        PointT point;
        if (!(parser >> station >> point.x >> point.y >> point.z) || !pcl::isFinite(point)) {
            continue;
        }
        (void)station;
        if (side == "L" || side == "LEFT") {
            left->push_back(point);
        } else {
            right->push_back(point);
        }
    }
    finalizeCloud(*left);
    finalizeCloud(*right);
    if (left->size() < 2 || right->size() < 2) {
        if (error) {
            *error = "The file does not contain both L and R seam tracks: " + path;
        }
        return false;
    }
    *leftSeam = left;
    *rightSeam = right;
    return true;
}

CloudT::Ptr PointCloudIO::loadSeamCsv(const std::string& path, std::string* error) {
    // 历史文件也只依赖 XYZ。saveSeamCsv 的首列是 station，所以这里兼容
    // “station,x,y,z,...”和普通“x,y,z”两种格式。
    std::ifstream input(path.c_str());
    CloudT::Ptr cloud(new CloudT);
    if (!input) {
        if (error) {
            *error = "Unable to open seam file: " + path;
        }
        return cloud;
    }

    std::string line;
    bool generatedFormat = false;
    bool formatKnown = false;
    while (std::getline(input, line)) {
        if (!formatKnown) {
            std::string lower = line;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (lower.find("station") != std::string::npos) {
                generatedFormat = true;
                formatKnown = true;
                continue;
            }
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream parser(line);
        std::vector<double> values;
        double value = 0.0;
        while (parser >> value) {
            values.push_back(value);
        }
        if (values.empty()) {
            // 表头无法解析成数字，会自然走到这里。
            continue;
        }

        PointT point;
        if (generatedFormat && values.size() >= 4) {
            point.x = static_cast<float>(values[1]);
            point.y = static_cast<float>(values[2]);
            point.z = static_cast<float>(values[3]);
        } else if (values.size() >= 3) {
            point.x = static_cast<float>(values[0]);
            point.y = static_cast<float>(values[1]);
            point.z = static_cast<float>(values[2]);
        } else {
            continue;
        }
        formatKnown = true;
        if (pcl::isFinite(point)) {
            cloud->push_back(point);
        }
    }
    finalizeCloud(*cloud);
    if (cloud->empty() && error) {
        *error = "No valid XYZ seam points were found in: " + path;
    }
    return cloud;
}

} // namespace weld
