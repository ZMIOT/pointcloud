# PointCloudWeldSdk V1

本包提供波纹板特征提取的 Windows x64 Release C ABI。SDK 接收带帧号的 XYZ 点云，内部执行当前波纹板点云预处理、板面支持过滤、C 特征候选恢复和白色均匀点拟合，不读写业务点云文件。

## 文件

- `bin/PointCloudWeldSdk.dll`：运行库。
- `bin/vcomp140.dll`：SDK 使用的 MSVC OpenMP 运行库（构建环境可定位时随包安装）。
- `bin/msvcp140.dll`、`vcruntime140.dll`、`vcruntime140_1.dll`：SDK 使用的
  MSVC x64 运行库（构建环境可定位时随包安装）。
- `lib/PointCloudWeldSdk.lib`：MSVC 导入库。
- `include/PointCloudWeldSdk.h`：C/C++ 公共头文件和 ABI 尺寸断言。
- `config/parameters.json`：DLL 可直接读取的波纹板运行参数。
- `README.md`：本说明。

## 输入与输出

输入数组元素为 `PcwsFramePointV1`：

```c
typedef struct PcwsFramePointV1 {
    uint64_t frame_number;
    PcwsPoint3d position_mm;
} PcwsFramePointV1;
```

同一帧允许多个点，帧号不要求连续。坐标单位为毫米。`scan_direction` 可为零；为零时按输出点关联的帧号递增确定首尾。`profile_direction` 可为零；为零时由点云自动确定轮廓横向。

成功时 `PcwsResultV1` 同时返回：

- `sdkbase`：当前算法的 C 特征候选点，对应历史 `PreciseLaserPoint_SdkBase.txt` 的 XYZ 序列。
- `sdkclass`：当前算法的白色均匀点，对应历史 `PreciseLaserPoint_Classified.txt` 的几何序列；同时给出最近 `sdkbase` 索引、点类型和后续段类型。

## 最小调用示例

```cpp
#include <PointCloudWeldSdk.h>

#include <stdexcept>
#include <vector>

PcwsStatus process(const std::vector<PcwsFramePointV1>& cloud,
                   const char* parametersJsonPath) {
    PcwsOptionsV1 options{};
    char configError[256]{};
    if (PcwsLoadOptionsV1(parametersJsonPath, &options, configError) !=
        PCWS_OK) {
        throw std::runtime_error(configError);
    }

    PcwsRequestV1 request{};
    request.cloud_mm = cloud.data();
    request.cloud_count = static_cast<uint64_t>(cloud.size());
    request.options = &options;

    PcwsResultV1 result{};
    const PcwsStatus status = PcwsProcessV1(&request, &result);
    if (status == PCWS_OK) {
        // result.sdkbase[0..sdkbase_count) 和
        // result.sdkclass[0..sdkclass_count) 仅在释放前有效。
    }
    PcwsReleaseResultV1(&result);
    return status;
}
```

`PcwsLoadOptionsV1` 接收显式 UTF-8 文件路径，并把 JSON 转成可直接传给
`PcwsProcessV1` 的完整参数快照。DLL 不依赖当前工作目录，也不会暗中搜索文件；
调用方应根据自身程序目录拼出 `config/parameters.json` 的绝对路径。这样参数文件
缺失、字段拼错或取值非法时会立即返回错误，而不是悄悄换成另一套参数。
如果确实要使用 DLL 内置缺省值，可以令 `request.options = nullptr`。

输入和回调上下文由调用方持有到 `PcwsProcessV1` 返回。结果数组由 DLL 分配，必须由同一个 DLL 的 `PcwsReleaseResultV1` 释放。每次独立调用使用独立且已清零的 `PcwsResultV1`；失败与取消不返回部分数组，原因写入 `error_utf8`。

## 构建与验收

仓库内的 CMake 目标为 `PointCloudWeldSdk`，自动测试为 `corrugated_panel_sdk`。测试覆盖 JSON 参数读取、ABI 尺寸、非法参数、取消、帧号自动定向、显式反向、C 候选/白点映射、索引单调性及重复释放。
