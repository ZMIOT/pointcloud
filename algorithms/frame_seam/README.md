# 独立焊缝算法工程

这里是**写算法源码的地方**。DLL 是编译产物，不是源码的替代品。本目录可独立构建，使用已安装 FrameworkSDK 和同版本开发依赖，不需要重编工作台或参数界面。

## 平时修改哪里

| 内容 | 位置 |
| --- | --- |
| 帧轮廓、候选、面修正、跟踪和平滑计算 | `core/FrameSeamAlgorithm.cpp` |
| 参数定义、默认值、约束、分组 | `parameters.json` |
| 参数映射、输入校验、输出和调试适配 | `Plugin.cpp` |
| 单步回调和协作取消桥接 | `core/AlgorithmPipeline.cpp/.h` |
| 独立构建规则 | `CMakeLists.txt` |
| 算法及插件回归测试 | `tests/` |

如果只调整计算逻辑，主要改核心计算文件。添加一个新参数时，同时修改配置 JSON、`Plugin.cpp` 的 `configuration()` 映射及核心使用处。当前插件显示的参数都已映射到原算法配置，没有让界面出现不起作用的占位选项。

## 编译、测试、部署、打包

从工作台工程根目录执行：

```powershell
# 只配置和编译这个算法工程，不编译主程序
./algorithms/frame_seam/build.ps1 Build

# 原算法回归 + 插件参数、调试一致性、取消和异常输入测试
./algorithms/frame_seam/build.ps1 Test

# 退出工作台后，仅复制算法 DLL；重启后生效
./algorithms/frame_seam/build.ps1 Deploy

# 可选：生成用于交付的 ZIP，只含算法 DLL 和部署说明
./algorithms/frame_seam/build.ps1 Package
```

开发时每次试运行都需要编译，**不需要每次制作 ZIP**。只有交付给另一套兼容工作台时才需要打包。

默认 DLL 输出为 `build/bin/Release/plugins/algorithm/FrameSeamAlgorithm.dll`；默认部署到宿主 `install/bin/plugins/algorithm`。ZIP 输出为 `build/FrameSeamAlgorithm-1.0.0-win64.zip`。参数通过 `parameters.qrc` 嵌入 DLL。

脚本默认从宿主已有 CMake 缓存读取依赖路径，只读取配置，不构建宿主。可以通过 `-SdkRoot`、`-HostDirectory`、`-DependencyCache` 指定自己的 SDK、工作台和依赖缓存。

复制本目录到其他位置独立开发时，也可直接使用 CMake：

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -T v142 `
  -DFrameworkSDK_DIR="<SDK>/lib/cmake/FrameworkSDK" `
  -DQt5_DIR="<Qt>/lib/cmake/Qt5" `
  -DPCL_DIR="<PCL>/cmake" `
  -DOpenCV_DIR="<OpenCV>/lib"
cmake --build build --config Release --parallel 4
```

运行测试时，将已部署宿主的 `bin` 加入当前进程 PATH，让测试找到相同运行库。可选的 GUI 集成测试工具默认关闭，它需要宿主头文件与已编译 DLL，仅供框架开发时验证，普通算法构建不需要它。

## 在工作台运行

1. 导入完整原始扫描点云，选择正确单位。当前原数据是毫米。
2. 算法列表选择“焊缝识别：单帧轮廓 + 邻帧局部面”。
3. 选择与点云逐点对应的 `manifest.json`，或加载示例预设。索引引用的每帧原始文件必须仍在同目录。
4. 普通运行生成左右两份结果。高级模式勾选单步调试后，默认弹出六个主要步骤窗口。
5. 如需逐帧排查，在高级参数中选择“主要步骤与逐帧快照”。框架负责窗口上限和快照回收，不要求算法保留全部帧副本。

迁移用的本机示例入口见宿主 `examples/frame-seam/README.md`。原项目 TXT 已无重排地转换为 PCD，方便工作台现有导入功能使用。转换后再次逐点校验原始帧索引；没有修改宿主导入模块。

## 输出约定

- 两份点云：左焊缝、右焊缝；保留输入坐标系及单位。
- XYZ 是算法计算的新轨迹坐标，附带每点 `confidence`（float32）与 `segment_id`（uint32）。不把原扫描的其他属性伪装成新轨迹属性。
- 段号从 0 开始；显示按段连接，不跨间断段连接。可以保存 PCD 后交给后续算法。
- 置信度沿用原算法评分，不代表经过标定的正确率。
- 与原实现一致，两侧分别允许缺帧、点数不同；整体成功仍要求两侧各至少三个可靠点。

## 迁移范围与源码来源

来源是用户指定的 `E:/CODE/extractFeature`，迁移其默认“方案3”。原目录未修改；源文件 SHA-256 清单在 `source-files.json`。

原数值计算保持不变；适配改动包括：

- 拆除旧 GUI/CLI 的多方案调度入口，不包含方案1及未实现的方案2。
- 在处理阶段及逐帧循环加入宿主中立的执行回调和取消检查；取消不是强杀线程。
- 原步骤可直接流式提交工作台；宿主模式仅保留原 trace 最近一步，完整不可变快照及其预算由工作台管理。原算法回归模式仍可保留完整 trace。
- 帧索引校验加入取消回调和非有限坐标拒绝；保留逐点、点序、点数校验。
- 将两份结果及分段信息转换成 FrameworkSDK 数据，不依赖 QWidget 或 VTK actor。

方案3需要可靠的采集帧索引，不按点距猜测帧边界，不自动启用参考算法或历史约束。后续如需方案1，应单独接入并定义历史输入契约。

## 本次验证

- 原算法合成回归通过：正反槽向、平滑曲面拒绝、局部遮挡、焊趾凸起、缺帧和分段。
- 插件测试通过：默认参数校验、普通与主要步骤调试一致、逐帧调试一致、检查点停止、预先取消、错误索引和坐标不匹配拒绝。
- 完整原始扫描共 704,808 点、695 帧。原 CLI 与插件均输出左侧 565 点、右侧 384 点；逐点 XYZ 最大差异为 0。左侧 4 段、右侧 2 段。
- 调试开启和关闭的输出一致；实际加载已部署主界面/可视化 DLL，完成 6 个主要检查点并显示两个结果，主窗口和调试窗口均未越界。
- 验证记录位于宿主 `build/frame-seam-real/`、`build/frame-seam-ui/`，原算法基线位于 `build/frame-seam-baseline/`。

这些检查验证迁移一致性和接入功能，不替代新扫描数据上的精度验收。
