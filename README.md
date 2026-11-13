# 点云处理工作台

Qt 5 Widgets / PCL 1.11 / VTK / OpenCV / C++17 的插件式桌面框架。主界面、自动参数界面、可视化和算法分别编译为 DLL；后续算法使用安装后的 SDK 独立开发。

当前接入算法为 **ExtractFeature2：三面结构焊缝提取**：[加载示例](examples/extract-feature2/README.md)、[源码与单独编译](algorithms/extract_feature2/README.md)。普通点云无需旧版的 `manifest.json`。

## 本机构建

### VS Code 编译与生成

用 VS Code 打开本工程根目录 `pointcloudExtractFeature`，安装推荐扩展 **CMake Tools** 和 **C/C++**（本机已安装）。默认构建主工作台，使用已有的 VS 2022 Build Tools、v142 工具集和 x64 Release 依赖。

- **Ctrl+Shift+B**：运行“工作台: Release 编译”，先配置 CMake 并生成工程，再编译。
- **F5**：在“运行和调试”中选择“工作台: 运行 Release”，自动编译、部署运行库，然后启动。
- **终端 → 运行任务**：可单独执行“工作台: CMake 配置并生成”“工作台: 编译并部署运行库”“工作台: CTest 测试”或“工作台: 安装 SDK 和运行库”。
- 使用 CMake Tools 面板时，配置预设选 `local`，构建预设选 `local-release`。

生成的 Visual Studio 工程为 `build/PointCloudWorkbench.sln`，编译产物在 `build/bin/Release`。首次直接运行 `Launcher.exe` 前需执行部署任务；F5 已包含此步骤。当前入口为 Release，源码断点调试需要另行配置带符号且与依赖兼容的构建。

原有 `carExtractFeature:` 任务和启动配置仍可使用，它们独立构建算法子工程并使用 `install/bin` 中的工作台。主工作台任务包含内置示例算法，其他独立算法按各自 README 编译部署。覆盖正在使用的 EXE/DLL 前请关闭对应工作台进程。

本机 CMake 路径在 `.vscode/settings.json`，Qt/PCL/VTK/OpenCV 路径在根目录 `CMakeUserPresets.json`；换机器时修改这两处。

### PowerShell 构建

已提供本机 `CMakeUserPresets.json`，该文件被 `.gitignore` 排除。依赖路径不写入公共构建配置。

```powershell
./scripts/Invoke-Local.ps1 Configure
./scripts/Invoke-Local.ps1 Build
./scripts/Invoke-Local.ps1 Test
./scripts/Invoke-Local.ps1 Smoke
./scripts/Invoke-Local.ps1 Deploy
```

`Deploy` 把运行依赖和 Qt 平台插件复制到 `build/bin/Release`。完成后直接运行该目录的 `Launcher.exe`。`Smoke` 自动生成点云、执行三个调试点并输出 `smoke-report.json`、`smoke-main.png` 和 `smoke-debug.png`，失败时返回非零退出码。

本机验证组合：MSVC v142 19.29.30159、Qt 5.14.1、PCL 1.11.0、VTK 8.2.0、OpenCV 4.2.0、Windows x64 Release。VTK 必须与 PCL 配套且包含 Qt 支持。实际配置生成 `build/dependencies.lock.txt`。

其他机器可自行提供依赖路径：

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -T v142 `
  -DQt5_DIR="<Qt>/lib/cmake/Qt5" `
  -DPCL_DIR="<PCL>/cmake" `
  -DVTK_DIR="<VTK>/lib/cmake/vtk-8.2" `
  -DOpenCV_DIR="<OpenCV>/x64/vc15/lib"
cmake --build build --config Release --parallel 4
cmake --build build --config Release --target deploy
```

依赖目录结构与 PCL Windows 安装包不同的环境，使用 `-DFW_RUNTIME_DIRS="路径1;路径2"` 补充运行库搜索目录。依赖不能解析时部署会失败，不会静默输出缺少 DLL 的包。

## 使用

1. 打开 PCD/PLY/TXT 点云，选择其单位。TXT 每个非空数据行使用前三列作为 X/Y/Z，支持空格、Tab 或逗号分隔，也可包含 `x y z` 表头和额外列。选择单位只记录元数据，不缩放坐标。
2. 在数据列表选择输入，在工具栏选择算法，右侧自动显示参数。
3. 点击运行。参数值在运行开始时冻结，之后修改只影响下一次运行。
4. 勾选“高级模式”和“单步调试”后运行，每个检查点自动弹出独立窗口并暂停。
5. 点击“下一步”“继续”或“停止”；关闭调试弹窗不会继续算法，主工具栏仍可操作。
6. 双击历史步骤重新打开尚未释放的快照。双击图层设置颜色、透明度、点大小和线宽。
7. 输出点云进入数据列表，可保存，或手动选为下一次算法输入。

可直接打开 `examples/data/demo.pcd` 试用（单位选择 m，包含平面、离群点和 intensity 字段）。`scripts/Generate-Sample.ps1` 可重新生成该文件。

默认保留 8 个步骤窗口和 512 MiB 的快照数据；“调试设置”可修改。达到任一上限会回收最早的窗口和快照，保留步骤名称和统计。内存预算只覆盖快照数据缓冲区，不包括算法本身、渲染转换数据和 GPU 内存。

## 工程结构

```text
app/                      启动程序
sdk/include/fw/            算法、数据、参数和界面公共接口
sdk/src/                   参数校验、数据适配和不可变快照
core/                      插件管理、运行调度、检查点同步
plugins/main/              主窗口动态库
plugins/parameters/        JSON 参数编辑器动态库
plugins/visualization/     Qt/VTK 视图和弹窗动态库
algorithms/sample/         三步示例算法和嵌入式参数定义
examples/algorithm-template/ 独立算法工程模板
tests/                     参数、线程、快照和插件集成测试
cmake/                     插件构建函数、SDK 包、运行库部署
```

## 安装 SDK 与独立算法

```powershell
./scripts/Invoke-Local.ps1 Install
cmake -S examples/algorithm-template -B build-external `
  -G "Visual Studio 17 2022" -A x64 -T v142 `
  -DCMAKE_PREFIX_PATH="<本工程>/install;<Qt>;<PCL>;<OpenCV>"
cmake --build build-external --config Release
```

外部模板生成 `build-external/bin/Release/plugins/algorithm/MyAlgorithm.dll`。退出工作台后复制到工作台的 `plugins/algorithm` 目录，再启动即可出现“我的算法”。修改算法 ID 时同时修改 CMake 注册项、descriptor、JSON 参数定义及资源路径。不要保留同 ID 的旧 DLL 副本。

`Invoke-Local.ps1 Install` 会安装 SDK 并为 `install/bin` 收集运行依赖，可以直接运行 `install/bin/Launcher.exe`。单独执行原生 `cmake --install` 只安装框架目标，不收集第三方 DLL。分发算法开发 SDK 时使用 `install`，开发机另外配置相同版本依赖。

参数接入先看 [算法参数接入指南](docs/算法参数接入指南.md)，其中提供全部类型的可复制 JSON 示例。详细约定见 [参数格式](docs/参数格式.md)、[算法开发](docs/算法开发.md)、[架构与线程](docs/架构与线程.md)、[界面设计与验证](docs/界面设计.md)。

已迁入原 `extractFeature` 工程的默认帧焊缝算法，源码和独立构建入口见 [独立焊缝算法工程](algorithms/frame_seam/README.md)。它使用已安装 SDK 单独编译，通过 `build.ps1 Deploy` 部署到工作台，不参与主界面工程构建。[本机示例运行入口](examples/frame-seam/README.md)。

## 已知边界

- 一次运行一个算法，未实现节点编排、进程隔离或运行时卸载 DLL。
- 停止为协作式取消：框架能立即唤醒检查点，但需要等待正在执行的 PCL 调用返回。
- 算法和宿主必须采用同一 SDK、工具链、依赖、CRT、构建模式及兼容编译选项；不能将 Debug DLL 放入 Release 主程序。
- 示例算法要求 XYZ 为 float32，原始其他字段通过索引提取保留；体素降采样使用 PCL 的字段聚合语义，不保证标签类字段含义不变。通用数据容器保留原始字段。
- 点云文件格式本身不可靠保存单位，重新打开文件仍需选择单位。显示用采样不影响算法输出。
- MSVC 工具链、第三方库和 Windows/GPU 环境不同的机器仍需独立验收；本机部署验证不能替代全新虚拟机验收。
