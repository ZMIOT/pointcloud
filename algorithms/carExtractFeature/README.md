# 汽车波纹搭接：上层边缘提取

插件 ID：`custom.carExtractFeature`。输入一份保留扫描点序的 XYZ 点云；默认 +Z 为物理上方。长度始终采用输入坐标单位，不自动换算。

`2.pcd` 高密度扫描的故障原因、修正结果和验收图见 [VALIDATION-2.md](VALIDATION-2.md)。

## 运行

在 VS Code 中打开工程根目录 `pointcloudExtractFeature`，安装 Microsoft C/C++ 与 CMake Tools 扩展。CMake Tools 的源码目录已指向本算法；首次使用时选择配置预设 `local`，构建预设 `local-release`。这两个预设读取本机的 `CMakeUserPresets.json`，它被 Git 忽略；公共工具链配置在 `CMakePresets.json`。

- `carExtractFeature: 运行工作台`：先构建并部署插件，再从 `install/bin` 启动工作台。导入点云后在界面中运行算法；可以在 `LapEdge.cpp` 和 `MyAlgorithm.cpp` 下断点。
- `carExtractFeature: 检查 3.pcd`：先构建，再直接运行独立检查器；结果写到 `algorithms/carExtractFeature/build/vscode-inspect`。检查其他文件时修改 `.vscode/launch.json` 中的第一个参数。

`Ctrl+Shift+B` 执行 `cmake --build --preset local-release`；“终端 → 运行任务”可执行 CTest 或 CMake 编译并部署。F5 的预启动任务直接调用 CMake 配置与 `CarExtractFeatureDeploy` 目标，不经过 `build.ps1`。两种启动配置都使用现有 Release x64 依赖与 ABI，构建产物会保留 PDB 调试符号。启动工作台前请先关闭正在运行的旧工作台，以便覆盖插件 DLL。

命令行使用同一套 CMake 预设：

```powershell
cd algorithms/carExtractFeature
cmake --preset local
cmake --build --preset local-release --parallel 4
ctest --preset local-release
cmake --build --preset local-release --target CarExtractFeatureDeploy
```

若普通 PowerShell 终端找不到 `cmake`/`ctest`，可直接使用 VS Code 任务；工作区配置已指向本机 Visual Studio 提供的 CMake。

从工作台工程根目录执行：

```powershell
./algorithms/carExtractFeature/build.ps1 Build
./algorithms/carExtractFeature/build.ps1 Test
./algorithms/carExtractFeature/build.ps1 Deploy
```

重启工作台，导入 `examples/carExtractFeature/1.pcd` 或 `2.pcd`，选择“汽车特征提取：上层搭接边缘”，使用默认参数运行。若使用“参数预设 → 加载预设”，请选择 `examples/carExtractFeature/parameters.preset.json`；同目录的 `parameters.json` 是插件参数定义，不能直接加载为预设。开启宿主单步调试会显示六个主要阶段，普通运行不创建调试快照。

不需要重编工作台。构建脚本沿用已安装 FrameworkSDK 的编译器与依赖缓存；可通过 `-SdkRoot`、`-HostDirectory`、`-DependencyCache` 覆盖路径。`Package` 可生成含算法 DLL 和部署说明的 ZIP。

## 代码阅读顺序

识别计算集中在 [`core/LapEdge.cpp`](core/LapEdge.cpp) 的 `detect()`；它按实际执行顺序串起六步。建议先读 [`core/LapEdge.h`](core/LapEdge.h) 中的输入、截面、候选和输出结构，再回到以下函数：

1. `recover`：恢复扫描截面，生成局部坐标及点距；完整原始点与分析代表点同时保存。
2. `profileCandidates`：排除无邻截面支持的孤立点，再从每个截面的高度跳变提出候选。
3. `validateSurfaces`、`validateEndpoints`：分别验证上下层局部曲面和邻截面端点趋势。
4. `track`：在所有截面的候选和缺失状态之间选择一致路径。
5. `refineConnectedRawEdges`：回查原始密集点，确定朝断口的实测端点。
6. `smooth`：锚定可靠实测端点，对其余轨迹做受约束平滑，并验证短缺口插补。

[`MyAlgorithm.cpp`](MyAlgorithm.cpp) 只做宿主参数、取消/调试回调及输出图层转换；[`tools/Inspect.cpp`](tools/Inspect.cpp) 是脱离界面的 PCD 检查器。`RawDecision` 记录第五步为何取当前端点、回退或拒绝，CSV 中仍保留原有数字编码。

## 流程与约束

1. 从沿扫描线的点间差分鲁棒估计扫描平面法向；若截面内离散度校验失败，则根据扫描线末尾到下一线开头的空间回跳分组，对每条线中心化后用 PCA 重新估计法向。重新恢复后仍须通过截面校验；少量偏离扫描平面的点不会使整份点云失败。在法向坐标中分离扫描截面，验证单调性、截面内离散程度和截面间隔。对每截面超过 96 点的高密度输入，按横向宽度形成约 64 个分析区间，每区间选一个原始点代表局部中位轮廓；原始点及索引仍完整保留。`Scalar_field` 不被解释为帧号。乱序、平面不可分或大部分截面支持不足时明确失败。
2. 根据截面内局部点距与 MAD 估计噪声。双侧大间距点只有同时缺少邻截面支持时才排除。对轮廓断点两侧拟合局部直线，保留台阶、间距及端部连续性证据；每截面最多 5 个不同位置的正向候选，反向和被拒候选保留诊断原因。
3. 从前后各 2、4、7 个截面中分别采集上、下层内部点，避开端点与当前截面。数据充足且能显著降低残差时采用局部二次曲面，否则保留局部平面；使用最小充分尺度。邻域倍率连续调整空间权重，不把阈值变化直接映射成整数窗口突变。再独立检查邻截面已验证端部的趋势，排除虽在板面上、却因缺测而退到板内的孤立端点。
4. 二阶动态规划保留前两个已选候选及缺失状态。结合单截面证据、曲面残差、位移及趋势预测残差选择路径。无曲面支持的点不进入动态规划。过长的编号或物理距离缺口会重新分段；不足 3 个实测点的独立轨迹段不输出。
5. 在整段选点完成后，回到对应原始扫描截面，结合已分开的上、下曲面与原始点距建立上层点的局部连通分量。短段若属于上层、具备多个原始点及横向跨度，并得到至少两个邻截面同层原始点支持，实测点取短段朝缝隙的一端；只在单截面出现的零散点回退到连续板面的端部或舍弃。实测边界始终保留原始 XYZ。拟合轨迹只在连续实测截面段内部，使用当前截面以外、两侧对称的较长邻域做鲁棒趋势拟合；直线足以解释局部数据时保持直线，持续弯曲有足够证据时使用二次曲线，同时估计横向和高度，避免短窗口追随噪声产生局部弯折。遇到缺失截面先隔开两侧拟合，避免最后断段时端点被另一侧拉偏。拟合位置再以上层局部曲面、下层分离和最大偏移验证，不把各截面独立曲面的微小变化直接写进轨迹。已验证为独立短段的端点不跨断口移动。短缺口连接平滑后的两端，仅在两端均有至少两个邻截面端部支持、且两个局部曲面模型均支持插补位置时补点，首尾不外推。

当前输入契约要求有序扫描；法向可以手动覆盖，但不会因此接受随机打乱的点序。恢复截面 ID 不是原始采集时间戳。数据中整个扫描截面不存在时，按真实几何间距断段，不伪造原始采集帧号。

对于扫描端部不可见、持续性伪边缘或极稀疏区域，结果可能断段。只有可复核的采样边界可作为实测点；采样缺失时的真实物理端部位置不能仅由最近的现有点保证。

## 输出与调试

两个结果都是包含以下字段的 PCD，保留输入的坐标系和单位：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| x / y / z | float32 | 实测输出保持原始坐标；轨迹输出为拟合坐标 |
| source_index | uint32 | 输入点的从 0 开始索引；插补点为 UINT32_MAX |
| section_id | uint32 | 几何恢复的有序截面编号 |
| segment_id | uint32 | 从 0 开始的段号；两份输出分别编号 |
| confidence | float32 | 证据质量评分，不是正确概率 |
| interpolated | uint32 | 1 表示插补，0 表示实测或对应实测点的拟合坐标 |

最终视图同时显示半透明青蓝色原始点云、黄色实测边界、红色拟合连接线和绿色“最终轨迹点”。绿色点逐一对应第二份输出 PCD 中的拟合轨迹坐标（包括通过验证的短缺口插补点），不是只供显示的抽样标记；红线按段连接这些点，不跨断段。可在图层列表分别切换显隐或调整样式。插补点没有伪造的输入点索引。

运行结束后，工作台底栏显示“整体”与“算法”两项耗时：整体从点击运行到结果图层设置完成，包含输入转换与输出显示；算法只计 `lap::detect` 的识别计算。单步调试的快照制作及等待操作时间不计入算法耗时。算法耗时同时写入结果指标 `algorithm_elapsed_ms`（毫秒）；取消或在识别核心之前失败时不提供该指标。

六个调试阶段：恢复截面、孤立杂点、单截面候选、邻截面上下曲面验证、全局选点、最终轨迹。曲面验证阶段按拒绝原因分图层，并显示代表性上下曲面片段。`metrics` 包含采样尺度、估计法向、候选拒绝原因计数、输出点数与段数。

## 回归与真实样本检查

核心与插件测试覆盖：有噪声/无噪声、噪声小团、连续多截面的同高漂浮短段、高密度扫描后回查原始边缘、短缺口插补、长缺口断段、无搭接平滑面、反向扫描、旋转坐标、单位缩放、连续陡坡、错误扫描法向、乱序、非有限点、调试结果一致性、取消、参数和输出属性。`3.pcd` 的 CTest 回归验证 424,513 点恢复为 395 个截面、391 个实测边缘点，并检查拟合轨迹是否压制单截面尖跳及保留断段端点。较长趋势拟合将该样本的最大单截面二阶跳动从旧版约 0.321 降至约 0.181（输入坐标单位）；`4.pcd`、`5.pcd` 中相同形状区域的最大跳动也从约 0.265 降至约 0.181。三份样本的实测点、分段均不变，135→138 截面缺口不插补，缺口两端轨迹点保持对应的实测位置。平滑强度 0.8～1.5 的默认轨迹共同点 P95 位移不超过 0.007。此前扫描恢复修复时，`1.pcd`、`2.pcd` 的候选、实测点、轨迹 CSV 逐字节一致。

真实样本诊断工具（先把 `install/bin` 加入当前终端 PATH）：

```powershell
$env:PATH="$PWD/install/bin;$env:PATH"
./algorithms/carExtractFeature/build/bin/Release/CarLapEdgeInspect.exe `
  examples/carExtractFeature/1.pcd algorithms/carExtractFeature/build/real/default
./algorithms/carExtractFeature/build/bin/Release/CarLapEdgePluginTests.exe `
  examples/carExtractFeature/1.pcd algorithms/carExtractFeature/build/real/plugin
```

检查器输出 `report.json`、实测/轨迹 CSV、逐候选原因 CSV；插件测试工具输出含完整属性的 `measured.pcd`、`trajectory.pcd`，并验证真实样本调试开关结果逐字节相同。

`tools/ValidateSample.py` 使用 numpy/matplotlib，运行默认值与 9 组扰动（邻域、平滑、内部证据尺度分别取 0.8/1.2/1.5），输出 `sensitivity.json`、`overview.png`、`sections.png` 和截面 1000 局部图。记录共同截面 P95 位移以及新增/丢失截面数；这是重复性检查，不是绝对精度验收。当前样本没有人工标注真值，不报告识别准确率或亚毫米精度。

## 代码位置

- `core/LapEdge.cpp/.h`：独立数值核心、输入输出类型、阶段和取消回调。
- `MyAlgorithm.cpp`：插件适配、参数映射、调试图层和 PCD 属性。
- `parameters.json`：参数界面定义；`CMakeLists.txt` 和 `build.ps1`：独立编译测试部署。

常规参数：上方方向、邻域尺度倍率、平滑强度、最大补点截面数。高级参数：扫描法向手动覆盖。核心内部的 `evidenceScale` 仅供敏感性回归使用，不作为用户必须调整的阈值。
