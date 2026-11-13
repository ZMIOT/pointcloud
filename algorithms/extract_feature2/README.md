# ExtractFeature2 算法插件

本工程迁移自 `E:/CODE/extractFeature2/src`，使用其中的 `SeamDetector` 三面结构算法；通用核心改动会同步维护，源码哈希记录在 `source-files.json`。插件 ID 为 `weld.extract_feature2`，与之前的 `weld.frame_seam` 是不同算法，参数预设不能混用。

当前插件版本为 1.5.0。第一阶段先在整船的“纵向轴—剖面 V 轴”投影中恢复连续的上外壁和底部外壁，并沿闭合包络裁掉外部支架、飞点与扫描拖影；裁剪结果作为独立图层和运行指标提供，后续三面结构流程只读取包络内点云。带帧号的输入仍由每个目标帧联合相邻 3–5 帧构造局部截面，输出绑定目标帧；主软件和独立工具均支持 `index x y z target`，其中 `target` 解释为帧号。默认“船体内部开口方向”为“自动”，按跨截面覆盖率和几何代价统一选择同一凹向；坐标方向明确时可指定剖面 V 正/负方向。默认“剖面分箱模式”为“原始滚动分箱（兼容）”，默认“最终轨迹后处理”为“原始算法（稳定）”。

## 先运行一次

1. 重启 `E:/CODE/pointcloudExtractFeature/install/bin/Launcher.exe`。
2. 右侧“处理算法”选择 **ExtractFeature2：三面结构焊缝提取**。
3. 点击“导入点云”，打开 `E:/CODE/pointcloudExtractFeature/examples/extract-feature2/original_scan.pcd`，单位选择 `mm`。
4. 保持默认参数即可；也可以在“参数预设 → 加载预设”中打开同目录的兼容预设 `parameters.preset.json`，或加载 `parameters.stable-grid.preset.json` 试用稳定分箱。
5. 点击“运行算法”。示例预期输出左右各 89 点。

这一版的普通 PCD/PLY 输入使用空间切片，不需要 `manifest.json`。不要加载旧算法的参数预设，也不要把之前提取好的 89 点轨迹当作原始扫描输入。

## 后续修改哪里

| 内容 | 文件 |
| --- | --- |
| 核心计算流程 | `core/SeamDetector.cpp` |
| 可选鲁棒 RTS 与几何限幅 | `core/SeamSmoother.cpp` |
| 算法参数及其 C++ 默认值 | `core/SeamTypes.h` |
| 界面参数名称、约束、分组及默认值 | `parameters.json` |
| JSON 参数到 C++ 配置的映射 | `Configuration.h` |
| 插件入口、调试回调、输出适配 | `Plugin.cpp` |

新增参数时，同时修改 C++ 配置、JSON 定义、映射和计算使用处。浮点参数的 JSON 类型是 `double`，映射到算法时转为 `float`。常规参数全部参与本算法；未接入历史轨迹输入，因此没有显示历史权重等无效控件。原项目 GUI/CLI 的普通单次运行也不传历史轨迹。

## 单独编译和部署

在工作台工程根目录执行：

```powershell
./algorithms/extract_feature2/build.ps1 Build
./algorithms/extract_feature2/build.ps1 Test
# 关闭工作台再部署，随后重启工作台
./algorithms/extract_feature2/build.ps1 Deploy
# 可选：交付算法 DLL 时打 ZIP 包
./algorithms/extract_feature2/build.ps1 Package
```

DLL 输出：`build/bin/Release/plugins/algorithm/ExtractFeature2Algorithm.dll`。
部署目录：`E:/CODE/pointcloudExtractFeature/install/bin/plugins/algorithm`。
参数 JSON 嵌入 DLL，不需要在主程序旁另放参数定义文件。主界面无需重新编译，日常开发无需打 ZIP。

独立工程依赖已安装 FrameworkSDK 和同版本 Qt/PCL/OpenCV 开发依赖；`build.ps1` 默认只读取宿主依赖缓存，可用 `-SdkRoot`、`-HostDirectory`、`-DependencyCache` 覆盖。使用 CMake 时传入 `FrameworkSDK_DIR`、`Qt5_DIR`、`PCL_DIR`、`OpenCV_DIR`。Windows PCL 包缺少 FLANN include 时，设置 `EXTRACT2_FLANN_INCLUDE`。

## 输入和原始帧

- 普通 PCD/PLY：和原算法一样按纵向轴、截面步长、截面厚度切片。
- 有原始帧的 TXT/CSV：工作台可直接导入，也可用下面的转换器生成包含 `frame_index` 字段的 PCD。帧号可来自 `frameIndex`、`frame_index`，或采集软件的 `index x y z target` 表头（此格式中 `target` 明确定义为帧号）；无表头文本仍只读取前三列 XYZ，不猜测额外列含义。

```powershell
$env:PATH="E:\CODE\pointcloudExtractFeature\install\bin;$env:PATH"
& ./algorithms/extract_feature2/build/bin/Release/ExtractFeature2Convert.exe input.txt output.pcd
```

转换器复用原项目解析器，保留 XYZ、点顺序及明确存在的帧号，其他文本列不会作为点云属性导出。`frame_index` 在 PCD 中为 int32，要求非负、同帧连续、帧号递增；损坏帧字段会拒绝运行。帧模式忽略纵向空间截面的步长/厚度，每个目标帧联合前后相邻帧形成局部截面：中部使用 5 帧，首尾自然使用 3–4 帧；横截面内部为建立有序轮廓而进行的细粒度采样仍保留。参数面板仍显示空间切片值，方便切换普通点云。长度参数使用输入坐标单位，不自动换算。

## 第一阶段：整船外壁包络裁剪

有限点、逐帧统计离群和体素处理只作为建立稳定投影的基础。随后算法把点投影到纵向轴 A 与剖面竖向轴 V：纵向为 X/Y 时 V 为 Z，纵向为 Z 时 V 为 Y。算法先寻找投影中最大的八邻域连通主体，再以主体每个纵向网格的最低/最高位置形成底部与上部外壁；跨纵向只保留具有连续厚度的最长主体，对两条边界分别做局部中值平滑，再加安全余量形成闭合包络。只有包络外的点被删除，包络内保留原始三维坐标。

默认参数按当前毫米样本设置：网格 2 mm、单格至少 8 点、主体最小投影厚度 8 mm、外扩余量 4 mm。细长扫描拖影即使局部点数较多，只要没有形成连续厚度也不会成为主体。若投影退化、主体长度不足或候选包络保留点少于 20%，算法回退到常规预处理点云并上报 `hull_envelope_unavailable`；这只是当前输入未形成可靠统计包络，不表示船体外壁不存在。调试步骤 `S04` 同时显示包络内点云、上部外壁和底部外壁。运行指标记录输入、保留、删除点数与包络列数，不把删除点数变多解释为识别更正确。

输出包括左右两份轨迹、`confidence`、`segment_id`、`rts_shift`、`innovation_nis`、`position_sigma`、`quality_flags`、`wall_rms`、`floor_rms`；原始帧模式还包含 `frame_index`。线段按分段绘制，不跨缺口连接。RTS 与质量字段是内部一致性证据，不能代替人工标注真值。

## 船体内部与起伏底面

候选仍从每个截面的“左侧壁—中央底面—右侧壁”三面结构产生，但 1.3.0 会给候选标记开口凹向，并在路径优化前只保留一个跨截面一致的方向。自动模式优先选择覆盖站位更多、几何代价更低的方向；若两个方向证据接近，诊断会提示人工确认，不会把内部判断写成已验证事实。纵向轴为 X 时剖面 V 是 Z，纵向轴为 Y 时 V 是 Z，纵向轴为 Z 时 V 是 Y。已知船体内部朝向时，明确选择正/负 V 比自动模式可靠。

底面不要求全局平坦。候选阶段使用轮廓区域生长和左右边界的局部切线；最终约束在每个站位单独拟合局部底面。固定 `floorPatchSize` 邻域因小山包曲率导致 RMS 超限时，会依次尝试 75% 和 50% 的窗口，但不会放宽 `floorPatchMaxRms`，也不会绕过交点位移与原始点支持门槛。缩小后仍不满足条件时保留贴壁结果并标记底面证据不足。

## 鲁棒 RTS 模式

RTS 以纵向站位为自变量，对截面内两个坐标分别使用“位置 + 斜率”状态。低置信度、插补或缺少局部几何约束的点会获得更大的观测噪声；NIS 超过门限时观测继续降权。最终修正还受以下条件限制：

- 不改变纵向站位；
- 单点修正不超过 `rtsMaxCorrection`；
- 已接受的侧壁/底面残差不超过 `rtsGeometryTolerance`；
- 左右间距不小于 `minSeamSeparation`。

插件会通过 `fw::Diagnostics` 上报局部几何证据不足、创新异常和被限幅的修正。异常来源统一为 `undetermined`，需要结合原始点云复核。

## 稳定剖面分箱模式

`legacy_rolling` 完整保留旧滚动分箱以及随 `profileBinSize` 一起变化的拟合和几何门槛。`stable_grid` 使用整云统一的横向网格原点，并以 `profileReferenceScale` 固定拟合窗口、断点距离、候选修正、侧壁取点和原始点支持尺度。

推荐先保持 `profileReferenceScale=1.2`，仅比较相邻的 `profileBinSize`，例如 0.8、1.0、1.2、1.4。该模式减少参数之间的隐式联动，但没有人工标注时仍不能用“更平滑”证明定位更正确。稳定分箱和鲁棒 RTS 是两个独立开关：前者作用于候选提取，后者只处理最终轨迹。

开启单步调试后，S06 会显示 `profile_bins`、`mean_bin_support`、`low_support_bins`、`profile_bin_size` 和 `profile_geometry_scale`，用于判断分箱过细、过粗或几何尺度是否意外联动。

## 单步调试

主界面开启“高级模式 → 启用单步调试”。默认“主要阶段”展示 S00–S18 每类步骤首次出现，其中 S04 是整船外壁包络裁剪；某阶段未执行就不生成它的窗口。选择“全部步骤”可逐次查看所有截面与局部拟合回调。每个步骤由工作台弹窗、暂停、高亮、保存快照；下一步、继续和停止均使用工作台控制。

调试关闭时不创建 `DebugTrace`、不构建显示快照。启用时取消原算法累积局部面步骤副本的行为，由工作台管理显示快照预算。原算法的截面诊断元数据仍在本次运行内保留。原始帧中间步骤可能显示算法内部展开后的坐标，最终输出恢复到真实扫描平面。

取消检查独立于调试，放在阶段、截面循环和轨迹修正循环；单次 PCL 滤波内部无法强行中断。迁移不改变几何计算公式。

## 验证入口

`tests/PluginTests.cpp` 验证参数、普通/主要阶段/详细调试结果一致、取消、空输入、非法参数、原始帧与输出属性及快照独立性，并覆盖稳定分箱在 0.8、1.2、1.6 三种采样宽度下的插件执行。可选 `EXTRACT_FEATURE2_GUI_SMOKE=ON` 编译已有主界面 DLL 的集成检查工具，仅框架维护者需要宿主头文件，普通插件构建不需要。

完整真实输入的核对记录位于宿主 `build/extract2-real`，原 CLI 基线位于 `build/extract2-baseline`。

本次 704,808 点示例输出左右各 89 点。1.3.0 自动模式观察到负 V/正 V 开口候选分别覆盖 92/2 个站位，未报告方向歧义；相对 1.2.0，左轨迹逐点相同，右轨迹有 1/89 个点变化，最大三维位移约 2.90（输入点云单位），这是统一凹向筛选带来的行为变化，尚无人工真值证明新位置更正确。合成回归覆盖凹向相反的内外结构及局部高斯凸起，自适应窗口必须在未放宽 RMS 的情况下通过底面交点约束。真实运行报告位于宿主 `build/extract2-interior-floor-validation/report.json`。

1.4.0 对 `extractFeature2Data` 的 11 份 `index x y z target` 数据逐份按帧运行，全部成功；3–5 帧窗口在 1/4 号对照中降低了相邻帧跳变，但 2 号的局部几何弱证据增加，说明固定 5 帧并不适合所有姿态变化区。1 号连续主体覆盖 18.3%，2 号覆盖 40.2%，其余为 54.8%–63.6%；2、10、11 号存在需复核的候选切换。没有人工标注，以上仅为运行事实和内部一致性证据。汇总见宿主 `build/extract2-frame-validation/validation-summary.md`，安装版 1.4.0 的代表性报告位于 `build/extract2-1.4-installed-validation`。

上一版已部署的 `FrameSeamAlgorithm.dll` 已移至宿主 `install/disabled-plugins/frame_seam` 备份，旧源码仍在 `algorithms/frame_seam`。需要恢复时退出工作台，将备份 DLL 放回 `install/bin/plugins/algorithm` 后重启。
