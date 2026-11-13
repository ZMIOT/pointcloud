# ExtractFeature2 示例

1. 重启 `E:/CODE/pointcloudExtractFeature/install/bin/Launcher.exe`。
2. 右侧算法选择 **ExtractFeature2：三面结构焊缝提取**。
3. 导入本目录 `original_scan.pcd`，单位选 `mm`。
4. 使用默认参数，加载兼容预设 `parameters.preset.json`，或加载 `parameters.stable-grid.preset.json` 试用固定网格。
5. 点击“运行算法”。预期左侧和右侧各 89 点。

预设默认使用 `legacy`，用于复现修改前插件坐标。需要比较平滑效果时，把“最终轨迹后处理”改为“鲁棒 RTS（实验）”；输出中的 NIS、修正量和质量标志仅用于定位待复核站位。

稳定分箱预设仍使用 `legacy` 后处理，只改变候选提取阶段。它把 `profileReferenceScale` 固定为 1.2；比较不同 `profileBinSize` 时不要同步修改这个参考尺度。

点云来自 `E:/CODE/extractFeature2/sample/3.txt`。原始文件没有帧号表头，按照原解析器使用前三列 XYZ；其第四列不猜测为帧号。本示例运行不依赖原始 TXT 或外部 manifest。

调试：勾选“高级模式”和“启用单步调试”，重新选中原始点云再运行。
算法源码、编译与部署说明见 [独立算法工程](../../algorithms/extract_feature2/README.md)。
