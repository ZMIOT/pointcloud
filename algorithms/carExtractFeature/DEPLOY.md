# 部署上层搭接边缘算法

将 `plugins/algorithm/CarExtractFeatureAlgorithm.dll` 放到兼容工作台的 `bin/plugins/algorithm`，重启工作台后选择“汽车特征提取：上层搭接边缘”。

要求与 FrameworkSDK 1 相同的 Release x64 ABI（MSVC v142、Qt 5.14.1、PCL 1.11.0、OpenCV 4.2.0）；依赖使用宿主 bin 中的现有 DLL，不必复制整套开发库。

输入一份保留原始扫描点序的 PCD；默认 +Z 为上方。输出黄色实测边界点和绿色最终轨迹点，红线连接绿色点供观察。最多插补 3 个连续缺失截面，长缺口保持断开，质量评分不是正确概率。

参数已嵌入插件，不需要另行复制 JSON。不要用单幅截图代替实际数据的精度验收；原始 XYZ 及输出坐标单位不自动换算。
