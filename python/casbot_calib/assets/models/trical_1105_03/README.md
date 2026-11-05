# 1105_03实际三表与球架

来源为仓库`references/cad/trical_1105_03/1105_03.stp`及配套坐标登记。此处是运行用米制STL、球/表轴登记和Coal凸包缓存；无需FreeCAD。大型STL/NPZ由LFS管理。

一个表架左右依次换装，左相对STEP+90°、右0°；普通手拆除，未使用侧裸法兰停放。测杆为工作接触状态，自由伸出全行程未量测。工具总质量和质心未知；CAD标签不作为实测负载。

`table_holes.json`和`pillar_holes.json`从源CAD圆柱面提取。孔位及平台升降是候选布局参数，不是实物已调好的位置。保持同一批双臂实验内布局锁定。`seeds.json`为原review种子，`orientation_seeds.json`为几何搜索种子；二者均不是采集数据或标定结果。

缓存使用逐部件外凸包，两个凹形联轴器采用各8个BREP单元再外包、加0.2 mm余量。普通10 mm、选定近接2 mm、三对有界接触5 mm由`trical_assembly.py`逐对应用，不导入review的腰/肩排除。

目前所测候选没有双臂八组齐全方案；原位和平台+50 mm各5/8通过连续门禁。使用及边界见仓库`docs/trical_1105_hole_trials.md`。不得将本目录存在、review种子或名义粗筛当作完整批次/实机放行。
