# 现场联调入口

- **[SHM现场逐例确认](shm/README.md)**：`bash fieldwork/shm/run.sh list`。Orin本机直接运行，不需要ROS、域号或distrobox。
- **[原ROS/SDK现场工具](ros/README.md)**：`bash fieldwork/ros/run.sh --help`。原根目录`run.sh`、`capture_zero.sh`、`return_zero.sh`保留转发，用法不变。

SHM用例范围及结论台账见[逐组确认手册](../docs/shm_field_validation.md)。原ROS的保存零位仍位于`fieldwork/zero.yaml`，不自动用于SHM。
