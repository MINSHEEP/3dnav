std_msgs/Header header
# 电量归一化为 [0,1]，SDK 原始 soc 的单位为百分比。
float32 percentage
float32 temperature
float32 voltage
uint16 cycle_count
# 手册未定义 charged 的全部枚举，保留原值。
uint16 charged
