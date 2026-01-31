基于 STM32F103C8T6 的裸机平衡车项目
1. 项目硬件清单
   核心控制板：STM32F103C8T6

 姿态检测传感器：MPU6050（自研姿态解算函数）

 电机驱动模块：TB6612FNG

 执行电机：JGB37-520 霍尔编码器直流减速电机

 PCB / 外观结构：亚克力板（参考 B 站 “会飞的鱿鱼 03” 设计）

2. 参考来源
 PCB 与平衡车外观设计参考 B 站 UP 主 “会飞的鱿鱼 03” 的开源内容：

 视频链接：https://www.bilibili.com/video/BV1Gc411v73h/?buvid=XU0B0550DD42BB456B98CEB3E250CDE28A135&from_spmid=playlist.playlist-detail.0.0&is_story_h5=false&mid=H5jIrBzya46VvB4E2OzdGg%3D%3D&p=13&plat_id=116&share_from=ugc&share_medium=android&share_plat=android&share_session_id=800114d3-2ae1-4969-bca2-a28c1a020ec0&share_source=WEIXIN&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1769521111&unique_k=dycAENL&up_id=405949964

4. 代码创新点
 相较于参考来源的代码，本项目的核心创新 / 修改点：

 集成 vofa + 在线调参功能，支持实时参数调试

 新增摇杆控制逻辑，扩展平衡车操控方式

 优化电机驱动逻辑，增加电机死区补偿处理

 自研 MPU6050 姿态解算函数，适配本项目硬件特性

5. 项目说明
 本人为首次开发此类平衡车项目，代码和硬件实现仍有优化空间，欢迎各位开发者 / 大佬提出指教和改进建议。

6.下面是我平衡车的展示
https://www.bilibili.com/video/BV1iE6GBFErF?buvid=XU0B0550DD42BB456B98CEB3E250CDE28A135&from_spmid=dt.space-dt.video.0&is_story_h5=false&mid=H5jIrBzya46VvB4E2OzdGg%3D%3D&plat_id=116&share_from=ugc&share_medium=android&share_plat=android&share_session_id=9bf5ef0f-e17e-4ffe-a187-2a5905ccbaa6&share_source=WEIXIN&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1769528171&unique_k=YNPF1FN&up_id=698438521

7.我会在我CSDN中更新对应的笔记和遇到的bug，感兴趣的可以关注一下：爱吃葱的羊  用户id是2401_86599913

