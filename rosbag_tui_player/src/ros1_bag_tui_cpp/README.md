# C++ Bag 播放器 TUI（基于 ROS1 + ncurses）


- Space（空格）：播放 / 暂停

- ← / →：按步长向后 / 向前单步前进（以 step size 为窗口发布消息区间 [t0, t1)）

- + / -：步长 ×2 / ÷2（最小/最大步长与 roself 相同）

- 0：步长重置为 1 秒

- Shift + ← / →：跳转总时长的 10%（只移动，不发布消息）

- L：循环播放开/关

    - 当没有设置播放片段（segment）时，循环整个 bag

    - 当设置了 segment 时，只循环 segment 区间

- Shift + 1..9：设置书签（bookmark）

- 1..9：跳转到书签

    - 若当前处于「Segment Modify（片段修改）」模式：则按 1..9 选择 A/B 片段点

- C / c：片段（segment）状态机控制：

    - Off → Modify

    - Modify 或 On 状态再按 C → Off 并清除片段

- R / r：跳转到开始位置

    - 若有 segment：跳到 segment 起点

    - 若无 segment：跳到 bag 的 t=0

- q：退出程序