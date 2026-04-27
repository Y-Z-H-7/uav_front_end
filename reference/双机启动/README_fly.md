### 前置条件
```bash
# 1. 安装tmux（如果未安装）
sudo apt-get install tmux

# 2. 设置脚本执行权限
sudo chmod +x ~/startup_fly.py

### 启动无人机
```bash
cd ~/双机启动
python3 startup_fly.py start
```

脚本将自动创建10个tmux窗口并准备好所有命令。

### 停止所有程序
```bash
python3 startup_fly.py stop

# 或在tmux内执行
Ctrl+B, :kill-session -t fly_drone
```

## 📋 窗口说明与启动顺序
每个窗口的命令都已经准备好，按 **Enter** 执行

## 🎮 使用tmux的快捷键
### 窗口导航
```
Ctrl+B, n        # 下一个窗口
Ctrl+B, p        # 上一个窗口
Ctrl+B, 0-9      # 跳到指定窗口（0-9）
Ctrl+B, w        # 列出所有窗口选择
```

### 其他操作
```
Ctrl+B, c        # 创建新窗口
Ctrl+B, x        # 关闭当前窗口
Ctrl+B, d        # 退出tmux（保持后台运行）
Ctrl+B, [        # 进入滚动模式（查看历史输出）
```

### 进入滚动模式后
```
空格/Page Down   # 向下翻
b/Page Up        # 向上翻
q                # 退出滚动模式
```

## 📊 状态检查

### 查看所有窗口状态
```bash
python3 startup_fly.py status
# 或
tmux list-windows -t fly_drone
```

### 在后台重新附加
```bash
# 如果你在tmux中按了Ctrl+B, d，可以重新进入
tmux attach-session -t fly_drone

# 从新的终端窗口查看某个窗口的输出
tmux send-keys -t fly_drone:3 "rostopic list" Enter
```

## 📝 调试技巧

### 查看特定窗口的完整输出
```bash
# 显示窗口0的最后100行
tmux capture-pane -t fly_drone:0 -p -S -100

# 保存窗口输出到文件
tmux capture-pane -t fly_drone:0 -p > window0_output.txt
```

### 在后台继续运行
```bash
# 在tmux中按 Ctrl+B, d 可以分离session（保持后台运行）
# 然后关闭终端窗口也没问题

# 重新进入
tmux attach-session -t fly_drone
```

### 获取实时ROS话题列表
```bash
# 在任意窗口中执行（不中断当前命令）
tmux send-keys -t fly_drone:0 "rostopic list" Enter
```

### 完整重启（保留日志）
```bash
python3 startup_fly.py stop
sleep 2
python3 startup_fly.py start
```

## 🛠️ 自定义修改

如果需要修改命令，编辑 `startup_fly.py` 中的 `TERMINALS` 列表：

```python
TERMINALS = [
    {
        "name": "window_name",           # 窗口名称
        "setup_cmds": ["cmd1", "cmd2"], # 前置准备命令（自动执行）
        "cd": "cd /path/to/dir",         # 切换目录（自动执行）
        "main_cmd": "your command"       # 主命令（需手动按Enter）
    },
    # ... 更多窗口
]
```
