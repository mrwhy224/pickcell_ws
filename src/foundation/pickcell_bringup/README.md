# PickCell bringup

Bringup reads one file:

```text
pickcell_config/config/application.yaml
```

That file contains every node currently launched, each node's mode, all topic
names, and the mock coordinates. Run it without arguments:

```bash
ros2 launch pickcell_bringup system.launch.py
```
