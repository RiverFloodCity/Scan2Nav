# Navigation Development Rules

本项目中的导航代码分为公共领域接口和内部实现辅助函数。

## 导入规则

公共领域对象可以从包级导入：

```python
from src.navigation import OccupancyGridMap
from src.navigation import TargetRegistry
from src.navigation import NavigationCommand
```

内部数学辅助函数应从具体模块导入：

```python
from src.navigation.pose_utils import quaternion_wxyz_to_yaw
from src.navigation.pose_utils import normalize_yaw
```

不要为了每个内部辅助函数不断扩大 `src.navigation.__init__` 的公开 API。

## 包级导出

如果新脚本确实需要：

```python
from src.navigation import SomeName
```

则必须同步更新：

- `src/navigation/__init__.py`
- `src/navigation/__all__`

否则 Script Editor 或普通 Python 都无法保证该包级导入稳定可用。

## 回归测试

`tests/test_navigation_import_contracts.py` 会静态扫描 `Scripts/` 下所有 `.py` 文件，找出：

```python
from src.navigation import X
```

并断言 `src.navigation` 中确实导出了 `X`。

该测试不会执行 Isaac Sim 脚本，只做 AST 静态解析。

## Script Editor 缓存说明

Script Editor 中的 `sys.path` 和 `sys.modules` 清理可以解决“加载了旧模块或同名包”的问题，但不能解决“名称本来没有从 `__init__.py` 导出”的问题。

因此，包级公开导入必须由 `__init__.py` 明确维护；内部工具函数则应从具体模块导入。

## 自动导航执行器

- `PathFollower` 必须保持纯数学，不导入 Isaac Sim，不访问 `builtins`。
- `NavigationExecutor` 可以适配 Isaac Sim，但 import 模块和构造函数不得注册回调或发运动命令。
- 只有 `Scripts/36_start_navigation.py` 可以注册 `builtins.SCAN2NAV_EXECUTOR`。
- 所有真实底盘运动命令必须经过 `builtins.TIANYI_BASE_CTRL.set_cmd(vx, wz)`。
- `Scripts/95_cancel_navigation.py` 取消后会删除 `builtins.SCAN2NAV_EXECUTOR`，下一次启动重新创建执行器。
