"""launch 文件共用的环境准备工具。"""

import os
import re

# 软件渲染用的 Mesa 环境脚本：Gazebo GUI 和 RViz2 里的 OGRE 都依赖它的 EGL。
# 这是本机环境的一部分，读不到（换台机器、没装 Mesa）就静默跳过，不影响仿真本身。
MESA_ENV_SCRIPT = '/opt/mesa-24.3.4/env.sh'

_EXPORT_RE = re.compile(r'^export\s+([A-Za-z_][A-Za-z0-9_]*)=(.*)$')
_VAR_RE = re.compile(r'\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}')


def mesa_environment(script=MESA_ENV_SCRIPT):
    """把 Mesa 环境脚本里**未被注释掉**的 export 解析成 {变量名: 值}。

    值里的 ${VAR} 和 ${VAR:-默认值} 按当前环境展开；脚本不存在时返回空字典，
    调用方不需要特判。这样用户直接在终端 `source env.sh` 过也没问题，
    在这里再设一遍是等价的。
    """
    if not script or not os.path.isfile(script):
        return {}

    with open(script, encoding='utf-8') as handle:
        lines = handle.readlines()

    environment = {}
    for line in lines:
        match = _EXPORT_RE.match(line.strip())
        if match is None:
            continue
        name, value = match.group(1), match.group(2).strip()
        value = _VAR_RE.sub(
            lambda m: os.environ.get(m.group(1), m.group(2) or ''), value)
        environment[name] = os.path.expandvars(value)
    return environment
