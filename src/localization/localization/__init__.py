"""Importable Python for the `localization` package.

`localization` is an ament_cmake package - it owns launch files, YAML configs
and the saved maps, none of which belong to a Python package - so this module
is installed by `ament_python_install_package()` in CMakeLists.txt rather than
by a setup.py. The split is deliberate: everything in here imports no ROS at
all and is unit-tested without a ROS graph (see `test/`), while the nodes that
use it live in `scripts/` and are installed as plain programs, the same way
`slam_lifecycle_watchdog.py` already is.
"""
