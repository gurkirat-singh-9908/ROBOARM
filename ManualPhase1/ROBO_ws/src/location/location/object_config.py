"""
object_config.py  —  per-object detection config (data-driven thresholding)

The location element is general: it knows nothing about tomatoes. What an
object looks like (HSV colour bands, area floor, real size, pick threshold)
lives in a small YAML file named after the object, e.g. ``tomato.yaml``.

Lookup order for ``<object>.yaml``:
  1. user dir   — ``$ROBOARM_OBJECTS`` or ``~/.roboarm/objects`` (writable;
                  where ``tune`` saves freshly-calibrated values)
  2. package    — the ``objects/`` shipped with the ``location`` package
                  (read-only defaults so e.g. tomato works out of the box)

If neither has the file, ``load()`` raises ``MissingObjectConfig`` so the
caller can tell the user to run the tuner. Nothing is hard-coded per object.
"""

import os
from pathlib import Path

import yaml


class MissingObjectConfig(FileNotFoundError):
    """Raised when no config YAML exists for the requested object."""


def user_objects_dir() -> Path:
    """Writable dir for tuned configs. Override with $ROBOARM_OBJECTS."""
    env = os.environ.get('ROBOARM_OBJECTS')
    return Path(env) if env else Path.home() / '.roboarm' / 'objects'


def _package_objects_dir() -> Path | None:
    """Read-only defaults shipped in the installed package share, if found."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('location')) / 'objects'
    except Exception:
        return None


def config_path(name: str) -> Path | None:
    """Return the path the config for ``name`` would be read from, or None."""
    candidates = [user_objects_dir() / f'{name}.yaml']
    pkg = _package_objects_dir()
    if pkg is not None:
        candidates.append(pkg / f'{name}.yaml')
    for p in candidates:
        if p.is_file():
            return p
    return None


def load(name: str) -> dict:
    """Load and validate the config for ``name``. Raise MissingObjectConfig."""
    path = config_path(name)
    if path is None:
        searched = [str(user_objects_dir())]
        pkg = _package_objects_dir()
        if pkg is not None:
            searched.append(str(pkg))
        raise MissingObjectConfig(
            f"No config for object '{name}'. Looked in: {', '.join(searched)}. "
            f"Run:  ros2 run location tune --ros-args -p object:={name}")
    with open(path, 'r') as fh:
        cfg = yaml.safe_load(fh) or {}
    cfg.setdefault('object', name)
    return cfg


def save(name: str, cfg: dict) -> Path:
    """Write ``cfg`` to the user objects dir as ``<name>.yaml`` and return path."""
    out_dir = user_objects_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f'{name}.yaml'
    cfg = dict(cfg)
    cfg['object'] = name
    with open(path, 'w') as fh:
        yaml.safe_dump(cfg, fh, default_flow_style=False, sort_keys=False)
    return path
