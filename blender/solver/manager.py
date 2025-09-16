"""Solver/session management utilities for the HinaCloth extension."""
from __future__ import annotations

import importlib
import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple

import bpy
import numpy as np

from ..props.settings import HinaClothSettings

__all__ = [
    "SolverUnavailableError",
    "SolverSession",
    "FrameCache",
    "acquire_session",
    "release_session",
    "release_all",
    "apply_positions_to_object",
    "bake_simulation",
    "clear_frame_cache",
    "frame_change_handler",
]

_LOGGER = logging.getLogger("hinacloth")
_MODULE_NAME = "xpbd_core"
_BACKEND_METHOD = {
    "NATIVE": "step_native",
    "TBB": "step_tbb",
    "AVX2": "step_avx2",
}


class SolverUnavailableError(RuntimeError):
    """Raised when the compiled XPBD solver module cannot be imported."""


@dataclass
class SolverSession:
    scene_key: int
    object_name: str
    backend: str
    config_signature: Tuple
    simulator: object

    def apply_config(self, config: dict) -> None:
        self.simulator.set_time_step(config["time_step"])
        self.simulator.set_substeps(config["substeps"])
        self.simulator.set_solver_iterations(config["solver_iterations"])
        self.simulator.set_velocity_damping(config["velocity_damping"])
        self.simulator.set_gravity(config["gravity"])
        self.simulator.enable_distance_constraints(config["enable_distance_constraints"])

    def reset(self) -> None:
        self.simulator.reset()

    def step(self, steps: int) -> np.ndarray:
        method = getattr(self.simulator, _BACKEND_METHOD[self.backend])
        method(steps)
        return self.positions()

    def positions(self) -> np.ndarray:
        return np.array(self.simulator.positions(), copy=True)

    def dispose(self) -> None:
        _LOGGER.debug("Disposing solver session for object %s", self.object_name)

    def ensure_mesh_layout(self, obj: bpy.types.Object) -> None:
        expected = int(self.simulator.particle_count())
        actual = len(obj.data.vertices)
        if expected != actual:
            raise RuntimeError(
                f"Cloth object '{obj.name}' vertex count {actual} does not match solver particles {expected}."
            )


@dataclass
class FrameCache:
    object_name: str
    backend: str
    config_signature: Tuple
    frames: Dict[int, np.ndarray] = field(default_factory=dict)

    def store(self, frame: int, positions: np.ndarray) -> None:
        self.frames[int(frame)] = np.array(positions, copy=True)

    def sample(self, frame: int) -> Optional[np.ndarray]:
        return self.frames.get(int(frame))


_SESSIONS: Dict[Tuple[int, str], SolverSession] = {}
_FRAME_CACHES: Dict[int, FrameCache] = {}
_FRAME_HANDLER_REGISTERED = False


def _load_solver_module():
    try:
        module = importlib.import_module(_MODULE_NAME)
    except ImportError as exc:  # pragma: no cover - executed inside Blender runtime
        raise SolverUnavailableError(
            f"Unable to import '{_MODULE_NAME}' module. Ensure the pybind solver is installed."
        ) from exc
    if not hasattr(module, "XPBDSimulator"):
        raise SolverUnavailableError(
            f"Module '{_MODULE_NAME}' does not expose XPBDSimulator class."
        )
    return module


def _build_session(scene: bpy.types.Scene, settings: HinaClothSettings, backend: str, obj: bpy.types.Object) -> SolverSession:
    module = _load_solver_module()
    config = settings.as_solver_config()
    simulator = module.XPBDSimulator(config["grid_width"], config["grid_height"], config["spacing"])
    session = SolverSession(
        scene_key=scene.as_pointer(),
        object_name=obj.name,
        backend=backend,
        config_signature=settings.solver_signature(),
        simulator=simulator,
    )
    session.apply_config(config)
    session.reset()
    session.ensure_mesh_layout(obj)
    _LOGGER.info(
        "Created solver session for %s using backend %s (grid %dx%d).",
        obj.name,
        backend,
        config["grid_width"],
        config["grid_height"],
    )
    return session


def acquire_session(
    scene: bpy.types.Scene,
    settings: HinaClothSettings,
    backend: str,
    obj: bpy.types.Object,
    *,
    force_rebuild: bool = False,
) -> SolverSession:
    key = (scene.as_pointer(), obj.name)
    session = _SESSIONS.get(key)
    config_signature = settings.solver_signature()
    if (
        force_rebuild
        or session is None
        or session.backend != backend
        or session.config_signature != config_signature
    ):
        if session is not None:
            session.dispose()
        session = _build_session(scene, settings, backend, obj)
        _SESSIONS[key] = session
    else:
        session.apply_config(settings.as_solver_config())
        session.ensure_mesh_layout(obj)
    return session


def release_session(scene: bpy.types.Scene, obj: bpy.types.Object) -> None:
    key = (scene.as_pointer(), obj.name)
    session = _SESSIONS.pop(key, None)
    if session is not None:
        session.dispose()


def release_all() -> None:
    for session in _SESSIONS.values():
        session.dispose()
    _SESSIONS.clear()
    _FRAME_CACHES.clear()
    _prune_frame_handler()


def apply_positions_to_object(obj: bpy.types.Object, positions: np.ndarray) -> None:
    if obj.type != "MESH":
        raise TypeError("Solver target object must be a mesh.")
    mesh = obj.data
    required = len(mesh.vertices) * 3
    flat = np.asarray(positions, dtype=np.float32).reshape(-1)
    if flat.size != required:
        raise ValueError(
            f"Expected {len(mesh.vertices)} vertex positions, received {flat.size // 3}."
        )
    mesh.vertices.foreach_set("co", flat)
    mesh.update()


def bake_simulation(
    scene: bpy.types.Scene,
    settings: HinaClothSettings,
    backend: str,
    obj: bpy.types.Object,
    progress_callback: Optional[Callable[[int, int, int], None]] = None,
) -> FrameCache:
    session = acquire_session(scene, settings, backend, obj, force_rebuild=True)
    start, end = settings.frame_range(scene)
    cache = FrameCache(obj.name, backend, session.config_signature)

    original_frame = scene.frame_current
    try:
        scene.frame_set(start)
        start_positions = session.positions()
        cache.store(start, start_positions)
        apply_positions_to_object(obj, start_positions)
        if progress_callback is not None:
            progress_callback(start, start, end)
        for frame in range(start + 1, end + 1):
            positions = session.step(settings.substeps)
            cache.store(frame, positions)
            if progress_callback is not None:
                progress_callback(frame, start, end)
    finally:
        scene.frame_set(original_frame)

    _FRAME_CACHES[scene.as_pointer()] = cache
    _ensure_frame_handler()
    return cache


def clear_frame_cache(scene: bpy.types.Scene) -> None:
    if _FRAME_CACHES.pop(scene.as_pointer(), None) is not None:
        _LOGGER.info("Cleared baked cache for scene %s", scene.name)
    _prune_frame_handler()


def frame_change_handler(scene: bpy.types.Scene, depsgraph) -> None:  # pragma: no cover - Blender callback
    cache = _FRAME_CACHES.get(scene.as_pointer())
    if cache is None:
        return
    obj = bpy.data.objects.get(cache.object_name)
    if obj is None:
        return
    positions = cache.sample(scene.frame_current)
    if positions is None:
        return
    try:
        apply_positions_to_object(obj, positions)
    except Exception as exc:  # pragma: no cover - protective logging
        _LOGGER.error("Failed to apply baked frame %s: %s", scene.frame_current, exc)


def _ensure_frame_handler() -> None:
    global _FRAME_HANDLER_REGISTERED
    if _FRAME_HANDLER_REGISTERED:
        return
    bpy.app.handlers.frame_change_post.append(frame_change_handler)
    _FRAME_HANDLER_REGISTERED = True


def _prune_frame_handler() -> None:
    global _FRAME_HANDLER_REGISTERED
    if _FRAME_HANDLER_REGISTERED and not _FRAME_CACHES:
        try:
            bpy.app.handlers.frame_change_post.remove(frame_change_handler)
        except ValueError:
            pass
        _FRAME_HANDLER_REGISTERED = False
