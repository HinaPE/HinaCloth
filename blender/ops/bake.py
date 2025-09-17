"""Bake operators for timeline simulation."""
from __future__ import annotations

import bpy
from bpy.props import BoolProperty
from bpy.types import Operator

from ..props.settings import HinaClothSettings
from ..solver.manager import SolverUnavailableError, bake_simulation, clear_frame_cache

__all__ = ["HINACLOTH_OT_bake_simulation", "HINACLOTH_OT_clear_bake"]


class HINACLOTH_OT_bake_simulation(Operator):
    """Bake XPBD frames into the scene cache."""

    bl_idname = "hinacloth.bake_simulation"
    bl_label = "Bake Simulation"
    bl_description = "Simulate the cloth over the timeline and cache frames for playback."
    bl_options = {"REGISTER"}

    clear_existing: BoolProperty(
        name="Clear Existing",
        description="Clear any baked cache before running a new bake.",
        default=True,
    )

    def execute(self, context: bpy.types.Context):
        scene = context.scene
        settings: HinaClothSettings = scene.hinacloth_settings
        state = scene.hinacloth_state
        obj = settings.target_object

        if obj is None:
            self.report({"ERROR"}, "Assign a cloth object before baking.")
            return {"CANCELLED"}
        if obj.type != "MESH":
            self.report({"ERROR"}, "Target object must be a mesh.")
            return {"CANCELLED"}
        if obj.mode != "OBJECT":
            self.report({"ERROR"}, "Switch the object back to Object Mode before baking.")
            return {"CANCELLED"}

        if self.clear_existing:
            clear_frame_cache(scene)
            state.bake_cache_active = False

        start, end = settings.frame_range(scene)
        total_frames = max(1, end - start + 1)

        wm = context.window_manager
        wm.progress_begin(0, total_frames)

        def _progress(frame: int, first: int, last: int) -> None:
            wm.progress_update(frame - first)

        try:
            cache = bake_simulation(
                scene,
                settings,
                settings.backend,
                obj,
                progress_callback=_progress,
            )
        except SolverUnavailableError as exc:
            wm.progress_end()
            settings.status_message = str(exc)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        except Exception as exc:  # pragma: no cover - runtime safeguard
            wm.progress_end()
            message = f"Bake failed: {exc}"
            settings.status_message = message
            self.report({"ERROR"}, message)
            return {"CANCELLED"}

        wm.progress_end()

        state.bake_cache_active = True
        state.active_backend = settings.backend
        state.active_object = obj.name
        settings.status_message = (
            f"Baked {len(cache.frames)} frames for {obj.name} using backend {settings.backend}."
        )
        scene.frame_set(start)
        return {"FINISHED"}


class HINACLOTH_OT_clear_bake(Operator):
    """Clear cached bake frames from the scene."""

    bl_idname = "hinacloth.clear_bake"
    bl_label = "Clear Cached Frames"
    bl_description = "Remove the baked cache for the active scene."
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context):
        scene = context.scene
        state = scene.hinacloth_state

        if not state.bake_cache_active:
            self.report({"INFO"}, "No bake cache registered for this scene.")
            return {"CANCELLED"}

        clear_frame_cache(scene)
        state.bake_cache_active = False
        state.active_object = ""
        settings: HinaClothSettings = scene.hinacloth_settings
        settings.status_message = "Cleared baked cache."
        return {"FINISHED"}
