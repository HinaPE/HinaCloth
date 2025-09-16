"""Modal simulation operators."""
from __future__ import annotations

from typing import Optional

import bpy
from bpy.props import BoolProperty
from bpy.types import Operator

from ..props.settings import HinaClothSettings
from ..solver.manager import (
    SolverUnavailableError,
    acquire_session,
    apply_positions_to_object,
    clear_frame_cache,
)

__all__ = [
    "HINACLOTH_OT_start_modal",
    "HINACLOTH_OT_stop_modal",
]


def _redraw_viewports(context: bpy.types.Context) -> None:
    wm = context.window_manager
    for window in wm.windows:
        screen = window.screen
        for area in screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


class HINACLOTH_OT_start_modal(Operator):
    bl_idname = "hinacloth.start_modal"
    bl_label = "Start Modal Simulation"
    bl_description = "Run the XPBD solver in a modal operator for interactive playback."
    bl_options = {"REGISTER"}

    reset_state: BoolProperty(
        name="Reset Solver",
        description="Reset solver state before starting the modal run.",
        default=True,
    )

    _timer: Optional[bpy.types.Timer] = None
    _session = None
    _object_name: str = ""

    def execute(self, context: bpy.types.Context):
        scene = context.scene
        settings: HinaClothSettings = scene.hinacloth_settings
        state = scene.hinacloth_state

        if state.is_modal_running:
            self.report({"WARNING"}, "Modal simulation already running.")
            return {"CANCELLED"}

        obj = settings.target_object
        if obj is None:
            self.report({"ERROR"}, "Assign a cloth object before starting the solver.")
            return {"CANCELLED"}
        if obj.type != "MESH":
            self.report({"ERROR"}, "Target object must be a mesh.")
            return {"CANCELLED"}
        if obj.mode != "OBJECT":
            self.report({"ERROR"}, "Switch the object back to Object Mode before simulation.")
            return {"CANCELLED"}

        clear_frame_cache(scene)
        state.bake_cache_active = False

        try:
            session = acquire_session(
                scene,
                settings,
                settings.backend,
                obj,
                force_rebuild=self.reset_state,
            )
        except SolverUnavailableError as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        except Exception as exc:  # pragma: no cover - runtime safeguard
            self.report({"ERROR"}, f"Failed to initialize solver: {exc}")
            return {"CANCELLED"}

        if self.reset_state:
            session.reset()
            try:
                apply_positions_to_object(obj, session.positions())
            except Exception as exc:  # pragma: no cover - runtime safeguard
                self.report({"ERROR"}, f"Unable to apply initial state: {exc}")
                return {"CANCELLED"}

        wm = context.window_manager
        interval = max(settings.time_step * settings.substeps, 1.0 / 120.0)
        self._timer = wm.event_timer_add(interval, window=context.window)
        wm.modal_handler_add(self)

        self._session = session
        self._object_name = obj.name

        state.is_modal_running = True
        state.active_backend = settings.backend
        state.active_object = obj.name
        settings.status_message = f"Running modal simulation on {obj.name} ({settings.backend})."

        return {"RUNNING_MODAL"}

    def modal(self, context: bpy.types.Context, event: bpy.types.Event):
        scene = context.scene
        settings: HinaClothSettings = scene.hinacloth_settings
        state = scene.hinacloth_state

        if event.type == "ESC":
            state.is_modal_running = False
            settings.status_message = "Modal simulation cancelled."
            return self.cancel(context)

        if event.type != "TIMER":
            return {"RUNNING_MODAL"}

        if not state.is_modal_running:
            return self.cancel(context)

        obj = bpy.data.objects.get(self._object_name)
        if obj is None:
            settings.status_message = "Target object removed; stopping solver."
            state.is_modal_running = False
            return self.cancel(context)

        try:
            positions = self._session.step(settings.substeps)
        except Exception as exc:  # pragma: no cover - solver runtime safeguard
            settings.status_message = f"Solver error: {exc}"
            state.is_modal_running = False
            return self.cancel(context)

        try:
            apply_positions_to_object(obj, positions)
        except Exception as exc:  # pragma: no cover - mesh update safeguard
            settings.status_message = f"Mesh update failed: {exc}"
            state.is_modal_running = False
            return self.cancel(context)

        _redraw_viewports(context)
        return {"RUNNING_MODAL"}

    def cancel(self, context: bpy.types.Context):
        wm = context.window_manager
        if self._timer is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None

        scene = context.scene
        settings: HinaClothSettings = scene.hinacloth_settings
        state = scene.hinacloth_state
        state.is_modal_running = False
        settings.status_message = "Modal simulation stopped."

        self._session = None
        self._object_name = ""

        return {"CANCELLED"}


class HINACLOTH_OT_stop_modal(Operator):
    bl_idname = "hinacloth.stop_modal"
    bl_label = "Stop Modal Simulation"
    bl_description = "Stop the currently running modal XPBD simulation."
    bl_options = {"REGISTER", "INTERNAL"}

    def execute(self, context: bpy.types.Context):
        scene = context.scene
        state = scene.hinacloth_state
        settings: HinaClothSettings = scene.hinacloth_settings

        if not state.is_modal_running:
            self.report({"INFO"}, "Modal solver is not running.")
            return {"CANCELLED"}

        state.is_modal_running = False
        settings.status_message = "Stopping modal solver..."
        return {"FINISHED"}
