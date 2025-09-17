"""User interface panels for the HinaCloth extension."""
from __future__ import annotations

import bpy
from bpy.types import Panel

__all__ = ["HINACLOTH_PT_main", "HINACLOTH_PT_solver"]


class HINACLOTH_PT_main(Panel):
    bl_idname = "HINACLOTH_PT_main"
    bl_label = "HinaCloth"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "HinaCloth"

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return hasattr(context.scene, "hinacloth_settings")

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        settings = scene.hinacloth_settings
        state = scene.hinacloth_state

        col = layout.column(align=True)
        col.prop(settings, "target_object")
        col.prop(settings, "backend")
        col.prop(settings, "start_frame")
        col.prop(settings, "end_frame")
        col.separator()

        status_col = col.column(align=True)
        cache_icon = "RADIOBUT_ON" if state.bake_cache_active else "RADIOBUT_OFF"
        cache_status = "Active" if state.bake_cache_active else "Idle"
        status_col.label(text=f"Bake Cache: {cache_status}", icon=cache_icon)
        if state.active_object:
            status_col.label(text=f"Object: {state.active_object}")
        status_col.label(text=f"Backend: {state.active_backend}")
        status_col.separator()

        controls = col.column(align=True)
        controls.operator("hinacloth.bake_simulation", icon="REC")
        controls.operator("hinacloth.clear_bake", icon="TRASH")

        layout.separator()
        layout.label(text=f"Status: {settings.status_message}", icon="INFO")


class HINACLOTH_PT_solver(Panel):
    bl_idname = "HINACLOTH_PT_solver"
    bl_label = "Solver Parameters"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "HinaCloth"
    bl_parent_id = "HINACLOTH_PT_main"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        settings = context.scene.hinacloth_settings

        col = layout.column(align=True)
        col.prop(settings, "grid_width")
        col.prop(settings, "grid_height")
        col.prop(settings, "spacing")
        col.prop(settings, "time_step")
        col.prop(settings, "substeps")
        col.prop(settings, "solver_iterations")
        col.prop(settings, "velocity_damping")
        col.prop(settings, "gravity")
        col.prop(settings, "enable_distance_constraints")
