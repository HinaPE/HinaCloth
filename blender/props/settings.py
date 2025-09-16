"""Property definitions for the HinaCloth Blender extension."""
from __future__ import annotations

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Object, PropertyGroup

__all__ = [
    "HinaClothSettings",
    "HinaClothState",
    "register_properties",
    "unregister_properties",
]

_BACKEND_ITEMS = (
    ("NATIVE", "Native", "Run XPBD solver on the baseline backend."),
    ("TBB", "Intel TBB", "Use Intel TBB accelerated solver variant."),
    ("AVX2", "AVX2", "Use AVX2 optimized solver variant."),
)


def _cloth_object_poll(_self, obj: Object) -> bool:
    return obj.type == "MESH"


class HinaClothSettings(PropertyGroup):
    """User configurable parameters exposed by the add-on UI."""

    target_object: PointerProperty(
        name="Cloth Object",
        description="Mesh object driven by the HinaCloth solver.",
        type=Object,
        poll=_cloth_object_poll,
    )

    backend: EnumProperty(
        name="Backend",
        description="Choose which solver backend implementation to run.",
        items=_BACKEND_ITEMS,
        default="NATIVE",
    )

    start_frame: IntProperty(
        name="Start Frame",
        description="First frame of the bake window.",
        default=1,
    )

    end_frame: IntProperty(
        name="End Frame",
        description="Last frame of the bake window (inclusive).",
        default=240,
    )

    grid_width: IntProperty(
        name="Grid Width",
        description="Number of particles along the horizontal grid axis.",
        min=2,
        default=40,
    )

    grid_height: IntProperty(
        name="Grid Height",
        description="Number of particles along the vertical grid axis.",
        min=2,
        default=40,
    )

    spacing: FloatProperty(
        name="Spacing",
        description="Rest spacing between cloth particles in meters.",
        min=0.001,
        default=0.025,
    )

    time_step: FloatProperty(
        name="Time Step",
        description="XPBD integration time step in seconds.",
        min=1.0e-5,
        default=1.0 / 60.0,
    )

    substeps: IntProperty(
        name="Substeps",
        description="Number of solver substeps executed per frame/tick.",
        min=1,
        default=4,
    )

    solver_iterations: IntProperty(
        name="Iterations",
        description="Number of constraint solver iterations per substep.",
        min=1,
        default=10,
    )

    velocity_damping: FloatProperty(
        name="Velocity Damping",
        description="Velocity damping factor applied every step.",
        min=0.0,
        max=1.0,
        default=0.01,
    )

    gravity: FloatVectorProperty(
        name="Gravity",
        description="Gravity vector applied to the cloth particles.",
        size=3,
        default=(0.0, 0.0, -9.81),
        subtype="XYZ",
    )

    enable_distance_constraints: BoolProperty(
        name="Enable Distance",
        description="Enable XPBD stretch constraints.",
        default=True,
    )

    status_message: StringProperty(
        name="Status",
        description="Latest status reported by the solver or operators.",
        default="Ready.",
    )

    def frame_range(self, scene) -> tuple[int, int]:
        start = max(self.start_frame, scene.frame_start)
        end = max(start, max(self.end_frame, start))
        return start, end

    def as_solver_config(self) -> dict:
        return {
            "grid_width": int(self.grid_width),
            "grid_height": int(self.grid_height),
            "spacing": float(self.spacing),
            "time_step": float(self.time_step),
            "substeps": int(self.substeps),
            "solver_iterations": int(self.solver_iterations),
            "velocity_damping": float(self.velocity_damping),
            "gravity": tuple(float(x) for x in self.gravity),
            "enable_distance_constraints": bool(self.enable_distance_constraints),
        }

    def solver_signature(self) -> tuple:
        cfg = self.as_solver_config()
        return (
            cfg["grid_width"],
            cfg["grid_height"],
            cfg["spacing"],
            cfg["time_step"],
            cfg["substeps"],
            cfg["solver_iterations"],
            cfg["velocity_damping"],
            cfg["gravity"],
            cfg["enable_distance_constraints"],
        )


class HinaClothState(PropertyGroup):
    """Runtime state stored on the scene for UI feedback."""

    is_modal_running: BoolProperty(
        name="Modal Running",
        description="True while the modal operator keeps stepping the solver.",
        default=False,
    )

    bake_cache_active: BoolProperty(
        name="Bake Cache Active",
        description="Indicates whether a frame cache is currently registered.",
        default=False,
    )

    active_backend: EnumProperty(
        name="Active Backend",
        description="Backend currently driving the solver instance.",
        items=_BACKEND_ITEMS,
        default="NATIVE",
    )

    active_object: StringProperty(
        name="Active Object",
        description="Name of the object controlled by the solver session.",
        default="",
    )


_CLASSES = (HinaClothSettings, HinaClothState)


def register_properties() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)

    bpy.types.Scene.hinacloth_settings = PointerProperty(type=HinaClothSettings)
    bpy.types.Scene.hinacloth_state = PointerProperty(type=HinaClothState)


def unregister_properties() -> None:
    del bpy.types.Scene.hinacloth_settings
    del bpy.types.Scene.hinacloth_state
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
