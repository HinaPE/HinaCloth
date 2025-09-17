"""Operator registrations for the HinaCloth extension."""
from __future__ import annotations

from bpy.utils import register_classes_factory

from .bake import HINACLOTH_OT_bake_simulation, HINACLOTH_OT_clear_bake

__all__ = [
    "register",
    "unregister",
    "HINACLOTH_OT_bake_simulation",
    "HINACLOTH_OT_clear_bake",
]

_CLASSES = (
    HINACLOTH_OT_bake_simulation,
    HINACLOTH_OT_clear_bake,
)

register, unregister = register_classes_factory(_CLASSES)
