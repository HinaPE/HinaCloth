"""Operator registrations for the HinaCloth extension."""
from __future__ import annotations

import bpy

from .bake import HINACLOTH_OT_bake_simulation, HINACLOTH_OT_clear_bake
from .modal import HINACLOTH_OT_start_modal, HINACLOTH_OT_stop_modal

__all__ = [
    "register",
    "unregister",
    "HINACLOTH_OT_start_modal",
    "HINACLOTH_OT_stop_modal",
    "HINACLOTH_OT_bake_simulation",
    "HINACLOTH_OT_clear_bake",
]

_CLASSES = (
    HINACLOTH_OT_start_modal,
    HINACLOTH_OT_stop_modal,
    HINACLOTH_OT_bake_simulation,
    HINACLOTH_OT_clear_bake,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
