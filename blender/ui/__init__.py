"""UI registration for the HinaCloth extension."""
from __future__ import annotations

import bpy

from .panel import HINACLOTH_PT_main, HINACLOTH_PT_solver

__all__ = ["register", "unregister", "HINACLOTH_PT_main", "HINACLOTH_PT_solver"]

_CLASSES = (
    HINACLOTH_PT_main,
    HINACLOTH_PT_solver,
)


def register() -> None:
    for cls in _CLASSES:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_CLASSES):
        bpy.utils.unregister_class(cls)
