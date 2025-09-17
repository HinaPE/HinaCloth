"""UI registration for the HinaCloth extension."""
from __future__ import annotations

from bpy.utils import register_classes_factory

from .panel import HINACLOTH_PT_main, HINACLOTH_PT_solver

__all__ = ["register", "unregister", "HINACLOTH_PT_main", "HINACLOTH_PT_solver"]

_CLASSES = (
    HINACLOTH_PT_main,
    HINACLOTH_PT_solver,
)

register, unregister = register_classes_factory(_CLASSES)
