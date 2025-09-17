"""Property registration helpers."""
from __future__ import annotations

from .settings import (
    HinaClothSettings,
    HinaClothState,
    register_properties,
    unregister_properties,
)

__all__ = [
    "HinaClothSettings",
    "HinaClothState",
    "register_properties",
    "unregister_properties",
    "register",
    "unregister",
]



def register() -> None:
    register_properties()



def unregister() -> None:
    unregister_properties()
