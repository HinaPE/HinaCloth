"""Blender entrypoint for the HinaCloth extension."""
from __future__ import annotations

import logging
from typing import Final

import bpy

from . import ops, props, solver, ui
from .solver.manager import release_all

bl_info: Final = {
    "name": "HinaCloth Extension",
    "author": "HinaCloth Team",
    "version": (1, 0, 0),
    "blender": (4, 5, 3),
    "location": "View3D > Sidebar > HinaCloth",
    "description": "XPBD cloth benchmarking utilities integrated into Blender.",
    "category": "Physics",
}

_LOGGER = logging.getLogger("hinacloth")
_HANDLERS_INSTALLED = False


def _configure_logging() -> None:
    if _LOGGER.handlers:
        return
    handler = logging.StreamHandler()
    formatter = logging.Formatter("[%(name)s] %(levelname)s: %(message)s")
    handler.setFormatter(formatter)
    _LOGGER.addHandler(handler)
    _LOGGER.setLevel(logging.INFO)


def _on_file_loaded(_dummy):  # pragma: no cover - Blender handler
    release_all()


def _install_handlers() -> None:
    global _HANDLERS_INSTALLED
    if _HANDLERS_INSTALLED:
        return
    bpy.app.handlers.load_post.append(_on_file_loaded)
    _HANDLERS_INSTALLED = True


def _remove_handlers() -> None:
    global _HANDLERS_INSTALLED
    if not _HANDLERS_INSTALLED:
        return
    try:
        bpy.app.handlers.load_post.remove(_on_file_loaded)
    except ValueError:
        pass
    _HANDLERS_INSTALLED = False


def register() -> None:
    _configure_logging()
    props.register_properties()
    ops.register()
    ui.register()
    _install_handlers()
    _LOGGER.info("HinaCloth extension registered.")


def unregister() -> None:
    _remove_handlers()
    ui.unregister()
    ops.unregister()
    props.unregister_properties()
    release_all()
    _LOGGER.info("HinaCloth extension unregistered.")
