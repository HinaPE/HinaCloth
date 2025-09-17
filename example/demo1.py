"""Demo script that builds a bake-ready HinaCloth test scene inside Blender."""
from __future__ import annotations

import argparse
import importlib
import math
import pathlib
import sys
from typing import Optional

import bpy

# Ensure Blender can locate the local extension package when run from the repository.
_EXTENSION_ROOT = pathlib.Path(__file__).resolve().parent.parent / "blender"
if _EXTENSION_ROOT.exists():
    root_str = str(_EXTENSION_ROOT)
    if root_str not in sys.path:
        sys.path.append(root_str)


def ensure_hinacloth_registered() -> None:
    scene = bpy.context.scene
    if hasattr(scene, "hinacloth_settings"):
        return
    module = importlib.import_module("hinacloth")
    module.register()
    if not hasattr(scene, "hinacloth_settings"):
        raise RuntimeError("Failed to register HinaCloth extension inside Blender.")


def reset_scene() -> None:
    bpy.ops.wm.read_homefile(use_empty=True)
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)


def setup_world() -> None:
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"

    if scene.world is None:
        scene.world = bpy.data.worlds.new("HinaClothWorld")

    world = scene.world
    world.use_nodes = True
    tree = world.node_tree
    if tree:
        bg = tree.nodes.get("Background")
        if bg:
            bg.inputs[1].default_value = 0.75

    bpy.ops.object.light_add(type="SUN", location=(3.0, -6.0, 12.0))
    sun = bpy.context.active_object
    sun.rotation_euler = (
        math.radians(50.0),
        math.radians(20.0),
        math.radians(25.0),
    )
    sun.data.energy = 4.5


def configure_material(mat: bpy.types.Material, base_color, roughness: float) -> None:
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = (*base_color, 1.0)
        bsdf.inputs["Roughness"].default_value = roughness


def create_ground() -> bpy.types.Object:
    bpy.ops.mesh.primitive_plane_add(
        size=6.0,
        enter_editmode=False,
        location=(0.0, 0.0, 0.0),
    )
    ground = bpy.context.active_object
    ground.name = "Ground"

    mat = bpy.data.materials.get("GroundMaterial")
    if mat is None:
        mat = bpy.data.materials.new(name="GroundMaterial")
    configure_material(mat, (0.2, 0.25, 0.3), 0.1)
    ground.data.materials.clear()
    ground.data.materials.append(mat)
    return ground


def create_cloth(width: int = 40, height: int = 40) -> bpy.types.Object:
    if width < 2 or height < 2:
        raise ValueError("width/height must be >= 2 for a valid grid")
    bpy.ops.mesh.primitive_grid_add(
        x_subdivisions=width - 1,
        y_subdivisions=height - 1,
        size=2.0,
    )
    cloth = bpy.context.active_object
    cloth.name = "HinaClothGrid"
    cloth.location = (0.0, 0.0, 1.5)

    mat = bpy.data.materials.get("ClothMaterial")
    if mat is None:
        mat = bpy.data.materials.new(name="ClothMaterial")
    configure_material(mat, (0.8, 0.1, 0.4), 0.35)
    cloth.data.materials.clear()
    cloth.data.materials.append(mat)
    return cloth


def configure_hinacloth(cloth_object: bpy.types.Object, width: int, height: int, *, start: int, end: int) -> None:
    ensure_hinacloth_registered()
    scene = bpy.context.scene

    if len(cloth_object.data.vertices) != width * height:
        raise RuntimeError(
            f"Mesh vertex count {len(cloth_object.data.vertices)} does not match requested grid {width}x{height}."
        )

    scene.frame_start = start
    scene.frame_end = end

    settings = scene.hinacloth_settings
    settings.target_object = cloth_object
    settings.grid_width = width
    settings.grid_height = height
    settings.spacing = 0.025
    settings.substeps = 4
    settings.solver_iterations = 12
    settings.backend = "NATIVE"
    settings.start_frame = start
    settings.end_frame = end
    settings.status_message = "Scene initialized; ready to bake."

    state = scene.hinacloth_state
    state.bake_cache_active = False
    state.active_backend = settings.backend
    state.active_object = cloth_object.name


def run_bake_if_available(clear_existing: bool = True) -> bool:
    try:
        result = bpy.ops.hinacloth.bake_simulation(
            "EXEC_DEFAULT",
            clear_existing=clear_existing,
        )
    except Exception as exc:  # pragma: no cover - Blender operator guard
        print(f"[HinaCloth demo] Bake failed to start: {exc}")
        return False

    finished = "FINISHED" in result
    message = bpy.context.scene.hinacloth_settings.status_message
    print(f"[HinaCloth demo] Bake result: {result} — {message}")
    return finished


def build_demo(
    scene_path: Optional[pathlib.Path] = None,
    *,
    grid_width: int = 40,
    grid_height: int = 40,
    frame_start: int = 1,
    frame_end: int = 150,
    bake: bool = False,
) -> None:
    reset_scene()
    setup_world()
    create_ground()
    cloth = create_cloth(grid_width, grid_height)
    configure_hinacloth(
        cloth,
        grid_width,
        grid_height,
        start=frame_start,
        end=frame_end,
    )

    if bake:
        run_bake_if_available(clear_existing=True)

    if scene_path is not None:
        bpy.ops.wm.save_mainfile(filepath=str(scene_path))


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the HinaCloth demo scene.")
    parser.add_argument("--width", type=int, default=40, help="Cloth grid width (particles).")
    parser.add_argument("--height", type=int, default=40, help="Cloth grid height (particles).")
    parser.add_argument("--start", type=int, default=1, help="Timeline start frame for the bake window.")
    parser.add_argument("--end", type=int, default=150, help="Timeline end frame for the bake window.")
    parser.add_argument("--no-bake", action="store_true", help="Skip triggering the bake operator after setup.")
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=pathlib.Path(__file__).resolve().parent / "demo1.blend",
        help="Path to save the generated .blend file.",
    )
    return parser.parse_args(argv)


if __name__ == "__main__":
    namespace = _parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])
    build_demo(
        namespace.output,
        grid_width=namespace.width,
        grid_height=namespace.height,
        frame_start=namespace.start,
        frame_end=namespace.end,
        bake=not namespace.no_bake,
    )
    print(f"HinaCloth demo scene saved to {namespace.output}")
