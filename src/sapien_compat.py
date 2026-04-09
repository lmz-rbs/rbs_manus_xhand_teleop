"""
SAPIEN 2 → 3 compatibility shim.

SAPIEN 3 removed/changed several APIs that GeoRT depends on.
This module patches `sapien.core` so that GeoRT's SAPIEN 2 code
runs unchanged on SAPIEN 3.

Usage: import this module BEFORE importing geort.
    import sapien_compat  # patches sapien.core in-place
    from geort.env.hand import HandKinematicModel  # now works on SAPIEN 3
"""

import sapien
import sys
import types
import warnings

_SAPIEN_MAJOR = int(sapien.__version__.split(".")[0])

if _SAPIEN_MAJOR >= 3:
    # ----------------------------------------------------------------
    # 1.  sapien.core  – SAPIEN 3 moved everything to top-level sapien.*
    # ----------------------------------------------------------------
    if not hasattr(sapien, "core"):
        # Create a shim module that proxies attribute access to sapien itself
        core = types.ModuleType("sapien.core")
        core.__package__ = "sapien"

        # Proxy all attribute lookups to the sapien package
        class _CoreProxy(types.ModuleType):
            def __getattr__(self, name):
                return getattr(sapien, name)
        core = _CoreProxy("sapien.core")
        sapien.core = core
        sys.modules["sapien.core"] = core
        # Also alias sapien.core.pysapien
        if hasattr(sapien, "pysapien"):
            core.pysapien = sapien.pysapien

    # ----------------------------------------------------------------
    # 2.  sapien.Engine  – deprecated in SAPIEN 3
    # ----------------------------------------------------------------
    if not hasattr(sapien, "Engine") or sapien.Engine is None:
        class _Engine:
            def __init__(self):
                pass
            def set_renderer(self, renderer):
                pass  # no-op in SAPIEN 3
            def create_scene(self, config=None):
                return sapien.Scene()
        sapien.Engine = _Engine
        sapien.core.Engine = _Engine

    # ----------------------------------------------------------------
    # 3.  sapien.SceneConfig  – replaced by physx.PhysxSceneConfig
    # ----------------------------------------------------------------
    _OrigSceneConfig = getattr(sapien, "SceneConfig", None)

    class _SceneConfigCompat:
        """Drop-in replacement that silently ignores SAPIEN-2 only fields."""
        def __init__(self):
            self._inner = None
            # Try to get the real SAPIEN 3 config
            try:
                self._inner = sapien.physx.PhysxSceneConfig()
            except Exception:
                pass

        def __setattr__(self, name, value):
            if name.startswith("_"):
                super().__setattr__(name, value)
                return
            # These SAPIEN 2 attributes don't exist in SAPIEN 3
            sapien2_only = {
                "default_dynamic_friction", "default_static_friction",
                "default_restitution", "contact_offset", "enable_pcm",
                "solver_iterations", "solver_velocity_iterations",
            }
            if name in sapien2_only:
                # Try to set on inner config, silently skip if not available
                if self._inner is not None:
                    try:
                        setattr(self._inner, name, value)
                    except AttributeError:
                        pass
                return
            if self._inner is not None:
                try:
                    setattr(self._inner, name, value)
                    return
                except AttributeError:
                    pass
            super().__setattr__(name, value)

        def __getattr__(self, name):
            if name.startswith("_"):
                raise AttributeError(name)
            if self._inner is not None:
                return getattr(self._inner, name)
            raise AttributeError(f"SceneConfigCompat has no attribute '{name}'")

    sapien.SceneConfig = _SceneConfigCompat
    sapien.core.SceneConfig = _SceneConfigCompat

    # ----------------------------------------------------------------
    # 4.  sapien.VulkanRenderer  – removed in SAPIEN 3
    # ----------------------------------------------------------------
    if not hasattr(sapien, "VulkanRenderer"):
        class _VulkanRenderer:
            """Stub that signals SAPIEN 3 Viewer to use its default renderer."""
            def __init__(self, *args, **kwargs):
                pass
            _is_stub = True
        sapien.VulkanRenderer = _VulkanRenderer
        sapien.core.VulkanRenderer = _VulkanRenderer

    # ----------------------------------------------------------------
    # 5.  sapien.Pose  – ensure it's available via sapien.core
    # ----------------------------------------------------------------
    if hasattr(sapien, "Pose"):
        sapien.core.Pose = sapien.Pose

    # ----------------------------------------------------------------
    # 6.  Engine.create_scene  – patch to accept SceneConfigCompat
    # ----------------------------------------------------------------
    _OrigEngine = sapien.Engine
    class _PatchedEngine(_OrigEngine):
        def create_scene(self, config=None):
            scene = sapien.Scene()
            # Apply physics config if possible
            if config is not None and hasattr(config, "_inner") and config._inner is not None:
                try:
                    scene.physx_system.set_scene_config(config._inner)
                except Exception:
                    pass
            return scene
    sapien.Engine = _PatchedEngine
    sapien.core.Engine = _PatchedEngine

    # ----------------------------------------------------------------
    # 7.  Scene methods compatibility
    # ----------------------------------------------------------------
    _OrigScene = sapien.Scene
    _orig_scene_init = _OrigScene.__init__

    # Patch Scene to add missing SAPIEN 2 methods
    def _scene_set_ambient_light(self, color):
        """SAPIEN 2 compat."""
        try:
            self.ambient_light = color
        except Exception:
            pass

    def _scene_add_directional_light(self, direction, color, shadow=False):
        """SAPIEN 2 compat."""
        try:
            self.add_directional_light(direction, color, shadow=shadow)
        except Exception:
            try:
                entity = sapien.DirectionalLightEntity()
                entity.color = color
                entity.shadow = shadow
                self.add_entity(entity)
            except Exception:
                pass

    def _scene_add_ground(self, altitude=0):
        """SAPIEN 2 compat."""
        try:
            self.add_ground(altitude)
        except Exception:
            pass

    # Only patch if methods are missing
    if not hasattr(_OrigScene, "set_ambient_light"):
        _OrigScene.set_ambient_light = _scene_set_ambient_light
    if not hasattr(_OrigScene, "add_ground"):
        _OrigScene.add_ground = _scene_add_ground

    # ----------------------------------------------------------------
    # 8.  Viewer compatibility
    # ----------------------------------------------------------------
    try:
        from sapien.utils import Viewer as _OrigViewer
    except ImportError:
        try:
            from sapien import Viewer as _OrigViewer
        except ImportError:
            _OrigViewer = None

    if _OrigViewer is not None:
        _orig_viewer_init = _OrigViewer.__init__
        def _patched_viewer_init(self, renderer=None, *args, **kwargs):
            # If renderer is our stub, pass None so SAPIEN 3 uses its default
            if renderer is not None and getattr(renderer, '_is_stub', False):
                renderer = None
            _orig_viewer_init(self, renderer=renderer, *args, **kwargs)
        _OrigViewer.__init__ = _patched_viewer_init

    print(f"[sapien_compat] Patched SAPIEN {sapien.__version__} for SAPIEN 2 API compatibility")

else:
    print(f"[sapien_compat] SAPIEN {sapien.__version__} (v2) - no patching needed")
