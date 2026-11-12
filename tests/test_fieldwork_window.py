"""Window GL backend follows the real GLFW context; workers keep their env."""
import importlib
import os
from types import SimpleNamespace
import pytest
from casbot_calib.fieldwork_window import bind_window_opengl


def fake_glfw(native=4,api=1):
    return SimpleNamespace(CONTEXT_CREATION_API=0,EGL_CONTEXT_API=2,OSMESA_CONTEXT_API=3,
        PLATFORM_X11=4,PLATFORM_WAYLAND=5,get_platform=lambda:native,get_window_attrib=lambda *args:api)


@pytest.mark.parametrize('native,api,expected',[(4,1,'glx'),(5,1,'egl'),(4,2,'egl'),(4,3,'osmesa')])
@pytest.mark.parametrize('override',[None,'egl'])
def test_backend_selection_and_environment_restoration(monkeypatch,native,api,expected,override):
    monkeypatch.setattr('sys.platform','linux')
    monkeypatch.setenv('DISPLAY',':unit');monkeypatch.setenv('WAYLAND_DISPLAY','wayland-unit')
    monkeypatch.setenv('MUJOCO_GL','egl')
    if override is None:monkeypatch.delenv('PYOPENGL_PLATFORM',raising=False)
    else:monkeypatch.setenv('PYOPENGL_PLATFORM',override)
    selected=[];module=SimpleNamespace(PLATFORM=SimpleNamespace(GetCurrentContext=lambda:123))
    def load(name):
        assert name=='OpenGL.platform';selected.append(os.environ['PYOPENGL_PLATFORM']);return module
    monkeypatch.setattr(importlib,'import_module',load)
    assert bind_window_opengl(fake_glfw(native,api),object()) is module
    assert selected==[expected]
    assert os.environ.get('PYOPENGL_PLATFORM')==override
    assert os.environ['MUJOCO_GL']=='egl' and os.environ['WAYLAND_DISPLAY']=='wayland-unit'


def test_preloaded_wrong_platform_reports_actionable_error(monkeypatch):
    monkeypatch.setattr('sys.platform','linux');monkeypatch.delenv('PYOPENGL_PLATFORM',raising=False)
    class EGLPlatform:
        def GetCurrentContext(self):return None
    monkeypatch.setattr(importlib,'import_module',lambda _:SimpleNamespace(PLATFORM=EGLPlatform()))
    with pytest.raises(RuntimeError,match='GLFW需要glx.*EGLPlatform.*新Python进程'):
        bind_window_opengl(fake_glfw(),object())
    assert 'PYOPENGL_PLATFORM' not in os.environ


def test_import_failure_restores_environment(monkeypatch):
    monkeypatch.setattr('sys.platform','linux');monkeypatch.setenv('PYOPENGL_PLATFORM','original')
    def fail(_):raise ImportError('broken driver binding')
    monkeypatch.setattr(importlib,'import_module',fail)
    with pytest.raises(ImportError,match='broken driver binding'):
        bind_window_opengl(fake_glfw(),object())
    assert os.environ['PYOPENGL_PLATFORM']=='original'
