"""GLFW/ImGui desktop loop: supervise at 60Hz, draw only at the display budget.

Uses ImGui Bundle's packaged OpenGL/GLFW backend. Physics and native streaming
remain in their existing child processes. No background heartbeat can conceal
an unresponsive frontend thread.
"""
import time

import numpy as np


class DisplaySchedule:
    def __init__(self):
        self.active_until=0.;self.next_draw=0.

    def interact(self):self.active_until=time.monotonic()+.5;self.next_draw=0.

    def due(self,now,*,visible=True):return visible and now>=self.next_draw

    def drawn(self,now,*,moving=False,short=False):
        hz=30. if short or now<self.active_until else 10. if moving else 2.
        self.next_draw=now+1./hz



def bind_window_opengl(glfw,window):
    """Bind PyOpenGL to the context GLFW actually made, not desktop env guesses.

    A Wayland desktop can still use the X11 GLFW wheel through XWayland. Limit
    the override to the platform import: EGL rendering/video workers must keep
    their own backend, not inherit this window's GLX selection.
    """
    import importlib
    import os
    import sys
    backend=None
    if sys.platform.startswith('linux'):
        api=glfw.get_window_attrib(window,glfw.CONTEXT_CREATION_API)
        if api==glfw.EGL_CONTEXT_API:backend='egl'
        elif api==glfw.OSMESA_CONTEXT_API:backend='osmesa'
        else:
            native=glfw.get_platform()
            if native==glfw.PLATFORM_X11:backend='glx'
            elif native==glfw.PLATFORM_WAYLAND:backend='egl'
            else:raise RuntimeError(f'GUI不支持当前GLFW窗口平台：{native}')
    previous=os.environ.get('PYOPENGL_PLATFORM')
    try:
        if backend:os.environ['PYOPENGL_PLATFORM']=backend
        platform=importlib.import_module('OpenGL.platform')
    finally:
        if previous is None:os.environ.pop('PYOPENGL_PLATFORM',None)
        else:os.environ['PYOPENGL_PLATFORM']=previous
    if not platform.PLATFORM.GetCurrentContext():
        loaded=type(platform.PLATFORM).__name__
        raise RuntimeError(f'GUI OpenGL上下文不匹配：GLFW需要{backend or "系统后端"}，PyOpenGL已加载{loaded}；请在新Python进程启动GUI')
    return platform


def run_window(args,root,monitor,mock):
    from pathlib import Path
    import glfw
    from imgui_bundle import imgui
    from .fieldwork_gui import Desk,_setup_gui
    from .fieldwork_display import latest_json
    if not glfw.init():raise RuntimeError('无法初始化桌面GLFW显示')
    window=None;backend=None;desk=None;context=None;canvas=None
    try:
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR,3);glfw.window_hint(glfw.CONTEXT_VERSION_MINOR,3)
        glfw.window_hint(glfw.OPENGL_PROFILE,glfw.OPENGL_CORE_PROFILE);glfw.window_hint(glfw.SAMPLES,0)
        window=glfw.create_window(1440,980,'CASBOT Fieldwork',None,None)
        if not window:raise RuntimeError('无法创建GUI窗口/OpenGL上下文')
        glfw.make_context_current(window);glfw.swap_interval(0)
        platform=bind_window_opengl(glfw,window)
        from .fieldwork_gl import ScissoredRenderer
        from OpenGL import GL
        context=imgui.create_context()
        imgui.get_io().set_ini_filename(str(Path(root)/'window.ini'))
        desk=Desk(root,preview=args.preview,profile=args.profile)
        desk.cache_monitor=monitor;desk.owned_mock=mock;desk.gui_managed_ticks=True
        class CompactFontRenderer(ScissoredRenderer):
            def refresh_font_texture(self):
                # Monochrome TTF atlas: white RGB with variable alpha. Store the
                # alpha only; swizzle restores identical RGBA for the shader.
                font=self.io.fonts.get_tex_data_as_rgba32()
                monochrome=bool(np.all(font[:,:,:3]==255))
                alpha=np.ascontiguousarray(font[:,:,3]) if monochrome else font
                self.font_storage='R8' if monochrome else 'RGBA8'
                self._font_texture=int(GL.glGenTextures(1))
                GL.glBindTexture(GL.GL_TEXTURE_2D,self._font_texture)
                GL.glTexParameteri(GL.GL_TEXTURE_2D,GL.GL_TEXTURE_MIN_FILTER,GL.GL_LINEAR)
                GL.glTexParameteri(GL.GL_TEXTURE_2D,GL.GL_TEXTURE_MAG_FILTER,GL.GL_LINEAR)
                if monochrome:
                    for component in (GL.GL_TEXTURE_SWIZZLE_R,GL.GL_TEXTURE_SWIZZLE_G,GL.GL_TEXTURE_SWIZZLE_B):
                        GL.glTexParameteri(GL.GL_TEXTURE_2D,component,GL.GL_ONE)
                    GL.glTexParameteri(GL.GL_TEXTURE_2D,GL.GL_TEXTURE_SWIZZLE_A,GL.GL_RED)
                GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT,1)
                GL.glTexImage2D(GL.GL_TEXTURE_2D,0,GL.GL_R8 if monochrome else GL.GL_RGBA8,alpha.shape[1],alpha.shape[0],0,GL.GL_RED if monochrome else GL.GL_RGBA,GL.GL_UNSIGNED_BYTE,alpha)
                self.io.fonts.tex_id=self._font_texture;GL.glBindTexture(GL.GL_TEXTURE_2D,0);self.io.fonts.clear_tex_data()
        _setup_gui(desk);backend=CompactFontRenderer(window)
        from .fieldwork_damage import CachedUI
        canvas=CachedUI();schedule=DisplaySchedule()
        # Chain the packaged backend's input handlers. Native input wakes drawing
        # immediately; numeric feedback alone never forces full-window 60Hz raster.
        def hook(setter):
            previous=None
            def changed(*values):
                schedule.interact()
                if previous:previous(*values)
            previous=setter(window,changed)
        for setter in (glfw.set_cursor_pos_callback,glfw.set_mouse_button_callback,glfw.set_scroll_callback,
                       glfw.set_key_callback,glfw.set_char_callback,glfw.set_window_size_callback,
                       glfw.set_window_iconify_callback):hook(setter)
        next_tick=0.;next_info=0.;last_draw=0.;polls=0;drawn_q=None;drawn_status=None
        renderer=GL.glGetString(GL.GL_RENDERER).decode()
        while not glfw.window_should_close(window):
            glfw.poll_events();now=time.monotonic()
            if now>=next_tick:
                desk.view_visible=not glfw.get_window_attrib(window,glfw.ICONIFIED) and bool(glfw.get_window_attrib(window,glfw.VISIBLE))
                desk.poll();desk.pulse();polls+=1;next_tick=now+1/60
            moving=desk.pending_start or desk.save_job is not None or (desk.q is not None and
                (drawn_q is None or np.max(np.abs(desk.q-drawn_q))>1e-4))
            status=(desk.message,desk.recording_error,desk.render_error,desk.save_message)
            if status!=drawn_status or (moving and schedule.next_draw>now+.1):schedule.next_draw=now
            if desk.short is not None and schedule.next_draw>now+1/30:schedule.next_draw=now
            if schedule.due(now,visible=desk.view_visible) and min(glfw.get_framebuffer_size(window))>0:
                backend.process_inputs();imgui.new_frame()
                width,height=imgui.get_io().display_size
                imgui.set_next_window_pos(imgui.ImVec2(0,0));imgui.set_next_window_size(imgui.ImVec2(width,height))
                flags=imgui.WindowFlags_.no_title_bar|imgui.WindowFlags_.no_move|imgui.WindowFlags_.no_resize|imgui.WindowFlags_.no_saved_settings
                imgui.begin('##manual_tuning',flags=flags);desk.gui();imgui.end();imgui.render()
                GL.glViewport(0,0,*glfw.get_framebuffer_size(window))
                bounds=desk.control_bounds.get('model_image')
                if bounds:
                    factor=tuple(imgui.get_io().display_framebuffer_scale)
                    bounds=tuple(v*factor[i%2] for i,v in enumerate(bounds))
                canvas.render(imgui.get_draw_data(),backend,glfw.get_framebuffer_size(window),
                    texture=(desk.texture,desk.texture_stamp,bounds) if bounds else None,
                    verify=getattr(args,'verify_damage',False))
                glfw.swap_buffers(window)
                last_draw=now;drawn_q=None if desk.q is None else desk.q.copy();drawn_status=status
                schedule.drawn(now,moving=moving,short=desk.short is not None)
                if desk.reopen_root or (args.frames and desk.frame>=args.frames):break
            if now>=next_info:
                latest_json(Path(root)/'view/display.json',dict(renderer=renderer,visible=desk.view_visible,
                    opengl_backend=type(platform.PLATFORM).__name__,glfw_context_api=glfw.get_window_attrib(window,glfw.CONTEXT_CREATION_API),
                    frontend_pid=__import__('os').getpid(),polls=polls,frames=desk.frame,last_draw_monotonic=last_draw,
                    font_storage=backend.font_storage,drawn_pixels=canvas.drawn_pixels,presented_frames=canvas.presented_frames,verified_frames=canvas.verified_frames,frame_transport='owned_memfd_double_buffer',display_shape=[360,480,3]))
                next_info=now+1.
            time.sleep(max(0.,min(.01,next_tick-time.monotonic())))
        if args.screenshot:
            from PIL import Image
            w,h=glfw.get_framebuffer_size(window)
            GL.glReadBuffer(GL.GL_FRONT)
            pixels=np.frombuffer(GL.glReadPixels(0,0,w,h,GL.GL_RGB,GL.GL_UNSIGNED_BYTE),np.uint8).reshape(h,w,3)
            Image.fromarray(pixels[::-1]).save(args.screenshot)
        return desk.reopen_root
    finally:
        if desk:desk.close()
        if canvas:canvas.close()
        if backend:backend.shutdown()
        if context:imgui.destroy_context(context)
        if window:glfw.destroy_window(window)
        glfw.terminate()
