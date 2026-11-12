"""Damage-scissored specialization of ImGui Bundle 1.5.2's OpenGL backend.

The vertex upload, shaders and GL state handling follow the packaged backend;
only DrawCmd scissor intersections are added. Full draws use the parent method.

Original backend license:
Copyright (c) 2016, Michał Jaworski
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of pyimgui nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
import ctypes
import OpenGL.GL as gl
from imgui_bundle import imgui
from imgui_bundle.python_backends.glfw_backend import GlfwRenderer
from imgui_bundle.python_backends.opengl_backend import get_common_gl_state,restore_common_gl_state


class ScissoredRenderer(GlfwRenderer):
    def render(self, draw_data: imgui.ImDrawData, regions=None):
        if regions is None:return super().render(draw_data)
        # perf: local for faster access
        io = self.io

        display_width, display_height = io.display_size
        fb_width = int(display_width * io.display_framebuffer_scale[0])
        fb_height = int(display_height * io.display_framebuffer_scale[1])

        if fb_width == 0 or fb_height == 0:
            return

        draw_data.scale_clip_rects(io.display_framebuffer_scale)

        # backup GL state
        # todo: provide cleaner version of this backup-restore code
        common_gl_state_tuple = get_common_gl_state()
        last_program = gl.glGetIntegerv(gl.GL_CURRENT_PROGRAM)
        last_active_texture = gl.glGetIntegerv(gl.GL_ACTIVE_TEXTURE)
        last_array_buffer = gl.glGetIntegerv(gl.GL_ARRAY_BUFFER_BINDING)
        last_element_array_buffer = gl.glGetIntegerv(gl.GL_ELEMENT_ARRAY_BUFFER_BINDING)
        last_vertex_array = gl.glGetIntegerv(gl.GL_VERTEX_ARRAY_BINDING)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendEquation(gl.GL_FUNC_ADD)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glDisable(gl.GL_CULL_FACE)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_SCISSOR_TEST)
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)

        gl.glViewport(0, 0, int(fb_width), int(fb_height))

        ortho_projection = (ctypes.c_float * 16)(  # noqa
            2.0 / display_width,
            0.0,
            0.0,
            0.0,
            0.0,
            2.0 / -display_height,
            0.0,
            0.0,
            0.0,
            0.0,
            -1.0,
            0.0,
            -1.0,
            1.0,
            0.0,
            1.0,
        )

        gl.glUseProgram(self._shader_handle)
        gl.glUniform1i(self._attrib_location_tex, 0)
        gl.glUniformMatrix4fv(self._attrib_proj_mtx, 1, gl.GL_FALSE, ortho_projection)
        gl.glBindVertexArray(self._vao_handle)

        for commands in draw_data.cmd_lists:

            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._vbo_handle)
            # todo: check this (sizes)
            gl.glBufferData(
                gl.GL_ARRAY_BUFFER,
                commands.vtx_buffer.size() * imgui.VERTEX_SIZE,
                ctypes.c_void_p(commands.vtx_buffer.data_address()),
                gl.GL_STREAM_DRAW,
            )

            gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self._elements_handle)
            # todo: check this (sizes)
            gl.glBufferData(
                gl.GL_ELEMENT_ARRAY_BUFFER,
                commands.idx_buffer.size() * imgui.INDEX_SIZE,
                ctypes.c_void_p(commands.idx_buffer.data_address()),
                gl.GL_STREAM_DRAW,
            )

            # todo: allow to iterate over _CmdList
            for command in commands.cmd_buffer:
                gl.glBindTexture(gl.GL_TEXTURE_2D, command.texture_id)

                # todo: use named tuple
                x, y, z, w = command.clip_rect


                if imgui.INDEX_SIZE == 2:
                    gltype = gl.GL_UNSIGNED_SHORT
                else:
                    gltype = gl.GL_UNSIGNED_INT

                # Intersect with the original integer GL scissor, preserving
                # the packaged backend's rounding at non-unit DPI scales.
                cx,cy=int(x),int(fb_height-w)
                cr,ct=cx+int(z-x),cy+int(w-y)
                for dx1,dy1,dx2,dy2 in regions:
                    left,bottom=max(cx,dx1),max(cy,fb_height-dy2)
                    right,top=min(cr,dx2),min(ct,fb_height-dy1)
                    if right<=left or top<=bottom:continue
                    gl.glScissor(left,bottom,right-left,top-bottom)
                    gl.glDrawElements(gl.GL_TRIANGLES,command.elem_count,gltype,
                        ctypes.c_void_p(command.idx_offset*imgui.INDEX_SIZE))



        # restore modified GL state
        restore_common_gl_state(common_gl_state_tuple)

        gl.glUseProgram(last_program)
        gl.glActiveTexture(last_active_texture)
        gl.glBindVertexArray(last_vertex_array)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, last_array_buffer)
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, last_element_array_buffer)
