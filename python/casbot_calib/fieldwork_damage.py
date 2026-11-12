"""Persistent UI framebuffer with triangle-derived tile damage.

ImGui still builds the complete current UI and its packaged renderer draws all
layers. Draw-command scissors limit rasterization to changed regions; a framebuffer
blit presents the complete image. Texture content changes are explicit damage.
"""
import ctypes
import math

import numpy as np


class Damage:
    tile=48
    def __init__(self):self.previous=None;self.shape=None;self.texture=None

    def regions(self,chunks,shape,*,texture=None):
        width,height=shape;grid=np.zeros((math.ceil(height/self.tile),math.ceil(width/self.tile)),dtype=bool)
        def mark(rect):
            x1,y1,x2,y2=rect
            left=max(0,int(math.floor(x1-1))//self.tile);right=min(grid.shape[1],int(math.ceil(x2+1)/self.tile)+1)
            top=max(0,int(math.floor(y1-1))//self.tile);bottom=min(grid.shape[0],int(math.ceil(y2+1)/self.tile)+1)
            if right>left and bottom>top:grid[top:bottom,left:right]=True
        full=self.previous is None or self.shape!=shape or len(chunks)!=len(self.previous)
        if not full:
            for (key,values),(old_key,old_values) in zip(chunks,self.previous):
                if key!=old_key:full=True;break
                if values.shape!=old_values.shape:mark(key[1:]);continue
                changed=np.any(values!=old_values,axis=1).reshape(-1,3).any(axis=1)
                if not np.any(changed):continue
                for array in (values,old_values):
                    triangles=np.ascontiguousarray(array[:,:8]).view(np.float32).reshape(-1,3,2)[changed]
                    for points in triangles:
                        lo=points.min(axis=0);hi=points.max(axis=0)
                        # DrawCmd clipping applies before the damage mask.
                        mark((max(lo[0],key[1]),max(lo[1],key[2]),min(hi[0],key[3]),min(hi[1],key[4])))
        if full:grid[:]=True
        if texture is not None and texture!=self.texture:mark(texture[2])
        self.previous=chunks;self.shape=shape;self.texture=texture
        # Coalesce horizontal tile runs, bounding GL clear calls independently of
        # the number of changing glyphs, while retaining small damage regions.
        regions=[]
        for y,row in enumerate(grid):
            x=0
            while x<len(row):
                if not row[x]:x+=1;continue
                begin=x
                while x<len(row) and row[x]:x+=1
                regions.append((begin*self.tile,y*self.tile,min(width,x*self.tile),min(height,(y+1)*self.tile)))
        merged=[];last={}
        for x1,y1,x2,y2 in regions:
            key=(x1,x2)
            if key in last and merged[last[key]][3]==y1:
                at=last[key];merged[at]=(x1,merged[at][1],x2,y2)
            else:last[key]=len(merged);merged.append((x1,y1,x2,y2))
        return merged


def chunks(draw_data,scale):
    from imgui_bundle import imgui
    result=[]
    for commands in draw_data.cmd_lists:
        vertices=np.ctypeslib.as_array((ctypes.c_ubyte*(commands.vtx_buffer.size()*imgui.VERTEX_SIZE)).from_address(commands.vtx_buffer.data_address())).reshape(-1,imgui.VERTEX_SIZE)
        indices=np.frombuffer((ctypes.c_ubyte*(commands.idx_buffer.size()*imgui.INDEX_SIZE)).from_address(commands.idx_buffer.data_address()),dtype=np.uint16 if imgui.INDEX_SIZE==2 else np.uint32)
        for command in commands.cmd_buffer:
            x1,y1,x2,y2=command.clip_rect
            expanded=vertices[indices[command.idx_offset:command.idx_offset+command.elem_count]+command.vtx_offset].copy()
            points=np.ascontiguousarray(expanded[:,:8]).view(np.float32).reshape(-1,2)
            points*=scale
            expanded[:,:8]=points.view(np.uint8).reshape(-1,8)
            result.append(((command.texture_id,x1*scale[0],y1*scale[1],x2*scale[0],y2*scale[1]),expanded))
    return result


class CachedUI:
    def __init__(self):
        self.damage=Damage();self.shape=None;self.fbo=None;self.color=None;self.stencil=None
        self.drawn_pixels=0;self.presented_frames=0;self.verified_frames=0

    def _allocate(self,shape):
        from OpenGL import GL
        self.close();width,height=shape
        self.fbo=int(GL.glGenFramebuffers(1));GL.glBindFramebuffer(GL.GL_FRAMEBUFFER,self.fbo)
        self.color=int(GL.glGenRenderbuffers(1));GL.glBindRenderbuffer(GL.GL_RENDERBUFFER,self.color)
        GL.glRenderbufferStorage(GL.GL_RENDERBUFFER,GL.GL_RGBA8,width,height)
        GL.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER,GL.GL_COLOR_ATTACHMENT0,GL.GL_RENDERBUFFER,self.color)
        if GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER)!=GL.GL_FRAMEBUFFER_COMPLETE:raise RuntimeError('GUI缓存帧缓冲不完整')
        self.shape=shape;self.damage=Damage()

    def render(self,draw_data,backend,shape,*,texture=None,verify=False):
        from imgui_bundle import imgui
        from OpenGL import GL
        width,height=shape
        if self.shape!=shape:self._allocate(shape)
        scale=tuple(imgui.get_io().display_framebuffer_scale)
        regions=self.damage.regions(chunks(draw_data,scale),shape,texture=texture)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER,self.fbo)
        GL.glDisable(GL.GL_SCISSOR_TEST)
        GL.glEnable(GL.GL_SCISSOR_TEST);GL.glClearColor(.94,.96,.98,1)
        for x1,y1,x2,y2 in regions:
            GL.glScissor(x1,height-y2,x2-x1,y2-y1);GL.glClear(GL.GL_COLOR_BUFFER_BIT)
            self.drawn_pixels+=(x2-x1)*(y2-y1)
        if regions:
            backend.render(draw_data,regions=regions)
        if verify:
            if scale!=(1.,1.):raise ValueError('Pixel comparison requires unit framebuffer scale')
            cached=np.frombuffer(GL.glReadPixels(0,0,width,height,GL.GL_RGB,GL.GL_UNSIGNED_BYTE),np.uint8).copy()
            GL.glDisable(GL.GL_SCISSOR_TEST);GL.glClear(GL.GL_COLOR_BUFFER_BIT);backend.render(draw_data)
            reference=np.frombuffer(GL.glReadPixels(0,0,width,height,GL.GL_RGB,GL.GL_UNSIGNED_BYTE),np.uint8)
            if not np.array_equal(cached,reference):raise RuntimeError('脏区域缓存与整窗绘制不一致：'+str(np.count_nonzero(cached!=reference)))
            self.verified_frames+=1
        GL.glDisable(GL.GL_SCISSOR_TEST)
        GL.glBindFramebuffer(GL.GL_READ_FRAMEBUFFER,self.fbo);GL.glBindFramebuffer(GL.GL_DRAW_FRAMEBUFFER,0)
        GL.glBlitFramebuffer(0,0,width,height,0,0,width,height,GL.GL_COLOR_BUFFER_BIT,GL.GL_NEAREST)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER,0);self.presented_frames+=1

    def close(self):
        from OpenGL import GL
        if self.fbo is not None:
            GL.glDeleteFramebuffers(1,[self.fbo]);GL.glDeleteRenderbuffers(1,[self.color]);self.fbo=None
        self.shape=None
