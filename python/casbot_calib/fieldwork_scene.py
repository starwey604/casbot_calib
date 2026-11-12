"""Fixed-camera live CAD compositor: cache static RGB/depth, draw moving arms.

Shadows and reflections are disabled, so static geometry does not depend on arm
pose. Both layers use the same MuJoCo scene transforms, lighting and depth test.
Uses public MuJoCo rendering APIs; never advances physics.
"""
import numpy as np


class LiveSceneRenderer:
    def __init__(self,model,*,width,height):
        import mujoco as mj
        self.mj=mj;self.model=model;self.width=width;self.height=height
        self.gl=mj.GLContext(width,height);self.gl.make_current()
        self.context=mj.MjrContext(model,mj.mjtFontScale.mjFONTSCALE_150)
        mj.mjr_setBuffer(mj.mjtFramebuffer.mjFB_OFFSCREEN,self.context)
        self.context.readDepthMap=mj.mjtDepthMap.mjDEPTH_ZERONEAR
        self.scene=mj.MjvScene(model,maxgeom=max(1000,model.ngeom+model.nsite+100))
        self.option=mj.MjvOption();self.rect=mj.MjrRect(0,0,width,height)
        self.rgb=np.empty((height,width,3),dtype=np.uint8);self.depth=np.empty((height,width),dtype=np.float32)
        self.static=np.flatnonzero(model.body_weldid[model.geom_bodyid]==0)
        self.background=None;self.background_depth=None;self.key=None;self.data=None;self.camera=None
        self.static_passes=0;self.dynamic_passes=0

    def update_scene(self,data,camera):self.data=data;self.camera=camera

    def _layer(self,category):
        mj=self.mj
        mj.mjv_updateScene(self.model,self.data,self.option,None,self.camera,category,self.scene)
        mj.mjr_render(self.rect,self.scene,self.context)
        mj.mjr_readPixels(self.rgb,self.depth,self.rect,self.context)

    def render(self):
        mj=self.mj;self.gl.make_current();camera=self.camera;data=self.data
        key=(tuple(camera.lookat),camera.distance,camera.azimuth,camera.elevation,camera.type,camera.fixedcamid,
             data.geom_xpos[self.static].tobytes(),data.geom_xmat[self.static].tobytes(),
             self.model.geom_rgba[self.static].tobytes(),data.light_xpos.tobytes(),data.light_xdir.tobytes())
        if key!=self.key:
            self._layer(mj.mjtCatBit.mjCAT_STATIC)
            self.background=self.rgb.copy();self.background_depth=self.depth.copy();self.key=key;self.static_passes+=1
        self._layer(mj.mjtCatBit.mjCAT_DYNAMIC|mj.mjtCatBit.mjCAT_DECOR);self.dynamic_passes+=1
        result=self.background.copy();mask=self.depth<self.background_depth;result[mask]=self.rgb[mask]
        return result[::-1].copy()

    def close(self):
        if self.gl is not None:
            self.gl.make_current();self.context.free();self.gl.free();self.gl=None
