#!/usr/bin/env python3
"""Compare source triangle surfaces with conservative shoulder/waist hulls.

This is an offline geometry audit, not a replacement collision checker.
Source BVH distances are surface diagnostics, never a solid/tube certificate.
"""
import argparse,csv
from pathlib import Path
import numpy as np
import pinocchio as pin
import coal
from casbot_calib.dvt2_bundle import load_bundle,robot_profile,read
from casbot_calib.collision import CollisionProfile
from casbot_calib.trical_assembly import asset_directory
from casbot_calib.data import write_json


def audit(bundle,output):
    out=Path(output);out.mkdir(parents=True,exist_ok=False);bundle=Path(bundle).resolve()
    engine,meta=load_bundle(bundle);base=robot_profile(bundle)
    model=pin.buildModelFromUrdf(str(bundle/'model.urdf'))
    geom=pin.buildGeomFromUrdf(model,str(bundle/'model.urdf'),pin.GeometryType.COLLISION,package_dirs=[str(bundle)])
    data=model.createData();gdata=geom.createData();seed=read(asset_directory(bundle)/'seeds.json')['results']
    rows=[];sources=[];grid={};reference=[]
    pitch=np.arange(-140.,11.,5.);outward=np.arange(-30.,101.,5.)
    for side,ix in [('left',0),('right',7)]:
        ids=[geom.getGeometryId(name+'_0') for name in ['waist_yaw_link',side+'_shoulder_roll_link']]
        native=CollisionProfile(engine,[base.geometry[base.names.index('robot_'+geom.geometryObjects[i].name)] for i in ids],[],{})
        sol=coal.ComputeDistance(*[geom.geometryObjects[i].geometry for i in ids])
        source=[]
        for i in ids:
            g=geom.geometryObjects[i];source.append(dict(name=g.name,mesh=str(Path(g.meshPath).relative_to(bundle.parent.parent)),vertices=len(g.geometry.vertices()),triangles=g.geometry.num_tris,placement=g.placement.homogeneous.tolist()))
        sources.append(dict(side=side,geometries=source))
        def sample(q,label):
            pin.updateGeometryPlacements(model,data,geom,gdata,q)
            result=coal.DistanceResult();gap=sol(*[coal.Transform3s(gdata.oMg[i].rotation,gdata.oMg[i].translation) for i in ids],coal.DistanceRequest(True,True),result)
            hull=float(native.check(q)['distances'][0,0]);pa=np.array(result.getNearestPoint1());pb=np.array(result.getNearestPoint2())
            item=dict(side=side,label=label,pitch_deg=float(np.rad2deg(q[ix])),outward_roll_deg=float(np.rad2deg(q[ix+1])*(1 if side=='left' else -1)),hull_distance_mm=hull*1000,source_surface_distance_mm=float(gap)*1000,
                      hull_10mm_met=hull>=.01,source_surface_10mm_met=gap>=.01)
            rows.append(item);return item,pa,pb
        for r in seed:
            if r['arm']!=side:continue
            item,a,b=sample(np.array(r['contact'][0]['q']),r['ball']+'_contact')
            reference.append(dict(**item,q_rad=r['contact'][0]['q'],source_nearest_points_world_m=[a.tolist(),b.tolist()]))
        grid[side]=[]
        for p in pitch:
            line=[]
            for r in outward:
                q=np.array(meta['profile']['home_rad']);q[ix]=np.deg2rad(p);q[ix+1]=np.deg2rad(r)*(1 if side=='left' else -1)
                item,_,_=sample(q,'grid');line.append([item['hull_distance_mm'],item['source_surface_distance_mm']])
            grid[side].append(line)
    fields=list(rows[0]);
    with (out/'distances.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fields);w.writeheader();w.writerows(rows)
    result=dict(scope='source mesh surface audit only; native conservative collision policy unchanged',bundle_sha256=meta['bundle_sha256'],sources=sources,reference=reference,
                grid_pitch_deg=pitch.tolist(),grid_outward_roll_deg=outward.tolist(),grid_samples_per_side=len(pitch)*len(outward),ordinary_clearance_m=.01)
    result['separated_grid_comparison']={}
    for side,values in grid.items():
        z=np.array(values);mask=(z[:,:,0]>0)&(z[:,:,1]>0)
        result['separated_grid_comparison'][side]=dict(maximum_absolute_difference_mm=float(np.max(abs(z[:,:,0][mask]-z[:,:,1][mask]))),
            source_10mm_met_hull_not_met=int(np.count_nonzero((z[:,:,0]<10)&(z[:,:,1]>=10))))
    write_json(out/'audit.json',result)
    np.savez_compressed(out/'grid.npz',**{side:np.array(value) for side,value in grid.items()})
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from casbot_calib.presentation import _font
    font=FontProperties(fname=_font(18).path)
    fig,axes=plt.subplots(2,2,figsize=(13,10),layout='constrained')
    for i,side in enumerate(['left','right']):
        for j,title in enumerate(['保守凸包距离','源三角网格表面距离']):
            ax=axes[i,j];z=np.array(grid[side])[:,:,j]
            im=ax.pcolormesh(outward,pitch,z,vmin=-5,vmax=30,cmap='RdYlGn',shading='nearest')
            ax.contour(outward,pitch,z,levels=[10],colors='black',linewidths=1)
            for ref in reference:
                if ref['side']==side:
                    ax.plot(ref['outward_roll_deg'],ref['pitch_deg'],'kx');ax.annotate(ref['label'][0],(ref['outward_roll_deg']+1,ref['pitch_deg']+1))
            ax.set_title(('左臂 / ' if side=='left' else '右臂 / ')+title,fontproperties=font)
            ax.set_xlabel('向外肩滚转角 / °（右臂取负号）',fontproperties=font);ax.set_ylabel('肩俯仰角 / °',fontproperties=font)
    fig.colorbar(im,ax=axes,label='mm',shrink=.75)
    fig.suptitle('肩部—腰部：黑线为10 mm；叉号为原review测量姿态\n源网格用于核对表面，未代替实心包络或连续门禁',fontproperties=font,fontsize=18)
    fig.savefig(out/'shoulder_waist_audit_zh.png',dpi=140);plt.close(fig)
    print([(r['side'],r['label'],r['hull_distance_mm'],r['source_surface_distance_mm']) for r in reference])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',required=True);p.add_argument('--output',required=True);a=p.parse_args();audit(a.bundle,a.output)
