#!/usr/bin/env python3
"""Summarise actual hole/height experiments; nominal screens are not certificates."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from casbot_calib.presentation import _font
from casbot_calib.dvt2_bundle import read
from casbot_calib.data import write_json


def build(output,column,side,height,pose,platform_pose,certificates=()):
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    groups={name:read(Path(root)/'screening.json') for name,root in [('column',column),('side',side),('height',height),('pose',pose),('platform_pose',platform_pose)]}
    fields=['family','layout','nominal_combinations','column_dx_mm','column_dy_mm','C_dz_mm','D_dz_mm','platform_dz_mm']
    series=[]
    for family,rows in groups.items():
        for r in rows:
            layout=r['layout'] or {};delta=layout.get('translation_m',[0,0,0]);balls=layout.get('ball_offsets_m',{})
            series.append(dict(family=family,layout=r['name'],nominal_combinations=r['nominal_combinations'],
                column_dx_mm=delta[0]*1000,column_dy_mm=delta[1]*1000,C_dz_mm=balls.get('C',[0,0,0])[2]*1000,
                D_dz_mm=balls.get('D',[0,0,0])[2]*1000,platform_dz_mm=layout.get('platform_lift_m',0)*1000))
    with (out/'screening.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fields);writer.writeheader();writer.writerows(series)
    image=Image.new('RGB',(1600,900),(12,21,33));d=ImageDraw.Draw(image)
    white=(232,239,247);grey=(160,179,199);blue=(63,202,211)
    d.text((28,20),'1105_03 / 孔位、球高与平台升降试算',font=_font(30),fill=white)
    d.text((28,66),'普通障碍10 mm · 选定近接对2 mm · 有界接触5 mm · 表架安装角保持左90° / 右0°',font=_font(18),fill=blue)
    for x,title,subtitle in [(30,'整柱换台面孔位','台面孔距25 mm；原柱底中心对齐候选孔'),(555,'C / D 球在柱侧上下换孔','柱侧孔距10 mm；A、B保持原位'),(1110,'平台整体升降','台面及随动安装件、柱和球一起移动')]:
        d.text((x,126),title,font=_font(23),fill=white);d.text((x,166),subtitle,font=_font(16),fill=grey)
    def tile(x,y,value,w=72,h=50):
        color=(25+value*3,45+value*13,65+value*12)
        d.rectangle((x,y,x+w-4,y+h-4),fill=color)
        d.text((x+w/2-9,y+9),str(value),font=_font(22 if w>50 else 16),fill=white)
    rows={r['name']:r for r in groups['column']}
    for j,col in enumerate([9,10,11]):d.text((140+j*96,225),f'第{col}列',font=_font(17),fill=grey)
    for i,row in enumerate(range(2,8)):
        d.text((35,282+i*58),f'第{row}排',font=_font(17),fill=grey)
        for j,col in enumerate([9,10,11]):tile(135+j*96,268+i*58,rows[f'hole_R{row:02}_C{col:02}']['nominal_combinations'],92,55)
    d.text((30,660),'排：从机器人向前；列：从机器人右侧向左',font=_font(16),fill=grey)
    d.text((30,696),'向前移得较远，固定朝向下的可达性下降。',font=_font(17),fill=white)
    side_map={(round((r['layout']['ball_offsets_m']['C'][2]*1000)),round((r['layout']['ball_offsets_m']['D'][2]*1000))):r['nominal_combinations'] for r in groups['side']}
    values=list(range(-40,41,10))
    d.text((585,220),'横轴：D相对原孔位的高度变化 / mm',font=_font(16),fill=grey)
    for j,value in enumerate(values):d.text((605+j*45,250),f'{value:+d}',font=_font(13),fill=grey)
    for i,c in enumerate(values):
        d.text((553,294+i*39),f'{c:+d}',font=_font(15),fill=grey)
        for j,v in enumerate(values):tile(600+j*45,282+i*39,side_map[c,v],44,37)
    d.text((555,660),'纵轴：C的高度变化 / mm',font=_font(16),fill=grey)
    d.text((555,696),'所测侧孔组合最多仍找到6组名义候选。',font=_font(17),fill=white)
    expanded={round(r['layout']['platform_lift_m']*1000):r['nominal_combinations'] for r in groups['platform_pose'] if not r['layout'].get('ball_offsets_m')}
    for i,r in enumerate(groups['height']):
        value=round(r['layout']['platform_lift_m']*1000);n=r['nominal_combinations'];y=242+i*31
        d.text((1110,y),f'{value:+4d} mm',font=_font(16),fill=grey)
        d.rectangle((1210,y+3,1210+n*30,y+20),fill=(49,106,150))
        d.text((1460,y),f'{n}/8',font=_font(16),fill=white)
        if value in expanded:d.text((1520,y),str(expanded[value]),font=_font(16),fill=blue)
    d.text((1110,680),'蓝字：另加测头朝向候选后的数量',font=_font(15),fill=blue)
    d.text((1110,714),'较高/较低并不自动带来更大工作空间。',font=_font(16),fill=white)
    d.line((28,770,1572,770),fill=(61,80,99),width=1)
    d.text((30,795),'格子与条形数字 = /8 名义组合候选数，不是连续门禁通过数。',font=_font(23),fill=(255,192,105))
    d.text((30,842),'未找到八组齐全的名义候选；不能把粗筛失败当作实机必撞，也不能据此批准加工或整批运行。',font=_font(18),fill=grey)
    image.save(out/'hole_height_screening_zh.png')
    certs={}
    for root in certificates:
        root=Path(root)
        if (root/'report.json').is_file():certs[root.name]=dict(report=read(root/'report.json'),cases=read(root/'cases.json'),source=str(root))
    summary=dict(screening_trials=sum(len(r) for r in groups.values()),trials_include_control_and_repeated_layouts=True,
        trial_counts={k:len(v) for k,v in groups.items()},maximum_nominal_combinations=max(r['nominal_combinations'] for v in groups.values() for r in v),
        required_combinations=8,all_eight_nominal_found=False,certificates=certs,hardware_tested=False,platform_travel_verified=False,
        recommendation='Keep current manufacturing geometry for now; investigate cross-arm posture and waist/shoulder collision model before choosing a new mounting hole or height')
    write_json(out/'summary.json',summary)
    (out/'README.md').write_text('# 1105_03 孔位与平台高度试算\n\n'
        '所测台面换孔、柱侧换孔、平台升降和测头朝向候选均未找到双臂八组齐全的名义候选。当前不推荐为此改加工图。\n\n'
        '![离线筛选](hole_height_screening_zh.png)\n\n'
        '图中数字为名义IK和采样路径的筛选结果；连续关节包络门禁另见`summary.json`的certificates。\n\n'
        '完整数列在`screening.csv`。平台升降试算为相对CAD的−150到+150 mm；实际行程、锁紧和调节过程未验收。\n\n'
        '台面孔位采用CAD提取的209个孔（25 mm孔距）；柱侧采用20排10 mm孔距。台面换孔假设柱底中心参考与孔中心对齐，螺栓/转接件兼容性未审核。\n',encoding='utf-8')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for x in ['output','column','side','height','pose','platform-pose']:p.add_argument('--'+x,required=True)
    p.add_argument('--certificates',nargs='*',default=[]);a=p.parse_args()
    build(a.output,a.column,a.side,a.height,a.pose,a.platform_pose,a.certificates)
