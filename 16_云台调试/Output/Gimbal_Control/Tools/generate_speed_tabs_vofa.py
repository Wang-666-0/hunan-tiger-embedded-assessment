"""Append read-only speed tabs to an exported VOFA 1.3 native profile.
Reads files only; does not access the serial port or modify the installed app.
"""
from pathlib import Path
from copy import deepcopy
import argparse
import hashlib
import json
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--include-sim', action='store_true', help='Add optional, visibly marked SIM widget tab')
    args = parser.parse_args()
    names = ('vofa+.tabviews.json', 'vofa+.cmds.json', 'vofa+.config.json')
    source = {n:json.loads((args.source/n).read_text(encoding='utf-8-sig')) for n in names}
    tabs, commands, config = [deepcopy(source[n]) for n in names]
    originals = deepcopy(tabs['ctx'][0]['tabs'])
    original_commands = deepcopy(commands['ctx']['subCmds'])
    widgets = [w for t in originals for w in t['widgets']]
    def template(path):
        return deepcopy(next(w for w in widgets if w['path']==path))
    def place(w,x,y,width,height):
        w['ctx']['.']['ctx'] = {'.':dict(x=x,y=y,width=width,height=height),
            'resize_border':{'ctx':{'.':dict(full_screen=False,full_height=False,full_width=False)}}}
        return w
    def title(w,label):
        w['ctx']['name_menu']['ctx']['attr'].update(name_=label,name_link_ch=False,name_link_cmd=False)
    def command(label,text):
        index=len(commands['ctx']['subCmds'])
        commands['ctx']['subCmds'].append(dict(name=label,intro_on=True,intro='只读切换/查询，不启动电机',
            hex_on=False,loop_on=False,loop_ms=100,loop_count=1,
            cmd_hex=(text+'\n').encode('ascii').hex(' ').upper(),is_group=False,down=False,subCmds=[]))
        return index
    def button(label,text,x,y,width):
        w=place(template('bound_bt'),x,y,width,68)
        w['ctx']['cmd_menu']['ctx']=[command(label,text)]
        for i,arg in enumerate(w['ctx']['argument_menu']['ctx']['.']['model_ctx']): arg['enabled']=i==0
        title(w,label)
        return w
    def gauge(label,channel,x,y,low,high,decimal=0):
        w=place(template('gauge'),x,y,160,250)
        w['ctx']['.'].update({'from':low,'to':high,'decimal':decimal,'danger_on':False})
        w['ctx']['ch_menu']['ctx']=channel
        w['ctx']['value_menu']['ctx']['attr']['decimal']=decimal
        title(w,label)
        return w
    def chart(channels,low,high):
        w=place(template('WaveChart'),0,86,960,530)
        ctx=w['ctx'];ctx['.'].update(font_size=16,xy_mode=False)
        ctx['axis_x_wave']['ctx']['.'].update(left_index=48999,right_index=49999,unit_text='ms',decimal=0)
        ctx['axis_x_wave']['ctx']['container']['ctx']=[]
        for key in ('axis_y_wave','axis_y_bar'):ctx[key]['ctx']['.'].update(bottom_value=low,top_value=high,decimal=0)
        ctx['rbw']['ctx']['.'].update(lines=channels,fake_continuous=False,line_width=2)
        ctx['wave_check']['checked']=True
        return w
    def tab(name,items):
        return dict(name=name,widgets=items,file_url='',bg=deepcopy(originals[0]['bg']))
    feeder = [button('拨盘转子转速 rpm · FEEDER VIEW','FEEDER VIEW',0,0,480),
        button('拨盘只读状态','FEEDER STATUS',500,0,240),chart([2,3],-6000,6000),
        gauge('拨盘实测rpm I2',2,980,88,-6000,6000),gauge('拨盘目标rpm I3',3,1152,88,-6000,6000),
        gauge('反馈在线 I16',16,980,350,0,1),gauge('故障码 I19',19,1152,350,0,65535)]
    wheels = [button('摩擦轮转速 rpm · YAW VIEW','YAW VIEW',0,0,480),
        button('摩擦轮只读状态','WHEEL SPEED',500,0,240),chart([36,37],-200,7000),
        gauge('左轮rpm I36',36,980,88,0,7000),gauge('右轮rpm I37',37,1152,88,0,7000),
        gauge('运行状态 I35',35,980,350,0,1),gauge('故障码 I38',38,1152,350,0,65535)]
    tabs['ctx'][0]['tabs'].extend([tab('拨盘转速',feeder),tab('摩擦轮转速',wheels)])
    assert tabs['ctx'][0]['tabs'][:len(originals)]==originals
    assert commands['ctx']['subCmds'][:len(original_commands)]==original_commands
    new_commands=commands['ctx']['subCmds'][len(original_commands):]
    decoded=[bytes.fromhex(c['cmd_hex']).decode('ascii') for c in new_commands]
    assert decoded==['FEEDER VIEW\n','FEEDER STATUS\n','YAW VIEW\n','WHEEL SPEED\n']
    for c in new_commands:assert not c['loop_on'] and c['loop_count']==1
    for t in tabs['ctx'][0]['tabs'][-2:]:
        for w in t['widgets']:
            if w['path']=='bound_bt':
                args_=w['ctx']['argument_menu']['ctx']['.']['model_ctx']
                assert args_[0]['enabled'] and all(not a['enabled'] for a in args_[1:])
                assert all(0<=i<len(commands['ctx']['subCmds']) for i in w['ctx']['cmd_menu']['ctx'])
    if args.include_sim:
        sim_widget={'path':'pitch_sim_plot','ctx':{'.':{
            'ctx':{'.':dict(x=0,y=0,width=1320,height=690),
                'resize_border':{'ctx':{'.':dict(full_screen=False,full_height=False,full_width=False)}}},
            'preferences_json':json.dumps({'automatic':True})}}}
        tabs['ctx'][0]['tabs'].append(tab('Pitch SIM 模拟',[sim_widget]))
    args.output.mkdir(parents=True,exist_ok=True)
    for name,value in zip(names,(tabs,commands,config)):
        (args.output/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    report={'source':str(args.source.resolve()),'source_hashes':{n:hashlib.sha256((args.source/n).read_bytes()).hexdigest() for n in names},
        'tabs':[t['name'] for t in tabs['ctx'][0]['tabs']], 'old_tabs_unchanged':True,'old_commands_unchanged':True,
        'old_settings_unchanged':config==source[names[2]],'commands_added':decoded,
        'feeder':{'group':'feeder','actual_rotor_rpm':2,'target_rotor_rpm':3,'online':16,'faults':19,'gear_ratio':13},
        'flywheels':{'group':'gimbal','left_filtered_rpm':36,'right_filtered_rpm':37,'active':35,'faults':38,'assumed_pole_pairs':7},
        'hardware_used':False,'native_gui_import_verified':False}
    (args.output/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'tabs':report['tabs'],'old_tabs_unchanged':True,'commands_added':len(decoded),'native_gui_import_verified':False},ensure_ascii=False))

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
