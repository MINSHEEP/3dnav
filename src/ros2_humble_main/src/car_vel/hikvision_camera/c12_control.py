"""Validated C12 UDP controls; command receipt is distinct from physical completion."""
import ipaddress
import socket
import threading
import time


def field(name, label, low, high, default=0):
    return dict(name=name, label=label, min=low, max=high, default=default)


COMMANDS = {
    'PTZ': dict(label='云台模式', dest='G', options={'00':'停止','05':'回中','06':'跟随','07':'锁头','0A':'吊装','0B':'倒装'}, read=False),
    'GAY': dict(label='航向绝对角度', dest='G', fields=[field('angle','角度 °',-90,90),field('speed','速度（0.5°/秒）',1,127,8)]),
    'GAP': dict(label='俯仰绝对角度', dest='G', fields=[field('angle','角度 °',-90,90),field('speed','速度（0.5°/秒）',1,127,8)]),
    'GAR': dict(label='横滚绝对角度', dest='G', fields=[field('angle','角度 °',-90,90),field('speed','速度（0.5°/秒）',1,127,8)]),
    'GAM': dict(label='航向与俯仰联合角度',dest='G',fields=[field('yaw','航向 °',-90,90),field('pitch','俯仰 °',-90,90),field('speed','速度（0.5°/秒）',1,99,8)]),
    'GAA': dict(label='姿态回传频率',dest='G',fields=[field('value','Hz（0关闭）',0,100,10)]),
    'GAC': dict(label='读取云台姿态',dest='G',readonly=True),
    'SUM': dict(label='目标跟踪',dest='G',options={'00':'停止跟踪','01':'确认跟踪'}),
    'GOT': dict(label='跟踪目标坐标',dest='G',fields=[field('x','原始画面 X（反转前）',0,1280,640),field('y','原始画面 Y（反转前）',0,720,360)]),
    'REC': dict(label='相机存储卡录像',options={'00':'停止录像','01':'开始录像'},read=True),
    'CAP': dict(label='相机存储卡拍照',options={'01':'拍照'}),
    'VID': dict(label='录像分辨率',options={'00':'720p','01':'1080p','02':'2K','03':'4K（需设备支持）'},read=True),
    'SDC': dict(label='存储卡容量',readonly=True),
    'AWB': dict(label='白平衡',options={'00':'自动','01':'夜间','02':'白炽灯','03':'荧光灯','04':'暖荧光灯','05':'日光','06':'阴天','07':'黄昏','08':'阴影'},read=True),
    'IQE': dict(label='图像参数',read=True,fields=[field('lens','镜头 ID',0,15),field('style','风格：0自定义 1标准 2明亮 3艳丽',0,3)]+[field(k,l,0,255,127) for k,l in [('hue','色调'),('brightness','亮度'),('saturation','饱和度'),('contrast','对比度'),('sharpness','锐度')]]),
    'DZM': dict(label='数码变焦',options={'01':'1×','02':'2×','03':'3×','04':'4×','0A':'放大一级','0B':'缩小一级'},read=True),
    'IMG': dict(label='红外伪彩',options={'01':'白热','03':'辉金','04':'铁红','05':'彩虹','06':'微光','07':'极光','08':'红热','09':'丛林','0A':'医疗','0B':'黑热','0C':'金红'},read=True),
    'VOM': dict(label='视频输出（改变翻转会叠加网页已有180°旋转）',read=True,fields=[field('hflip','水平翻转 0/1',0,1),field('vflip','垂直翻转 0/1',0,1),field('fps','帧率',1,60,30),field('gop','关键帧间隔',1,255,30),field('bitrate','码率 kbps',100,10240,1024),field('enabled','输出 0关/1开',0,1,1)]),
    'VER': dict(label='软件版本',readonly=True),
    'MOD': dict(label='相机型号',readonly=True),
    'EXT': dict(label='OSD / LED / 热像校准',read=True,fields=[field('calibrate','锅盖效应校准 0/1',0,1),field('osd','OSD 0/1',0,1,1),field('led','LED 0/1',0,1)]),
    'TAS': dict(label='自动快门间隔（协议范围存在歧义，暂仅读取）',readonly=True),
    **{tag:dict(label=label,read=True,network=True) for tag,label in [('IPV','相机 IP'),('GTW','网关'),('DNS','DNS'),('BNS','备用 DNS')]},
}


def frame(dest, tag, data='00', operation='w'):
    header = '#TP' if dest == 'G' or (len(data) == 2 and tag not in ('IPV','GTW','DNS','BNS')) else '#tp'
    body = f'{header}U{dest}{len(data):X}{operation}{tag}{data}'.encode('ascii')
    return body + f'{sum(body)&255:02X}'.encode()


def encode(tag, payload):
    spec = COMMANDS[tag]
    if payload.get('read'):
        if not (spec.get('read') or spec.get('readonly')): raise ValueError('该命令没有定义查询')
        return frame(spec.get('dest','D'),tag,operation='r')
    if spec.get('readonly'): raise ValueError('该功能仅支持读取')
    values = {}
    for f in spec.get('fields',[]):
        v = float(payload[f['name']])
        if not f['min'] <= v <= f['max']: raise ValueError(f["label"]+'超出范围')
        if f['name'] not in ('angle','yaw','pitch') and v != int(v): raise ValueError('参数必须为整数')
        values[f['name']] = v
    def h(k): return f'{int(values[k]):02X}'
    def angle(k): return f'{round(values[k]*100)&65535:04X}'
    if spec.get('network'):
        if payload.get('confirm') is not True: raise ValueError('修改网络将断开相机，需要明确确认')
        data = str(ipaddress.IPv4Address(payload['value']))
    elif 'options' in spec:
        data = str(payload.get('value',''))
        if data not in spec['options']: raise ValueError('无效选项')
    elif tag in ('GAY','GAP','GAR'): data=angle('angle')+h('speed')
    elif tag=='GAM': data=angle('yaw')+h('speed')+angle('pitch')+h('speed')
    elif tag=='GOT': data=f'{int(values["x"]):04X}{int(values["y"]):04X}'
    elif tag=='IQE': data=f'{int(values["lens"]):X}{int(values["style"]):X}'+''.join(h(k) for k in ('hue','brightness','saturation','contrast','sharpness'))
    elif tag=='VOM': data=f'{int(values["hflip"])}{int(values["vflip"])}'+h('fps')+h('gop')+f'{int(values["bitrate"]):04X}{int(values["enabled"])}'
    elif tag=='EXT': data='0'+''.join(str(int(values[k])) for k in ('calibrate','osd','led'))
    else: data=h('value')
    return frame(spec.get('dest','D'),tag,data)


class C12Control:
    def __init__(self, host='192.168.144.108', port=5000):
        self.address=(host,port)
        self.lock=threading.Lock()
        self.motion_guard=threading.RLock()
        self.motion_session=None

    def execute(self, payload):
        tag=str(payload.get('command',''))
        if tag not in COMMANDS: raise ValueError('未支持的 C12 命令')
        packet=encode(tag,payload)
        self.motion({"operation":"stop"})
        with self.lock, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
            sock.connect(self.address);sock.settimeout(.8)
            # GAC is documented as active telemetry, so request one-Hz output
            # and disable it on this socket after receiving the attitude.
            sock.send(frame('G','GAA','0A') if tag=='GAC' else packet)
            reply=None
            deadline=time.monotonic()+.8
            try:
                while time.monotonic()<deadline:
                    raw=sock.recv(4096)
                    try:
                        if len(raw)<12 or raw[:3] not in (b'#TP',b'#tp'): continue
                        try:
                            # C12 firmware reports D reply lengths as byte counts
                            # for IMG/VER, while G replies count ASCII characters.
                            length = int(chr(raw[5]),16)
                            actual = len(raw)-12
                            valid = (int(raw[-2:],16)==(sum(raw[:-2])&255)
                                     and (actual==length or (raw[3:5]==b'DU' and actual==2*length)))
                        except ValueError:
                            continue
                        if not valid: continue
                        if raw[7:10] == b'ERE': raise ValueError('相机拒绝该指令')
                        if raw[7:10].decode()!=tag: continue
                        reply=raw.decode('ascii');break
                    except (UnicodeError,IndexError): continue
            except socket.timeout: pass
            finally:
                if tag=='GAC': sock.send(frame('G','GAA','00'))
            result=dict(sent=True,acknowledged=reply is not None,command=tag,reply=reply,
                        message='相机已回复；动作完成状态需观察画面' if reply else '指令已发送，但未收到确认；设备是否支持尚未验证')
            if reply:
                data=reply[10:-2];result['data']=data
                if tag=='GAC' and len(data)==12:
                    nums=[int(data[i:i+4],16) for i in (0,4,8)]
                    result['angles']=[(v-65536 if v>=32768 else v)/100 for v in nums]
            return result


    def motion(self, payload):
        operation = payload.get('operation')
        token = str(payload.get('token', ''))
        with self.motion_guard:
            session = self.motion_session
            if operation == 'keepalive':
                if (not session or session['token'] != token
                        or session['stop'].is_set() or not session['thread'].is_alive()
                        or time.monotonic() >= session['deadline']):
                    raise ValueError('云台操作已结束，请重新按住方向按钮')
                session['deadline'] = time.monotonic() + .5
                return dict(active=True)
            if operation not in ('start', 'stop'):
                raise ValueError('无效云台操作')
            if operation == 'stop' and token and session and session['token'] != token:
                return dict(active=True, message='旧操作已经结束')
            if session:
                session['stop'].set()
                session['thread'].join(timeout=1)
                if session['thread'].is_alive():
                    raise ValueError('云台停止仍在处理中')
                self.motion_session = None
            if operation == 'stop':
                return dict(active=False, message='已结束持续控制并发送停止帧，请观察云台')
            if not token or len(token) > 100:
                raise ValueError('缺少操作标识')
            direction = payload.get('direction')
            directions = {'left':('GSY',-8), 'right':('GSY',8),
                          'up':('GSP',8), 'down':('GSP',-8)}
            if direction not in directions:
                raise ValueError('无效方向')
            value = float(payload.get('speed', 8))
            if not 1 <= value <= 127 or value != int(value):
                raise ValueError('速度必须为 1–127 的整数')
            tag, sign = directions[direction]
            speed = int(value) * (1 if sign > 0 else -1)
            session = dict(token=token, stop=threading.Event(),
                           deadline=time.monotonic()+.5, error='')
            thread = threading.Thread(target=self._run_motion,
                                      args=(session, tag, speed), daemon=True,
                                      name='c12-held-motion')
            session['thread'] = thread
            self.motion_session = session
            thread.start()
            return dict(active=True, message='持续控制已启动，松开停止')

    def _run_motion(self, session, tag, speed):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(.1)
            try:
                sock.connect(self.address)
                while not session['stop'].is_set() and time.monotonic() < session['deadline']:
                    sock.send(frame('G', tag, f'{speed & 255:02X}'))
                    session['stop'].wait(.1)
            except OSError as error:
                session['error'] = str(error)
            finally:
                session['stop'].set()
                # Repeat zero-speed frames since UDP delivery is not guaranteed.
                for _ in range(3):
                    for stop_tag in ('GSY', 'GSP', 'PTZ'):
                        try:
                            sock.send(frame('G', stop_tag, '00'))
                        except OSError as error:
                            session['error'] = str(error)
                    time.sleep(.02)

    def close(self):
        self.motion({'operation':'stop'})
