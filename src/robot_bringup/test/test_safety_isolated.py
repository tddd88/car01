#!/usr/bin/env python3
"""仅运行隔离 ROS master/假输入；绝不加载硬件驱动。"""
import json
import math
import os
import pathlib
import signal
import socket
import subprocess
import time

assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11401'
assert os.environ.get('ROS_IP') == '127.0.0.1'
import rospy
import rosgraph
import tf2_ros
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

WORKSPACE=pathlib.Path(os.environ.get('CAR01_TEST_WORKSPACE',str(pathlib.Path(__file__).resolve().parents[3])))
ROOT=pathlib.Path(os.environ.get('CAR01_TEST_OUTPUT','/tmp/car01_safety_test'))
ROOT.mkdir(parents=True,exist_ok=True)
BIN=WORKSPACE/'devel/lib/robot_bringup'
results=[]; processes=[]; logs=[]; latest={}; samples=[]
def process(args,name):
    log=open(ROOT/('safety_test_'+name+'.log'),'w'); logs.append(log)
    p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p); return p
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=3)
def remember(topic):
    def cb(msg):
        latest[topic]=(time.monotonic(),msg)
        if topic=='requested': samples.append(time.monotonic())
    return cb
def val(topic): return latest[topic][1]
def speed(t): return max(abs(t.linear.x),abs(t.linear.y),abs(t.angular.z))
def expect(name,condition,detail=''):
    results.append({'name':name,'passed':bool(condition),'detail':detail})
    print(('PASS ' if condition else 'FAIL ')+name,flush=True)
def pump(seconds,callback=None):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if callback: callback()
        time.sleep(.035)
def twist(x=0,y=0,w=0):
    t=Twist(); t.linear.x=x; t.linear.y=y; t.angular.z=w; return t
def automatic(x=0,y=0,w=0,age=0,frame='base_link'):
    t=TwistStamped(); t.header.stamp=rospy.Time.now()-rospy.Duration(age); t.header.frame_id=frame
    t.twist=twist(x,y,w); return t
def joy(enabled=False,x=0,y=0,w=0,age=0):
    j=Joy(); j.header.stamp=rospy.Time.now()-rospy.Duration(age)
    j.axes=[y,x,0,w]; j.buttons=[0]*8; j.buttons[5]=int(enabled); return j
def scan(points=(),frame='base_link',age=0,kind='normal'):
    s=LaserScan(); s.header.stamp=rospy.Time.now()-rospy.Duration(age); s.header.frame_id=frame
    s.angle_min=-math.pi; s.angle_increment=2*math.pi/720; s.angle_max=math.pi-s.angle_increment
    s.range_min=.01; s.range_max=10.; s.ranges=[float('inf')]*720
    for x,y in points:
        angle=math.atan2(y,x); i=int(round((angle-s.angle_min)/s.angle_increment))%720
        s.ranges[i]=math.hypot(x,y)
    if kind=='invalid': s.ranges=[float('nan')]*720
    if kind=='partial': s.angle_increment=math.pi/720; s.angle_max=s.angle_min+math.pi
    return s

try:
    with socket.socket() as probe: probe.bind(('127.0.0.1',11401))
    master=process(['roscore','-p','11401'],'master')
    for _ in range(100):
        try:
            rosgraph.Master('/safety_test_setup').getPid(); break
        except Exception: time.sleep(.1)
    else: raise RuntimeError('隔离master未就绪')
    rospy.init_node('safety_isolated_test',anonymous=True,disable_signals=True)
    pubs={
        'auto':rospy.Publisher('/cmd_vel_auto',TwistStamped,queue_size=1),
        'joy':rospy.Publisher('/joy',Joy,queue_size=1),
        'raw':rospy.Publisher('/cmd_vel_raw',Twist,queue_size=1),
        'scan':rospy.Publisher('/scan',LaserScan,queue_size=1),
        'requested':rospy.Publisher('/cmd_vel_requested',TwistStamped,queue_size=1),
    }
    subscriptions=[
        rospy.Subscriber('/cmd_vel_requested',TwistStamped,remember('requested'),queue_size=10),
        rospy.Subscriber('/cmd_vel_raw',Twist,remember('raw'),queue_size=10),
        rospy.Subscriber('/cmd_vel',Twist,remember('final'),queue_size=10),
        rospy.Subscriber('/control/manual_active',Bool,remember('manual'),queue_size=10),
        rospy.Subscriber('/control/ready',Bool,remember('ready'),queue_size=10),
        rospy.Subscriber('/control/reason',String,remember('control_reason'),queue_size=10),
        rospy.Subscriber('/safety/ready',Bool,remember('scan_ready'),queue_size=10),
        rospy.Subscriber('/safety/reason',String,remember('reason'),queue_size=10)]
    arbiter=process([str(BIN/'cmd_vel_arbiter_node')],'arbiter')
    repeat=process([str(BIN/'joy_repeat_node'),'cmd_vel:=/cmd_vel_raw','_enable_button:=5','_scale_vx:=0.5'],'joy')
    safety=process([str(BIN/'laser_safety_node')],'laser')
    pump(1)
    pump(.4,lambda:pubs['auto'].publish(automatic(.25)))
    expect('从未使用手柄时自动请求可运行',val('ready').data and abs(val('requested').twist.linear.x-.25)<1e-4)
    expect('缺扫描时最终速度强制为零',speed(val('final'))<1e-6 and not val('scan_ready').data)
    pump(.9,lambda:(pubs['auto'].publish(automatic(.25)),pubs['scan'].publish(scan())))
    expect('新鲜全周扫描放行自动前进',val('scan_ready').data and val('final').linear.x>.20)
    expect('仲裁周期至少20Hz',len(samples)>20 and (len(samples)-1)/(samples[-1]-samples[0])>=20)
    pump(.3,lambda:(pubs['auto'].publish(automatic(.25,age=1)),pubs['scan'].publish(scan())))
    expect('旧自动时间戳不能被接收重打时间变新',speed(val('requested').twist)<1e-6)
    pump(.3,lambda:pubs['auto'].publish(automatic(.25,age=-1)))
    expect('未来自动时间戳拒绝',speed(val('requested').twist)<1e-6)
    pump(.4,lambda:pubs['auto'].publish(automatic(.25)))
    pump(.5)
    expect('自动上游断流后请求清零',speed(val('requested').twist)<1e-6)
    pump(.5,lambda:(pubs['auto'].publish(automatic(.25)),pubs['joy'].publish(joy(True)),pubs['scan'].publish(scan())))
    expect('使能按下且摇杆零也立即人工接管',val('manual').data and speed(val('requested').twist)<1e-6)
    pump(.5,lambda:(pubs['auto'].publish(automatic(.25)),pubs['joy'].publish(joy(True,x=-.6)),pubs['scan'].publish(scan())))
    expect('使能非零人工速度覆盖自动',val('requested').twist.linear.x<-.25)
    pump(.7,lambda:(pubs['auto'].publish(automatic(.25)),pubs['raw'].publish(twist(-.3)),pubs['scan'].publish(scan())))
    expect('Joy失联时新鲜重发raw不能冒充设备在线',not val('ready').data and speed(val('requested').twist)<1e-6)
    pump(.25,lambda:pubs['joy'].publish(joy(True,x=.2)))
    rospy.wait_for_service('/control/reset',timeout=3); reset=rospy.ServiceProxy('/control/reset',Trigger)
    expect('重连仍按使能时拒绝解锁',not reset().success)
    pump(.25,lambda:pubs['joy'].publish(joy(False)))
    expect('新鲜归零释放使能后允许显式解锁',reset().success)
    pump(.25,lambda:pubs['joy'].publish(joy(False)))
    expect('解锁不会复用旧自动缓存',val('ready').data and speed(val('requested').twist)<1e-6)
    pump(.25,lambda:(pubs['joy'].publish(joy(False)),pubs['auto'].publish(automatic(.15))))
    expect('解锁后新自动请求可用',val('requested').twist.linear.x>.1)
    pump(.25,lambda:pubs['joy'].publish(joy(True)))
    stale_auto=automatic(.2)
    pump(.2,lambda:pubs['joy'].publish(joy(False)))
    pump(.1,lambda:pubs['auto'].publish(stale_auto))
    expect('释放使能不复用接管前自动指令',speed(val('requested').twist)<1e-6)
    stop_control=rospy.ServiceProxy('/control/stop',Trigger)
    pump(.25,lambda:(pubs['joy'].publish(joy(True,x=.6)),pubs['auto'].publish(automatic(.2))))
    expect('显式stop服务成功',stop_control().success)
    pump(.3,lambda:(pubs['joy'].publish(joy(True,x=.6)),pubs['auto'].publish(automatic(.2))))
    expect('manual运动中stop持续锁止所有速度',not val('ready').data and speed(val('requested').twist)<1e-6)
    expect('operator_stop原因与input_lost可区分',val('control_reason').data=='OPERATOR_STOP')
    expect('operator_stop使能按住不允许reset',not reset().success)
    pump(.3,lambda:(pubs['joy'].publish(joy(False)),pubs['auto'].publish(automatic(.2))))
    expect('松手不能自动解除显式stop',not val('ready').data and speed(val('requested').twist)<1e-6)
    expect('显式stop在安全输入后可reset',reset().success)
    pump(.3,lambda:(pubs['joy'].publish(joy(False)),pubs['auto'].publish(automatic(.2))))
    expect('auto运动中stop服务成功',stop_control().success)
    pump(.3,lambda:(pubs['joy'].publish(joy(False)),pubs['auto'].publish(automatic(.2))))
    expect('auto运动中stop持续锁止直到reset',not val('ready').data and speed(val('requested').twist)<1e-6)
    expect('auto停止可在安全输入后reset',reset().success)
    stop(repeat); stop(arbiter); pump(.2)

    # 安全几何测试直接输入requested，仍仅存在同一laser最终出口。
    def run_safe(command,points=(),seconds=.65,frame='base_link',age=0,kind='normal',cmd_age=0,cmd_frame='base_link'):
        pump(.12,lambda:(pubs['requested'].publish(automatic()),pubs['scan'].publish(scan())))
        pump(seconds,lambda:(pubs['requested'].publish(automatic(*command,age=cmd_age,frame=cmd_frame)),
                             pubs['scan'].publish(scan(points,frame,age,kind))))
        return val('final')
    v=run_safe((.2,0,0))
    expect('清场平移最终速度正常',v.linear.x>.18)
    v=run_safe((.2,0,0),[(.19,0)])
    expect('正前方近点抑制前进',abs(v.linear.x)<.002)
    v=run_safe((-.2,0,0),[(.19,0)])
    expect('正前方近点允许后退脱离',v.linear.x<-.18)
    v=run_safe((0,.2,0),[(0,.12)])
    expect('左侧近点抑制ROS正Y横移',abs(v.linear.y)<.002)
    v=run_safe((0,-.2,0),[(0,.12)])
    expect('左侧近点允许ROS负Y脱离',v.linear.y<-.18)
    v=run_safe((0,-.2,0),[(0,-.12)])
    expect('右侧近点抑制ROS负Y横移',abs(v.linear.y)<.002)
    v=run_safe((.2,.2,0),[(.20,.14)])
    angular_index=round((math.atan2(.14,.20)+math.pi)/(2*math.pi/720))%720
    measured_angle=-math.pi+angular_index*(2*math.pi/720)
    point=(math.hypot(.20,.14)*math.cos(measured_angle),math.hypot(.20,.14)*math.sin(measured_angle))
    # 独立纯平移解析式：反应阶段匀速，随后以0.6m/s²制动；检查整个路径的矩形边界。
    travel_time=.25+math.hypot(v.linear.x,v.linear.y)/(2*.6)
    clearances=[max(abs(point[0]-v.linear.x*travel_time*t/100)-.175,
                    abs(point[1]-v.linear.y*travel_time*t/100)-.11) for t in range(101)]
    expect('角部障碍限速后的解析制动全路径满足margin',
           math.hypot(v.linear.x,v.linear.y)<.28 and min(clearances)>=.025-1e-4,
           'vx=%.6f, vy=%.6f, min_clearance=%.6f, margin=.025'%(v.linear.x,v.linear.y,min(clearances)))
    v=run_safe((.2,.2,0),[(.19,.12)])
    expect('侵入安全边界的角部障碍完全抑制对角前进',speed(v)<.002)
    v=run_safe((0,0,.4),[(.176,.116)])
    expect('矩形角扫掠禁止贴边原地旋转',abs(v.angular.z)<.002)
    v=run_safe((0,0,.4),[(.15,.15)])
    expect('未来旋转扫掠提前减速',abs(v.angular.z)<.30)
    v=run_safe((0,0,.4))
    expect('清场低速旋转可用',v.angular.z>.35)
    v=run_safe((.2,0,0),seconds=.2)
    expect('从零起步有限加速度',0<v.linear.x<.12)
    pump(.1,lambda:pubs['requested'].publish(automatic()))
    expect('零速停车不经过缓慢减速',speed(val('final'))<1e-6)
    v=run_safe((.2,0,0),cmd_age=1)
    expect('安全层拒绝陈旧请求时间戳',speed(v)<1e-6)
    v=run_safe((.2,0,0),cmd_frame='map')
    expect('安全层拒绝非底盘速度坐标系',speed(v)<1e-6)
    v=run_safe((.2,0,0),age=1)
    expect('扫描有接收但源时间戳过期仍停车',speed(v)<1e-6 and not val('scan_ready').data)
    v=run_safe((.2,0,0),kind='invalid')
    expect('全无效点不能伪装清场',speed(v)<1e-6)
    v=run_safe((.2,0,0),kind='partial')
    expect('不足全周覆盖拒绝自动全向',speed(v)<1e-6)
    v=run_safe((.2,0,0),frame='missing_radar_frame')
    expect('未知雷达TF强制停车',speed(v)<1e-6)
    broadcaster=tf2_ros.StaticTransformBroadcaster()
    tr=TransformStamped(); tr.header.stamp=rospy.Time.now(); tr.header.frame_id='base_link'
    tr.child_frame_id='radar_cylinder'; tr.transform.translation.x=.08
    tr.transform.rotation.z=math.sin(math.pi/4); tr.transform.rotation.w=math.cos(math.pi/4)
    broadcaster.sendTransform(tr); pump(.3)
    # 雷达扫描的右方(.0,-.11)经+90°TF与x偏移变成底盘正前方(.19,0)。
    v=run_safe((.2,0,0),[(0,-.11)],frame='radar_cylinder')
    expect('真实TF旋转和平移后正前方障碍正确阻止',abs(v.linear.x)<.002)
    v=run_safe((-.2,0,0),[(0,-.11)],frame='radar_cylinder')
    expect('变换后的正前障碍仍允许后退',v.linear.x<-.18)
    v=run_safe((.2,0,0))
    pump(.55,lambda:pubs['requested'].publish(automatic(.2)))
    expect('scan断流即停且不会复用旧清场',speed(val('final'))<1e-6 and not val('scan_ready').data)
    run_safe((.2,0,0))
    pump(.3,lambda:pubs['scan'].publish(scan()))
    expect('仲裁输出断流后唯一安全出口持续零速',speed(val('final'))<1e-6)
    pubs_state=rosgraph.Master(rospy.get_name()).getSystemState()[0]
    final_nodes=[nodes for topic,nodes in pubs_state if topic=='/cmd_vel'][0]
    expect('最终cmd_vel只有laser安全一个发布者',final_nodes==['/laser_safety_node'],str(final_nodes))
    # 单独检查joy_repeat；没有恶意raw竞争时明确确认停止重放。
    repeat2=process([str(BIN/'joy_repeat_node'),'cmd_vel:=/cmd_vel_raw','_enable_button:=5'],'joy_repeat_only')
    for _ in range(40):
        if pubs['joy'].get_num_connections()>0: break
        pump(.05)
    else: raise RuntimeError('新joy_repeat订阅握手未完成')
    movement=[]
    pump(.5,lambda:(pubs['joy'].publish(joy(True,x=.5)),movement.append(val('raw').linear.x)))
    expect('joy_repeat握手后能连续续发新鲜输入',len(movement)>10 and sum(v>.2 for v in movement)>=len(movement)-3,
           'samples=%d active=%d'%(len(movement),sum(v>.2 for v in movement)))
    pump(.65)
    expect('joy_repeat真实Joy陈旧立即归零',speed(val('raw'))<1e-6)
except Exception as e:
    results.append({'name':'测试异常','passed':False,'detail':repr(e)})
    print(repr(e),flush=True)
finally:
    for p in reversed(processes):
        try: stop(p)
        except Exception: pass
    for f in logs: f.close()
    report={'master':'http://127.0.0.1:11401','hardware_drivers':False,'results':results,
            'passed':sum(r['passed'] for r in results),'failed':sum(not r['passed'] for r in results)}
    (ROOT/'safety_test_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='results'},ensure_ascii=False),flush=True)
raise SystemExit(0 if results and all(r['passed'] for r in results) else 1)
