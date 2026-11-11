'use strict';
const $ = id => document.getElementById(id);
let socket, connected = false, armed = false, mode = '未知', busy = false, avoid = null;
let motorStates = [];
let latestVoxel = null;
let robotQuaternion = [0, 0, 0, 1];
const axes = {x:0,y:0,z:0};
const sticks = [];
let lidarSource = 'compressed';
const pressedKeys = new Set();
let speedScale = 0.5;
const driveLimits = {x:0.6,y:0.3,z:1.0};
function scaledAxes() { return {x:axes.x*speedScale,y:axes.y*speedScale,z:axes.z*speedScale}; }
function notice(message, error=false) { $('notice').textContent = message; $('notice').classList.toggle('error', error); }
function send(data) { if (socket?.readyState === WebSocket.OPEN) { socket.send(JSON.stringify(data)); return true; } busy=false;updateControls();notice('本机服务未连接，请刷新页面并检查终端。', true); return false; }
function clearSticks() { pressedKeys.clear(); axes.x=axes.y=axes.z=0; for(const stick of sticks) stick.reset(); $('command-speed').innerHTML='0.00 <small>m/s</small>' }
function clearCloudView() {
  scanPoints = [];
  mapImage = null; mapMeta = null;
  latestVoxel = null;
  window.go2Scene?.clear();
  $('cloud-empty').hidden = false;
  $('point-count').textContent = '扫描 0 点';
  $('lidar-status').textContent = '等待点云';
  $('lidar-status').classList.remove('fresh');
  $('lidar-meta').textContent = '';
}
function halt() { clearSticks(); armed=false; send({action:'stop'}); updateControls(); }
function keyboardAxes() {
  const forward = Number(pressedKeys.has('ArrowUp')) - Number(pressedKeys.has('ArrowDown'));
  const turn = Number(pressedKeys.has('ArrowLeft')) - Number(pressedKeys.has('ArrowRight'));
  const lateral = Number(pressedKeys.has('KeyA')) - Number(pressedKeys.has('KeyD'));
  axes.x = forward * driveLimits.x;
  axes.y = lateral * driveLimits.y;
  axes.z = turn * driveLimits.z;
}
function updateControls() {
  $('connect').textContent = busy ? '正在建立连接…' : connected ? '断开连接' : '连接机器人 ↗';
  $('connect').disabled=busy;
  $('dot').classList.toggle('on', connected);
  $('connection').textContent=connected?'Go2 已连接':'设备未连接';
  $('arm').textContent=armed?'⌁ 控制已解锁':'⌁ 控制已锁定';
  $('arm').classList.toggle('armed', armed);
  document.body.classList.toggle('drive-armed', armed&&connected);
  $('arm').disabled=!connected;
  $('nav-goal').disabled=!connected;$('map-reset').disabled=!connected;
  document.querySelectorAll('[data-pose]').forEach(b=>b.disabled=!connected);
  document.querySelectorAll('[data-mode]').forEach(b=>b.disabled=!connected);
  document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b.dataset.mode===mode));
  updateAvoid();
  $('mode-label').textContent=mode==='normal'?'普通运动':mode==='mcf'?'MCF 运动':mode==='ai'?'AI 运动':'模式未知';
}
function fresh(id, age, label) {const ok=age!==null&&age<2; $(id).textContent=ok?label:age===null?'等待数据':'数据过期'; $(id).classList.toggle('fresh',ok);}
function number(v,d=2) {return typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—';}
function openSocket() {
  socket=new WebSocket(`ws://${location.host}/ws`);
  socket.onopen=()=>notice('本机服务已就绪。点击「连接机器人」开始接收真实数据。');
  socket.onerror=()=>notice('WebSocket 连接失败，请确认终端中的 go2_web 服务仍在运行。',true);
  socket.onclose=e=>{ connected=armed=busy=false;clearSticks();clearCloudView();updateControls();notice(e.code===4000?'本页已被另一个控制页面接管，请使用最新打开的页面。':'本机服务连接已关闭；请刷新页面。控制已锁定。',true); };
  socket.onmessage=e=>{
    const data=JSON.parse(e.data);
    if(data.type==='status') {
      connected=data.connected; armed=data.armed;mode=data.mode;$('robot-ip').textContent=data.robot_ip?`WebRTC · STA-L · ${data.robot_ip}（${data.ip_source}）`:'WebRTC · STA-L';avoid=connected?data.avoid:null;
      if(connected) busy=false;
      if(!armed) clearSticks();
      updateControls();
      fresh('video-status',data.ages.video,'实时视频');fresh('lidar-status',data.ages.lidar,'实时建图');
      navState=data.nav||{state:'idle'};slamInfo=data.slam||{};updateNavUI();
      if(data.lidar_topic) { const next=data.lidar_topic.endsWith('voxel_map_compressed')?'compressed':'voxel'; if(next!==lidarSource) clearCloudView(); lidarSource=next; }
      if(data.lidar_error) { notice(data.lidar_error,true); $('lidar-status').textContent='点云格式错误'; $('lidar-status').classList.remove('fresh'); }
      $('lidar-meta').textContent=slamInfo.floor!=null?`已知 ${slamInfo.known_m2} m² · 地面 z ${Number(slamInfo.floor).toFixed(2)} m · 本帧 地面 ${slamInfo.frame?.ground??'—'} / 障碍 ${slamInfo.frame?.obstacle??'—'} · 位姿 ${slamInfo.pose_source==='robot_pose'?'雷达里程计':'运动状态'} · 校正 ${slamInfo.correction?`${Number(slamInfo.correction[2]).toFixed(1)}° / ${Math.hypot(slamInfo.correction[0],slamInfo.correction[1]).toFixed(2)} m`:'—'}${slamInfo.match&&slamInfo.match.used===false&&slamInfo.match.reason?'（'+slamInfo.match.reason+'）':''}`:(connected?(slamInfo.status||'等待里程计与地面估计'):'');
      const s=data.ages.state!==null&&data.ages.state<2?data.state:{};
      $('velocity').innerHTML=number(s.velocity?.[0])+'<small>m/s</small>';
      $('height').innerHTML=number(s.body_height)+'<small>m</small>';
      $('rpy').innerHTML=(s.imu_state?.rpy?s.imu_state.rpy.map(v=>number(v*180/Math.PI,0)).join(' / '):'—')+'<small>°</small>';
      $('robot-mode').textContent=s.mode??'—';
      if(Array.isArray(data.motors)) { motorStates=data.motors; renderJointList(); }
      if(Array.isArray(s.position) && s.position.length>=3) robotPosition=s.position.slice(0,3).map(Number);
      if(Array.isArray(s.imu_state?.rpy) && s.imu_state.rpy.length>=3) robotYaw=Number(s.imu_state.rpy[2]);
      // LiDAR odometry shares the voxel map frame. Sport state is a fallback.
      if(data.odom) {
        robotPosition=data.odom.position;
        robotQuaternion=data.odom.quaternion;
        const [x,y,z,w]=robotQuaternion;
        robotYaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
      } else if(Array.isArray(s.imu_state?.quaternion) && s.imu_state.quaternion.length===4) {
        const [w,x,y,z]=s.imu_state.quaternion;
        robotQuaternion=[x,y,z,w];
      } else if(Array.isArray(s.imu_state?.rpy)) {
        const [r,p,y]=s.imu_state.rpy.map(v=>v/2);
        robotQuaternion=[Math.sin(r)*Math.cos(p)*Math.cos(y)-Math.cos(r)*Math.sin(p)*Math.sin(y),
          Math.cos(r)*Math.sin(p)*Math.cos(y)+Math.sin(r)*Math.cos(p)*Math.sin(y),
          Math.cos(r)*Math.cos(p)*Math.sin(y)-Math.sin(r)*Math.sin(p)*Math.cos(y),
          Math.cos(r)*Math.cos(p)*Math.cos(y)+Math.sin(r)*Math.sin(p)*Math.sin(y)];
      }
      if(!connected){
        $('camera').hidden=true;$('camera-empty').hidden=false;clearCloudView();
        robotPosition=[0,0,0];robotQuaternion=[0,0,0,1];robotYaw=0;
      }
      window.go2Scene?.update(motorStates,robotPosition,robotQuaternion);
      if(connected && data.ages.lidar!==null && data.ages.lidar>=2) {
        $('fp-status').textContent='地图数据过期';$('fp-status').classList.remove('fresh');
      }
      if(data.error) notice(data.error,true);
    } else if(data.type==='video') { $('camera').src='data:image/jpeg;base64,'+data.jpeg;$('camera').hidden=false;$('camera-empty').hidden=true; }
    else if(data.type==='points') {
      scanPoints=data.scan||[];
      $('point-count').textContent=`扫描 ${scanPoints.length.toLocaleString()} 点`;
      latestVoxel=data.voxel || null;
      if(latestVoxel) window.go2Scene?.voxelMap.processCompressed(latestVoxel);
      else {
        window.go2Scene?.clear();
        if(scanPoints.length) $('fp-status').textContent='当前数据源无压缩体素网格';
      }
    }
    else if(data.type==='map') {
      if(!data.png){mapImage=null;mapMeta=null;return;}
      const img=new Image(),seq=++mapSeq,meta={origin:data.origin,resolution:data.resolution};
      img.onload=()=>{if(seq===mapSeq){mapImage=img;mapMeta=meta;}};
      img.src='data:image/png;base64,'+data.png;
    }
    else {busy=false;updateControls();notice(data.message,data.type==='error');
      if(data.type==='error'&&performance.now()-navActionAt<3000) flashNav(data.message);}
  };
}
$('connect').onclick=()=>{ if(connected){halt();send({action:'disconnect'});}else{busy=true;updateControls();send({action:'connect'});} };
$('arm').onclick=()=>{if(armed){halt();return;}if(window.confirm('即将允许机器人执行姿态、模式和摇杆指令。请确认周围空旷，且遥控器可用。'))send({action:'arm'});};
$('stop').onclick=halt;
function updateAvoid() {
  const button=$('avoid');
  button.disabled=!connected||busy;
  button.classList.toggle('armed',avoid===true);
  button.textContent=avoid===true?'避障 · 已开启':avoid===false?'避障 · 已关闭':'避障 · 未知';
}
$('avoid').onclick=()=>{
  const next=avoid!==true;
  if(!confirm(next?'开启避障？手动驾驶将改为模拟摇杆，速度按比例换算且会被避障服务限制。切换时会先停车并锁定控制。'
    :'关闭避障？手动驾驶将改为直接速度指令，遇到障碍物不会自动停。切换时会先停车并锁定控制。')) return;
  clearSticks();armed=false;busy=true;updateControls();
  send({action:'avoid',enable:next});
};
function updateSpeedLimits() {
  $('speed-value').textContent=Math.round(speedScale*100)+'%';
}
$('speed-slider').oninput=e=>{speedScale=Math.max(.1,Math.min(1,Number(e.target.value)/100));updateSpeedLimits();};
updateSpeedLimits();
document.querySelectorAll('[data-pose]').forEach(button=>button.onclick=()=>{if(confirm(`确认执行「${button.textContent.trim()}」？机器人将发生实际动作。`)){clearSticks();send({action:'pose',name:button.dataset.pose});}});
document.querySelectorAll('[data-mode]').forEach(button=>button.onclick=()=>{if(confirm(`确认切换至${button.textContent.trim()}？切换模式可能引发站立等动作。`)){clearSticks();armed=false;updateControls();send({action:'mode',mode:button.dataset.mode});}});
function joystick(id, apply) {
  const element=$(id),knob=element.querySelector('.knob');let pointer=null;
  const reset=()=>{pointer=null;knob.style.transform='translate(0,0)';apply(0,0);};
  sticks.push({reset});
  function move(event){
    const rect=element.getBoundingClientRect(), radius=rect.width*.30;
    let x=(event.clientX-rect.left-rect.width/2)/radius,y=(event.clientY-rect.top-rect.height/2)/radius;
    const length=Math.hypot(x,y);if(length>1){x/=length;y/=length;}
    if(Math.hypot(x,y)<.12)x=y=0;
    knob.style.transform=`translate(${x*radius}px,${y*radius}px)`;apply(x,y);
  }
  element.onpointerdown=e=>{if(!armed||!connected||!['normal','mcf'].includes(mode)){notice('请连接机器人并解锁控制；当前需处于普通或 MCF 运动模式。',true);return;}clearSticks();pointer=e.pointerId;element.setPointerCapture(pointer);move(e);};
  element.onpointermove=e=>{if(pointer===e.pointerId)move(e);};
  element.onpointerup=element.onpointercancel=element.onlostpointercapture=()=>{reset();if(armed)send({action:'move',...scaledAxes()});};
}
joystick('translate',(x,y)=>{axes.x=-y*driveLimits.x;axes.y=-x*driveLimits.y;});
joystick('rotate',x=>{axes.z=-x*driveLimits.z;});
setInterval(()=>{if(armed&&connected&&['normal','mcf'].includes(mode)){send({action:'move',...scaledAxes()});const command=scaledAxes();if(avoid===true){$('command-speed').innerHTML=Math.round(Math.max(Math.abs(command.x)/driveLimits.x,Math.abs(command.y)/driveLimits.y)*100)+' <small>% 摇杆</small>'}else{$('command-speed').innerHTML=Math.hypot(command.x,command.y).toFixed(2)+' <small>m/s</small>'}}},100);
window.addEventListener('blur',()=>{if(armed)halt();});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&armed)halt();});
window.addEventListener('pagehide',()=>{if(armed)halt();});
window.addEventListener('keydown',e=>{
  if(['INPUT','TEXTAREA','SELECT'].includes(document.activeElement.tagName) || document.activeElement.isContentEditable) return;
  if(e.metaKey||e.ctrlKey||e.altKey) return;
  if(e.code==='Escape'&&goalTool){e.preventDefault();setGoalTool(false);return;}
  if(e.code==='Space'){e.preventDefault();halt();return;}
  const allowed=['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','KeyA','KeyD'];
  if(!allowed.includes(e.code)) return;
  e.preventDefault();
  if(!armed||!connected||!['normal','mcf'].includes(mode)) return;
  pressedKeys.add(e.code);
  keyboardAxes();
});
window.addEventListener('keyup',e=>{
  const allowed=['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','KeyA','KeyD'];
  if(!allowed.includes(e.code)) return;
  e.preventDefault();
  if(!pressedKeys.has(e.code)) return;
  pressedKeys.delete(e.code);
  if(pressedKeys.size){keyboardAxes();}else{clearSticks();send({action:'move',...scaledAxes()});}
});
// ── 实时建图视图：2D 占据栅格 + 过滤后扫描 + 路径/目标，纯 Canvas 绘制 ──
// 地图在 z=0 平面上绘制；保留原有的旋转 / 平移 / 缩放操作。
const canvas=$('cloud'),ctx=canvas.getContext('2d');
const DEFAULT_VIEW={yaw:Math.PI/2,pitch:1.5,zoom:40};   // 俯视，X 朝上（机器狗开机朝向）
let scanPoints=[], robotPosition=[0,0,0], robotYaw=0, yaw=DEFAULT_VIEW.yaw,pitch=DEFAULT_VIEW.pitch, zoom=DEFAULT_VIEW.zoom, panX=0, panY=0, drag=null;
let mapImage=null,mapMeta=null,mapSeq=0,navState={state:'idle'},slamInfo={},goalTool=false,goalDraft=null;
// Canvas Y grows downward. Negate the horizontal depth term so the view
// looks down on the XY plane (Z-up), rather than viewing it from underneath.
function project(p,w,h) { const a=p[0]*Math.cos(yaw)-p[1]*Math.sin(yaw),b=p[0]*Math.sin(yaw)+p[1]*Math.cos(yaw);return [w/2+panX+a*zoom,h*.58+panY+(-b*Math.sin(pitch)-(p[2]||0)*Math.cos(pitch))*zoom]; }
// Inverse of project() on the z=0 plane: screen point → map (x, y).
function unproject(sx,sy,w,h) {
  const A=(sx-w/2-panX)/zoom,B=-(sy-h*.58-panY)/(zoom*Math.sin(pitch));
  return [A*Math.cos(yaw)+B*Math.sin(yaw),-A*Math.sin(yaw)+B*Math.cos(yaw)];
}
function arrow(from,to,color,width,head) {
  const angle=Math.atan2(to[1]-from[1],to[0]-from[0]);
  ctx.beginPath();ctx.moveTo(...from);ctx.lineTo(...to);
  ctx.moveTo(to[0]-head*Math.cos(angle-.5),to[1]-head*Math.sin(angle-.5));
  ctx.lineTo(...to);ctx.lineTo(to[0]-head*Math.cos(angle+.5),to[1]-head*Math.sin(angle+.5));
  ctx.strokeStyle='#0b0e13';ctx.lineWidth=width+3;ctx.stroke();
  ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();
}
function poseArrow(x,y,theta,color,w,h) {
  const a=project([x,y,0],w,h),b=project([x+Math.cos(theta)*.7,y+Math.sin(theta)*.7,0],w,h);
  ctx.save();ctx.lineCap='round';ctx.lineJoin='round';arrow(a,b,color,3,11);
  ctx.beginPath();ctx.arc(...a,4,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();ctx.restore();
}
function drawCloud() {
  const rect=canvas.getBoundingClientRect(),ratio=Math.min(window.devicePixelRatio||1,2),w=rect.width,h=rect.height;
  if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
  ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);ctx.lineWidth=.7;ctx.strokeStyle='#1f3340';
  for(let n=-10;n<=10;n++){for(const pair of [[[n,-10,0],[n,10,0]],[[-10,n,0],[10,n,0]]]){const a=project(pair[0],w,h),b=project(pair[1],w,h);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}}
  // Occupancy grid: the projection of z=0 is affine, so the PNG is drawn with one transform.
  if(mapImage&&mapMeta){
    const [x0,y0]=mapMeta.origin,r=mapMeta.resolution,c=Math.cos(yaw),s=Math.sin(yaw),sp=Math.sin(pitch),o=project([x0,y0,0],w,h);
    ctx.save();ctx.imageSmoothingEnabled=false;
    ctx.setTransform(ratio*zoom*r*c,-ratio*zoom*sp*r*s,-ratio*zoom*r*s,-ratio*zoom*sp*r*c,ratio*o[0],ratio*o[1]);
    ctx.drawImage(mapImage,0,0);ctx.restore();
  }
  ctx.fillStyle='#ffcc33';
  for(const p of scanPoints){const a=project([p[0],p[1],0],w,h);if(a[0]<0||a[0]>w||a[1]<0||a[1]>h)continue;ctx.fillRect(a[0]-1.1,a[1]-1.1,2.2,2.2);}
  $('cloud-empty').hidden=!!mapImage||scanPoints.length>0;
  const path=navState.path||[];
  if(path.length>1){
    ctx.save();ctx.lineCap='round';ctx.lineJoin='round';ctx.beginPath();
    path.forEach((p,i)=>{const a=project([p[0],p[1],0],w,h);i?ctx.lineTo(...a):ctx.moveTo(...a);});
    ctx.strokeStyle='#0b0e13';ctx.lineWidth=5;ctx.stroke();ctx.strokeStyle='#3ddc84';ctx.lineWidth=2.5;ctx.stroke();ctx.restore();
  }
  if(navState.goal&&navState.state!=='idle') poseArrow(navState.goal[0],navState.goal[1],navState.goal[2],'#b388ff',w,h);
  if(goalDraft){
    const [x,y]=goalDraft.start,theta=goalDraft.moved?Math.atan2(goalDraft.end[1]-y,goalDraft.end[0]-x):Math.atan2(y-mapPose()[1],x-mapPose()[0]);
    poseArrow(x,y,theta,'#d6c2ff',w,h);
  }
  // Reference-frame origin of this power-on session (odometry zero).
  const origin=project([0,0,0],w,h);
  ctx.save();ctx.lineCap='round';ctx.lineJoin='round';ctx.font='11px sans-serif';
  for(const [point,label,color] of [[[1,0,0],'X','#ff7676'],[[0,1,0],'Y','#69e49d'],[[0,0,1],'Z','#7aabff']]){
    const tip=project(point,w,h);if(Math.hypot(tip[0]-origin[0],tip[1]-origin[1])<4)continue;
    arrow(origin,tip,color,2,6);
    ctx.lineWidth=3;ctx.strokeStyle='#0b0e13';ctx.strokeText(label,tip[0]+6,tip[1]-6);
    ctx.fillStyle=color;ctx.fillText(label,tip[0]+6,tip[1]-6);
  }
  ctx.beginPath();ctx.arc(...origin,3,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();
  ctx.strokeStyle='#0b0e13';ctx.lineWidth=4;ctx.strokeText('O (0, 0) m',origin[0]+9,origin[1]+18);
  ctx.fillStyle='#e4ecf3';ctx.fillText('O (0, 0) m',origin[0]+9,origin[1]+18);ctx.restore();
  // Robot last, on top of everything (drift-corrected SLAM pose when available).
  const [rx,ry,ryaw]=mapPose();
  const center=project([rx,ry,0],w,h);
  const heading=project([rx+Math.cos(ryaw)*0.65,ry+Math.sin(ryaw)*0.65,0],w,h);
  ctx.save();ctx.lineCap='round';ctx.lineJoin='round';
  ctx.beginPath();ctx.moveTo(...center);ctx.lineTo(...heading);
  ctx.strokeStyle='#10151b';ctx.lineWidth=7;ctx.stroke();
  ctx.strokeStyle='#ff454f';ctx.lineWidth=3;ctx.stroke();
  const angle=Math.atan2(heading[1]-center[1],heading[0]-center[0]);
  ctx.beginPath();ctx.moveTo(...heading);
  ctx.lineTo(heading[0]-12*Math.cos(angle-.55),heading[1]-12*Math.sin(angle-.55));
  ctx.lineTo(heading[0]-12*Math.cos(angle+.55),heading[1]-12*Math.sin(angle+.55));
  ctx.closePath();ctx.strokeStyle='#10151b';ctx.lineWidth=3;ctx.stroke();
  ctx.fillStyle='#ff454f';ctx.fill();
  ctx.beginPath();ctx.arc(...center,5,0,Math.PI*2);ctx.stroke();ctx.fill();ctx.restore();
}
function mapPose(){return Array.isArray(slamInfo.pose)?slamInfo.pose:[robotPosition[0],robotPosition[1],robotYaw];}
const NAV_ACTIVE=['planning','following','blocked'];
let navFlash=null,navActionAt=0;
// Show a short message right on the map (failures were easy to miss in the footer).
function flashNav(text){navFlash={text,until:performance.now()+5000};updateNavUI();}
function updateNavUI() {
  const s=navState.state||'idle',active=NAV_ACTIVE.includes(s);
  $('nav-cancel').hidden=!active;
  if(navFlash&&performance.now()<navFlash.until&&!active){
    const el=$('nav-state');el.hidden=false;el.textContent=navFlash.text;el.className='nav-state failed';return;
  }
  const text={planning:'规划路径中…',
    following:navState.message||`导航中 · 剩余 ${navState.remaining!=null?Number(navState.remaining).toFixed(1):'—'} m`,
    blocked:navState.message||'前方受阻',arrived:navState.message||'已到达目标',
    failed:'导航失败：'+(navState.message||''),canceled:navState.message||'已取消导航'}[s];
  const el=$('nav-state');el.hidden=!text;el.textContent=text||'';el.className='nav-state '+s;
}
function setGoalTool(on) {
  goalTool=on;goalDraft=null;
  $('nav-goal').classList.toggle('active',on);canvas.classList.toggle('goal-mode',on);
  $('cloud-help').textContent=on?'按下设置目标位置 · 拖动设置朝向 · 松开发送 · Esc 取消':'左键旋转 · 右键平移 · 滚轮缩放';
}
$('nav-goal').onclick=()=>{
  if(goalTool){setGoalTool(false);return;}
  if(!connected){flashNav('请先连接机器人');return;}
  if(!armed){flashNav('请先在中控台点「控制已锁定」解锁，再设置导航目标');return;}
  if(!['normal','mcf'].includes(mode)){flashNav(`导航需要普通或 MCF 运动模式（当前 ${mode}）`);return;}
  navFlash=null;setGoalTool(true);
};
$('nav-cancel').onclick=()=>send({action:'nav_cancel'});
$('map-reset').onclick=()=>{if(confirm('清空本次建立的地图？正在进行的导航会被取消。'))send({action:'map_reset'});};
setInterval(drawCloud,80);
// The follow-view canvas belongs exclusively to Three.js. Do not call
// getContext('2d') here: a canvas cannot switch from a 2D context to WebGL.
window.addEventListener('go2-scene-ready',()=>{
  window.go2Scene.update(motorStates,robotPosition,robotQuaternion);
  if(latestVoxel) window.go2Scene.voxelMap.processCompressed(latestVoxel);
});
window.addEventListener('go2-model-error',event=>{
  $('fp-status').textContent='机器人模型加载失败';notice(event.detail,true);
});
$('toggle-3d-view').onclick=()=>window.go2Scene?.toggleView();
function renderJointList(){
  const names=['FR hip','FR thigh','FR calf','FL hip','FL thigh','FL calf','RR hip','RR thigh','RR calf','RL hip','RL thigh','RL calf'];
  if(!$('joint-list')) return;
  $('joint-count').textContent=`${motorStates.length} / 12`;
  $('joint-list').innerHTML=motorStates.length?motorStates.map((m,i)=>`<div class="joint-row"><span>${names[i]}</span><b>${(Number(m.q||0)*180/Math.PI).toFixed(1)}°</b></div>`).join(''):'<div class="joint-empty">等待 lowstate 数据</div>';
}
canvas.oncontextmenu=e=>e.preventDefault();
function canvasPoint(e){const rect=canvas.getBoundingClientRect();return [e.clientX-rect.left,e.clientY-rect.top,rect.width,rect.height];}
canvas.onpointerdown=e=>{
  if(drag || goalDraft || ![0,2].includes(e.button)) return;
  e.preventDefault();
  if(goalTool&&e.button===0){
    const [sx,sy,w,h]=canvasPoint(e),p=unproject(sx,sy,w,h);
    goalDraft={id:e.pointerId,start:p,end:p,sx,sy,moved:false};
    canvas.setPointerCapture(e.pointerId);return;
  }
  drag={x:e.clientX,y:e.clientY,id:e.pointerId,pan:e.button===2};
  canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing';
};
canvas.onpointermove=e=>{
  if(goalDraft&&goalDraft.id===e.pointerId){
    const [sx,sy,w,h]=canvasPoint(e);goalDraft.end=unproject(sx,sy,w,h);
    if(Math.hypot(sx-goalDraft.sx,sy-goalDraft.sy)>10)goalDraft.moved=true;
    return;
  }
  if(!drag || drag.id!==e.pointerId) return;
  const dx=e.clientX-drag.x,dy=e.clientY-drag.y;
  if(drag.pan){panX+=dx;panY+=dy;}
  else{yaw+=dx*.008;pitch=Math.max(.15,Math.min(1.5,pitch+dy*.008));}
  drag.x=e.clientX;drag.y=e.clientY;
};
canvas.onpointerup=canvas.onpointercancel=canvas.onlostpointercapture=e=>{
  if(goalDraft&&goalDraft.id===e.pointerId){
    const g=goalDraft;goalDraft=null;
    if(canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
    if(e.type!=='pointerup') return;
    const [x,y]=g.start;
    // 拖动给出朝向；单击则朝向「从机器狗指向目标」的方向
    const theta=g.moved?Math.atan2(g.end[1]-y,g.end[0]-x):Math.atan2(y-mapPose()[1],x-mapPose()[0]);
    setGoalTool(false);
    navActionAt=performance.now();
    send({action:'nav_goal',x:Number(x.toFixed(3)),y:Number(y.toFixed(3)),yaw:Number(theta.toFixed(4))});
    return;
  }
  if(!drag || drag.id!==e.pointerId) return;
  drag=null;canvas.style.cursor='';
  if(canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
};
canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(5,Math.min(200,zoom*Math.exp(-e.deltaY*.001)));};
$('reset-view').onclick=()=>{yaw=DEFAULT_VIEW.yaw;pitch=DEFAULT_VIEW.pitch;zoom=DEFAULT_VIEW.zoom;panX=panY=0;};
updateControls();openSocket();
