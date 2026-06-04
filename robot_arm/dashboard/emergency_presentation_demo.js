/*
 * Emergency presentation demo.
 *
 * Frontend-only and removable by design:
 * 1. Delete this file.
 * 2. Remove the emergency_presentation_demo.js script tag from sambo_robot_arm_dashboard.html.
 *
 * This module never sends motor commands and never modifies the Raspberry Pi bridge.
 */
(function(){
  "use strict";

  const ITEM_LABELS={BOLT:"볼트",NUT:"너트",NONE:"대기"};
  const BIN_LABELS={BOLT:"볼트 통",NUT:"너트 통",NONE:"대기"};
  const PHASE_LABELS={
    TEACHING:"티칭 경로 준비",
    SEARCH:"탐색 모션",
    CAMERA_FOUND:"그리퍼 카메라 검출",
    CLASSIFY:"AI 판정",
    APPROACH:"접근",
    GRASP:"그리퍼 집기",
    LIFT:"들어 올림",
    CARRY:"분류 위치 이동",
    PLACE:"통 투입",
    HOME:"HOME 복귀"
  };
  const DEMO_STEP_MS=1450;
  const MOTION_MS=1180;
  const SORT_POSITIONS={
    NONE:{x:0,y:.08,z:1.08},
    SEARCH_LEFT:{x:-.38,y:.18,z:1.08},
    SEARCH_CENTER:{x:0,y:.18,z:1.08},
    SEARCH_RIGHT:{x:.38,y:.18,z:1.08},
    BOLT_FOUND:{x:-.18,y:.2,z:1.0},
    NUT_FOUND:{x:.18,y:.2,z:1.0},
    PRE_GRASP:{x:-.12,y:.48,z:.68},
    GRASP:{x:-.18,y:.66,z:.42},
    LIFT:{x:-.18,y:1.02,z:.36},
    CARRY_BOLT:{x:-.92,y:.92,z:.62},
    CARRY_NUT:{x:.92,y:.92,z:.62},
    PLACE_BOLT:{x:-1.68,y:.62,z:.82},
    PLACE_NUT:{x:1.68,y:.62,z:.82},
    HOME:{x:0,y:.12,z:1.08}
  };

  const DEMO_STEPS=[
    {
      mode:"TEACHING",step:"TEACHING",progress:4,target:"NONE",item:"NONE",bin:"NONE",phase:"TEACHING",motion:"NONE",tof:90,gripper:"OPEN",
      message:"티칭 모드 시작: MPU 3개 센서 자세를 로봇 목표각으로 변환",level:"warn",
      joints:[180,360,0,360,180,180],correction:[0,0],confidence:.86,stable:1,counts:{BOLT:0,NUT:0},
      human:{shoulder_motion:35,elbow_bend:64,wrist_direction:8,hand_delta_x:4,hand_delta_y:-3}
    },
    {
      mode:"TEACHING",step:"TEACHING",progress:10,target:"NONE",item:"NONE",bin:"NONE",phase:"TEACHING",motion:"NONE",tof:88,gripper:"OPEN",
      message:"티칭 경로 저장 완료: SCAN-PICK-PLACE 순서 준비",level:"ok",
      joints:[186,350,10,350,184,178],correction:[0,0],confidence:.92,stable:2,counts:{BOLT:0,NUT:0},
      human:{shoulder_motion:44,elbow_bend:78,wrist_direction:15,hand_delta_x:12,hand_delta_y:-8}
    },
    {
      mode:"AUTO_RUN",step:"SCAN",progress:16,target:"NONE",item:"NONE",bin:"NONE",phase:"SEARCH",motion:"SEARCH_LEFT",tof:82,gripper:"OPEN",
      message:"탐색 구간 모션: 작업대 왼쪽을 스캔",level:"info",
      joints:[154,350,10,350,182,180],correction:[0,0],confidence:.30,stable:0,counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"SCAN",progress:22,target:"NONE",item:"NONE",bin:"NONE",phase:"SEARCH",motion:"SEARCH_CENTER",tof:78,gripper:"OPEN",
      message:"탐색 구간 모션: 중앙 영역 스캔",level:"info",
      joints:[180,348,12,348,180,180],correction:[0,0],confidence:.34,stable:0,counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"SCAN",progress:28,target:"NONE",item:"NONE",bin:"NONE",phase:"SEARCH",motion:"SEARCH_RIGHT",tof:74,gripper:"OPEN",
      message:"탐색 구간 모션: 오른쪽 영역 스캔",level:"info",
      joints:[206,350,10,350,178,182],correction:[0,0],confidence:.36,stable:0,counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"DETECT",progress:34,target:"BOLT",item:"BOLT",bin:"NONE",phase:"CAMERA_FOUND",motion:"BOLT_FOUND",tof:62,gripper:"OPEN",
      message:"그리퍼 카메라에 후보 표시: 볼트 형태 검출",level:"info",
      joints:[170,340,20,340,176,184],correction:[3.2,-2.1],confidence:.76,stable:1,bbox:{x:304,y:198,w:112,h:78},counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"DETECT",progress:40,target:"BOLT",item:"BOLT",bin:"BOLT",phase:"CLASSIFY",motion:"BOLT_FOUND",tof:55,gripper:"OPEN",
      message:"YOLO/Hailo 판정: 볼트 95%, 볼트 통 경로 선택",level:"ok",
      joints:[170,336,24,336,176,184],correction:[1.8,-1.2],confidence:.95,stable:3,bbox:{x:312,y:202,w:106,h:74},counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"PRE_GRASP",progress:48,target:"BOLT",item:"BOLT",bin:"BOLT",phase:"APPROACH",motion:"PRE_GRASP",tof:36,gripper:"OPEN",
      message:"가까이 접근: ToF 거리와 위치 보정값으로 집기 지점 보정",level:"info",
      joints:[166,326,34,326,174,186],correction:[.9,-.5],confidence:.96,stable:3,bbox:{x:318,y:206,w:98,h:70},counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"GRASP",progress:56,target:"BOLT",item:"BOLT",bin:"BOLT",phase:"GRASP",motion:"GRASP",tof:23,gripper:"CLOSED",
      message:"그리퍼 닫힘: 볼트 집기 완료",level:"ok",
      joints:[166,322,38,322,174,186],correction:[.4,-.2],confidence:.96,stable:3,bbox:{x:322,y:208,w:92,h:66},counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"LIFT",progress:62,target:"BOLT",item:"BOLT",bin:"BOLT",phase:"LIFT",motion:"LIFT",tof:34,gripper:"CLOSED",
      message:"작업물 들어 올림: 그리퍼에 볼트 유지",level:"info",
      joints:[160,332,28,332,182,188],correction:[.2,-.1],confidence:.95,stable:3,bbox:{x:322,y:208,w:92,h:66},counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"CARRY",progress:68,target:"BOLT",item:"BOLT",bin:"BOLT",phase:"CARRY",motion:"CARRY_BOLT",tof:46,gripper:"CLOSED",
      message:"볼트 통으로 이동: CAN 0x100 목표각, STM32 0x201 피드백 표시",level:"info",
      joints:[132,344,16,344,188,190],correction:[0,0],confidence:.94,stable:3,bbox:{x:322,y:208,w:92,h:66},counts:{BOLT:0,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"PLACE",progress:74,target:"BOLT",item:"BOLT",bin:"BOLT",phase:"PLACE",motion:"PLACE_BOLT",tof:58,gripper:"OPEN",
      message:"볼트 통 투입 완료: 볼트 1개",level:"ok",
      joints:[120,354,6,354,188,180],correction:[0,0],confidence:.94,stable:3,bbox:{x:322,y:208,w:92,h:66},counts:{BOLT:1,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"SCAN",progress:79,target:"NONE",item:"NONE",bin:"NONE",phase:"SEARCH",motion:"SEARCH_CENTER",tof:78,gripper:"OPEN",
      message:"다음 작업물 탐색: 다시 스캔 자세로 복귀",level:"info",
      joints:[180,348,12,348,180,180],correction:[0,0],confidence:.38,stable:0,counts:{BOLT:1,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"DETECT",progress:84,target:"NUT",item:"NUT",bin:"NONE",phase:"CAMERA_FOUND",motion:"NUT_FOUND",tof:60,gripper:"OPEN",
      message:"그리퍼 카메라에 후보 표시: 너트 형태 검출",level:"info",
      joints:[194,338,22,338,178,176],correction:[-2.4,1.7],confidence:.78,stable:1,bbox:{x:246,y:198,w:88,h:86},counts:{BOLT:1,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"DETECT",progress:88,target:"NUT",item:"NUT",bin:"NUT",phase:"CLASSIFY",motion:"NUT_FOUND",tof:52,gripper:"OPEN",
      message:"YOLO/Hailo 판정: 너트 94%, 너트 통 경로 선택",level:"ok",
      joints:[194,334,26,334,178,176],correction:[-1.2,.8],confidence:.94,stable:3,bbox:{x:252,y:202,w:82,h:80},counts:{BOLT:1,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"PRE_GRASP",progress:91,target:"NUT",item:"NUT",bin:"NUT",phase:"APPROACH",motion:"PRE_GRASP",tof:35,gripper:"OPEN",
      message:"가까이 접근: 너트 중심점으로 집기 보정",level:"info",
      joints:[190,326,34,326,176,174],correction:[-.6,.4],confidence:.94,stable:3,bbox:{x:258,y:206,w:78,h:76},counts:{BOLT:1,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"GRASP",progress:94,target:"NUT",item:"NUT",bin:"NUT",phase:"GRASP",motion:"GRASP",tof:22,gripper:"CLOSED",
      message:"그리퍼 닫힘: 너트 집기 완료",level:"ok",
      joints:[190,322,38,322,176,174],correction:[-.3,.2],confidence:.94,stable:3,bbox:{x:260,y:208,w:74,h:72},counts:{BOLT:1,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"CARRY",progress:97,target:"NUT",item:"NUT",bin:"NUT",phase:"CARRY",motion:"CARRY_NUT",tof:43,gripper:"CLOSED",
      message:"너트 통으로 이동: 분류 위치까지 운반",level:"info",
      joints:[226,344,16,344,188,172],correction:[0,0],confidence:.93,stable:3,bbox:{x:260,y:208,w:74,h:72},counts:{BOLT:1,NUT:0}
    },
    {
      mode:"AUTO_RUN",step:"PLACE",progress:99,target:"NUT",item:"NUT",bin:"NUT",phase:"PLACE",motion:"PLACE_NUT",tof:60,gripper:"OPEN",
      message:"너트 통 투입 완료: 볼트 1개, 너트 1개",level:"ok",
      joints:[238,354,6,354,188,180],correction:[0,0],confidence:.93,stable:3,bbox:{x:260,y:208,w:74,h:72},counts:{BOLT:1,NUT:1}
    },
    {
      mode:"AUTO_RUN",step:"HOME",progress:100,target:"NONE",item:"NONE",bin:"NONE",phase:"HOME",motion:"HOME",tof:72,gripper:"OPEN",
      message:"HOME 복귀: 티칭-탐색-판정-집기-분류 시연 완료",level:"ok",
      joints:[180,360,0,360,180,180],correction:[0,0],confidence:.90,stable:3,counts:{BOLT:1,NUT:1}
    }
  ];

  const runtime={
    active:false,
    done:false,
    index:0,
    timer:null,
    motionRaf:null,
    snapshot:null,
    currentStep:DEMO_STEPS[0],
    originalHandlePayload:null,
    originalDemoTick:null,
    originalDrawGripper:null
  };

  function ready(){
    return typeof state!=="undefined"&&typeof setMode==="function"&&typeof renderAll==="function";
  }

  function clone(value){
    return JSON.parse(JSON.stringify(value));
  }

  function takeSnapshot(){
    runtime.snapshot={
      mode:state.mode,
      demo:state.demo,
      wpList:clone(state.wpList),
      events:clone(state.events),
      logCount:state.logCount,
      system:clone(state.system),
      robot:clone(state.robot),
      human_pose:clone(state.human_pose),
      gripper:clone(state.gripper),
      vision:clone(state.vision),
      path:clone(state.path),
      precheck:clone(state.precheck),
      can_health:clone(state.can_health),
      calibration:clone(state.calibration),
      alerts:clone(state.alerts),
      digital_twin:clone(state.digital_twin)
    };
  }

  function restoreSnapshot(){
    const s=runtime.snapshot;
    if(!s)return;
    state.mode=s.mode;
    state.demo=s.demo;
    state.wpList=s.wpList;
    state.events=s.events;
    state.logCount=s.logCount;
    Object.assign(state.system,s.system);
    Object.assign(state.robot,s.robot);
    Object.assign(state.human_pose,s.human_pose);
    Object.assign(state.gripper,s.gripper);
    Object.assign(state.vision,s.vision);
    Object.assign(state.path,s.path);
    Object.assign(state.precheck,s.precheck);
    Object.assign(state.can_health,s.can_health);
    Object.assign(state.calibration,s.calibration);
    state.alerts=s.alerts;
    Object.assign(state.digital_twin,s.digital_twin);
  }

  function installPayloadGuard(){
    if(runtime.originalHandlePayload||typeof window.handlePayload!=="function")return;
    runtime.originalHandlePayload=window.handlePayload;
    window.handlePayload=function(payload){
      if(runtime.active||runtime.done)return;
      return runtime.originalHandlePayload(payload);
    };
  }

  function removePayloadGuard(){
    if(runtime.originalHandlePayload){
      window.handlePayload=runtime.originalHandlePayload;
      runtime.originalHandlePayload=null;
    }
  }

  function installDemoTickGuard(){
    if(runtime.originalDemoTick||typeof window.demoTick!=="function")return;
    runtime.originalDemoTick=window.demoTick;
    const guarded=function(now){
      if(runtime.active||runtime.done)return;
      return runtime.originalDemoTick(now);
    };
    window.demoTick=guarded;
    try{demoTick=guarded}catch(_){}
  }

  function removeDemoTickGuard(){
    if(!runtime.originalDemoTick)return;
    window.demoTick=runtime.originalDemoTick;
    try{demoTick=runtime.originalDemoTick}catch(_){}
    runtime.originalDemoTick=null;
  }

  function installDrawGripperGuard(){
    if(runtime.originalDrawGripper||typeof window.drawGripper!=="function")return;
    runtime.originalDrawGripper=window.drawGripper;
    const guarded=function(){
      if((runtime.active||runtime.done)&&state.mode==="AUTO_RUN")return drawEmergencyGripper();
      return runtime.originalDrawGripper();
    };
    window.drawGripper=guarded;
    try{drawGripper=guarded}catch(_){}
  }

  function removeDrawGripperGuard(){
    if(!runtime.originalDrawGripper)return;
    window.drawGripper=runtime.originalDrawGripper;
    try{drawGripper=runtime.originalDrawGripper}catch(_){}
    runtime.originalDrawGripper=null;
  }

  function smooth(t){
    return t<.5?2*t*t:1-Math.pow(-2*t+2,2)/2;
  }

  function animateJoints(targetAngles){
    if(runtime.motionRaf)cancelAnimationFrame(runtime.motionRaf);
    const from=state.robot.joints.map(j=>Number(j.current||0));
    const target=targetAngles.map(Number);
    const start=performance.now();
    state.robot.joints.forEach((joint,index)=>{
      joint.target=target[index];
      joint.status=joint.load>=82?"WARN":"OK";
    });
    function frame(now){
      const t=smooth(Math.min(1,(now-start)/MOTION_MS));
      state.robot.joints.forEach((joint,index)=>{
        joint.current=from[index]+(target[index]-from[index])*t;
      });
      if(typeof draw3D==="function")draw3D();
      if(typeof drawTopDown==="function")drawTopDown();
      if(typeof renderRightPanel==="function")renderRightPanel();
      syncThreeSortMarker();
      if(t<1&&runtime.active)runtime.motionRaf=requestAnimationFrame(frame);
    }
    runtime.motionRaf=requestAnimationFrame(frame);
  }

  function applyStep(step,index){
    runtime.currentStep=step;
    state.demo=true;
    if(state.mode!==step.mode)setMode(step.mode,true);
    else state.mode=step.mode;
    state.system.server_connected=false;
    state.system.can_status="OK";
    state.system.camera_status="OK";
    state.system.ai_status="RUNNING";
    state.path.current_step=step.step;
    state.path.progress_percent=step.progress;
    state.vision.target=step.target;
    state.vision.confidence=step.confidence;
    state.vision.stable_frames=step.stable;
    state.vision.correction.dx_mm=step.correction[0];
    state.vision.correction.dy_mm=step.correction[1];
    if(step.bbox)Object.assign(state.vision.bbox,step.bbox);
    state.gripper.state=step.gripper;
    state.gripper.sg90_angle=step.gripper==="OPEN"?90:30;
    state.gripper.tof_mm=step.tof;
    state.gripper.object_detected=step.target!=="NONE";
    state.gripper.pick_ready=step.phase==="GRASP";
    if(step.human){
      Object.assign(state.human_pose,step.human);
      state.human_pose.confidence=step.confidence;
      state.human_pose.status="STABLE";
    }
    state.can_health.received+=12+index;
    state.can_health.last_rx_ms=18+index;
    state.can_health.delay_ms=14+index;
    state.can_health.errors=0;
    state.digital_twin.cmd_id="0x100";
    state.digital_twin.telemetry_id="0x201";
    state.digital_twin.current_id="0x202";
    state.digital_twin.latency_ms=state.can_health.last_rx_ms;
    state.digital_twin.last_cmd=`EMERGENCY_DEMO ${PHASE_LABELS[step.phase]} ${ITEM_LABELS[step.item]} -> ${BIN_LABELS[step.bin]}`;
    state.digital_twin.last_ack=`STM 0x201 demo feedback ${index+1}/${DEMO_STEPS.length}`;
    state.robot.joints.forEach((joint,jointIndex)=>{
      const carrying=["GRASP","LIFT","CARRY","PLACE"].includes(step.phase);
      const tempBase=[41,44,47,40,39,38][jointIndex];
      const loadBase=[34,48,carrying?68:55,31,29,27][jointIndex];
      joint.temp=tempBase+(carrying?2:0);
      joint.load=loadBase;
      joint.status=joint.load>=82?"WARN":"OK";
    });
    animateJoints(step.joints);
    if(typeof addLog==="function")addLog(`[긴급시연] ${step.message}`,step.level);
    refresh();
  }

  function refresh(){
    if(typeof renderAll==="function")renderAll();
    if(typeof drawHumanPose==="function")drawHumanPose();
    if(typeof drawGripper==="function")drawGripper();
    if(typeof draw3D==="function")draw3D();
    if(typeof drawTopDown==="function")drawTopDown();
    syncThreeSortMarker();
    updateButton();
    updateOverlay();
    updateSortPanel();
  }

  function nextStep(){
    if(!runtime.active)return;
    applyStep(DEMO_STEPS[runtime.index],runtime.index);
    runtime.index+=1;
    if(runtime.index>=DEMO_STEPS.length){
      runtime.active=false;
      runtime.done=true;
      clearTimeout(runtime.timer);
      runtime.timer=null;
      removePayloadGuard();
      updateButton();
      updateOverlay("완료");
      updateSortPanel();
      return;
    }
    runtime.timer=setTimeout(nextStep,DEMO_STEP_MS);
  }

  function start(){
    if(!ready())return;
    if(document.getElementById("splashScreen")&&typeof dismissSplash==="function")dismissSplash();
    takeSnapshot();
    installPayloadGuard();
    installDemoTickGuard();
    installDrawGripperGuard();
    runtime.active=true;
    runtime.done=false;
    runtime.index=0;
    runtime.currentStep=DEMO_STEPS[0];
    setMode("TEACHING",true);
    window.scrollTo({top:0,behavior:"smooth"});
    if(typeof addLog==="function")addLog("[긴급시연] 티칭부터 시작하는 발표용 분류 시연 - 실제 백엔드 명령 전송 없음","warn");
    clearTimeout(runtime.timer);
    nextStep();
  }

  function stop(restore=true){
    clearTimeout(runtime.timer);
    runtime.timer=null;
    if(runtime.motionRaf)cancelAnimationFrame(runtime.motionRaf);
    runtime.motionRaf=null;
    runtime.active=false;
    runtime.done=false;
    removePayloadGuard();
    removeDemoTickGuard();
    removeDrawGripperGuard();
    if(restore)restoreSnapshot();
    hideThreeSortMarker();
    if(typeof addLog==="function")addLog("[긴급시연] 시연 상태 복구 완료","info");
    refresh();
  }

  function toggle(){
    if(runtime.active){stop(true);return}
    if(runtime.done){stop(true);return}
    start();
  }

  function injectStyle(){
    if(document.getElementById("emergencyDemoStyle"))return;
    const style=document.createElement("style");
    style.id="emergencyDemoStyle";
    style.textContent=`
      .emergency-demo-btn{color:#06101b;background:#f7c948;border-color:#f7c948;font-weight:900}
      .emergency-demo-btn.running{color:#fff;background:#b91c1c;border-color:#ff7777;box-shadow:0 0 18px rgba(255,91,91,.25)}
      .emergency-demo-overlay{position:fixed;left:104px;bottom:18px;z-index:70;display:none;min-width:320px;padding:10px 12px;border:1px solid rgba(247,201,72,.55);border-radius:7px;color:#f7e6a0;background:rgba(5,12,20,.9);box-shadow:0 14px 34px rgba(0,0,0,.38);font-size:11px;line-height:1.55}
      .emergency-demo-overlay b{display:block;color:#f7c948;font-size:13px;margin-bottom:2px}
      .emergency-demo-overlay.show{display:block}
      .emergency-sort-panel{position:fixed;left:104px;bottom:114px;z-index:70;display:none;width:min(440px,calc(100vw - 128px));padding:12px;border:1px solid rgba(32,213,232,.42);border-radius:8px;background:rgba(5,12,20,.92);box-shadow:0 16px 40px rgba(0,0,0,.36);color:#dcecff}
      .emergency-sort-panel.show{display:block}
      .emergency-sort-title{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:10px;font-weight:900}
      .emergency-sort-title span{color:#f7c948;font-size:11px}
      .emergency-sort-route{display:grid;grid-template-columns:1fr auto 1fr;align-items:center;gap:8px;margin-bottom:10px}
      .emergency-sort-node{border:1px solid rgba(32,213,232,.35);background:rgba(12,29,48,.9);border-radius:7px;padding:9px 10px;min-width:0}
      .emergency-sort-node small{display:block;color:#91a8bb;font-size:10px;margin-bottom:4px}
      .emergency-sort-node b{display:block;font-size:17px;color:#fff;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
      .emergency-sort-arrow{color:#20d5e8;font-weight:900}
      .emergency-sort-bins{display:grid;grid-template-columns:1fr 1fr;gap:8px}
      .emergency-sort-bin{border-radius:7px;padding:9px 10px;border:1px solid rgba(255,255,255,.12);background:rgba(255,255,255,.04)}
      .emergency-sort-bin.active{box-shadow:0 0 18px rgba(32,213,232,.22)}
      .emergency-sort-bin.bolt.active{border-color:#4ade80}
      .emergency-sort-bin.nut.active{border-color:#60a5fa}
      .emergency-sort-bin small{display:block;color:#91a8bb;font-size:10px}
      .emergency-sort-bin b{font-size:22px;color:#fff}
      @media (max-width:900px){
        .emergency-demo-overlay{left:14px;right:14px;bottom:14px;min-width:0}
        .emergency-sort-panel{left:14px;right:14px;bottom:106px;width:auto}
      }
    `;
    document.head.appendChild(style);
  }

  function injectButton(){
    if(document.getElementById("emergencyDemoBtn"))return;
    const spacer=document.querySelector(".toolbar-spacer");
    if(!spacer)return;
    const button=document.createElement("button");
    button.id="emergencyDemoBtn";
    button.type="button";
    button.className="tool-btn emergency-demo-btn";
    button.textContent="긴급 시연";
    button.title="티칭부터 탐색, 카메라 검출, AI 판정, 집기, 볼트/너트 통 분류까지 보여주는 프론트 전용 발표 시연입니다.";
    button.addEventListener("click",toggle);
    spacer.insertAdjacentElement("afterend",button);
  }

  function injectOverlay(){
    if(document.getElementById("emergencyDemoOverlay"))return;
    const overlay=document.createElement("div");
    overlay.id="emergencyDemoOverlay";
    overlay.className="emergency-demo-overlay";
    overlay.innerHTML="<b>긴급 발표 시연</b><span>티칭부터 자동 분류까지 보여주는 프론트 전용 데모입니다. 실제 CAN/모터 명령은 전송하지 않습니다.</span>";
    document.body.appendChild(overlay);
  }

  function injectSortPanel(){
    if(document.getElementById("emergencySortPanel"))return;
    const panel=document.createElement("div");
    panel.id="emergencySortPanel";
    panel.className="emergency-sort-panel";
    document.body.appendChild(panel);
  }

  function updateButton(){
    const button=document.getElementById("emergencyDemoBtn");
    if(!button)return;
    button.classList.toggle("running",runtime.active);
    button.textContent=runtime.active?"시연 중지":runtime.done?"시연 리셋":"긴급 시연";
  }

  function updateOverlay(status){
    const overlay=document.getElementById("emergencyDemoOverlay");
    if(!overlay)return;
    const step=runtime.currentStep||DEMO_STEPS[0];
    overlay.classList.toggle("show",runtime.active||runtime.done);
    overlay.innerHTML=`<b>긴급 발표 시연 ${status||""}</b><span>${PHASE_LABELS[step.phase]} · ${step.message}<br>프론트 전용 데모라 실제 CAN/모터 명령은 전송하지 않습니다.</span>`;
  }

  function updateSortPanel(){
    const panel=document.getElementById("emergencySortPanel");
    if(!panel)return;
    const step=runtime.currentStep||DEMO_STEPS[0];
    const counts=step.counts||{BOLT:0,NUT:0};
    const item=runtime.done?"분류 완료":(step.phase==="SEARCH"||step.phase==="TEACHING"?"탐색 전":ITEM_LABELS[step.item]||ITEM_LABELS.NONE);
    const bin=runtime.done?"볼트 통 + 너트 통":(step.bin==="NONE"?"판정 대기":BIN_LABELS[step.bin]||BIN_LABELS.NONE);
    panel.classList.toggle("show",runtime.active||runtime.done);
    panel.innerHTML=`
      <div class="emergency-sort-title">티칭 → 탐색 → AI 판정 → 집기 → 분류 <span>${PHASE_LABELS[step.phase]}</span></div>
      <div class="emergency-sort-route">
        <div class="emergency-sort-node"><small>그리퍼 카메라 / AI</small><b>${item}</b></div>
        <div class="emergency-sort-arrow">→</div>
        <div class="emergency-sort-node"><small>선택된 투입 위치</small><b>${bin}</b></div>
      </div>
      <div class="emergency-sort-bins">
        <div class="emergency-sort-bin bolt ${step.bin==="BOLT"?"active":""}"><small>볼트 통</small><b>${counts.BOLT}</b> 개</div>
        <div class="emergency-sort-bin nut ${step.bin==="NUT"?"active":""}"><small>너트 통</small><b>${counts.NUT}</b> 개</div>
      </div>
    `;
  }

  function drawEmergencyGripper(){
    const canvas=document.getElementById("gripperCanvas");
    if(!canvas||state.mode!=="AUTO_RUN")return;
    const step=runtime.currentStep||DEMO_STEPS[0];
    const box=step.bbox||{x:300,y:200,w:100,h:76};
    const W=canvas.clientWidth||canvas.width||640;
    const H=canvas.clientHeight||canvas.height||360;
    const dpr=window.devicePixelRatio||1;
    if(canvas.width!==Math.floor(W*dpr)||canvas.height!==Math.floor(H*dpr)){
      canvas.width=Math.floor(W*dpr);
      canvas.height=Math.floor(H*dpr);
    }
    const ctx=canvas.getContext("2d");
    ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.clearRect(0,0,W,H);
    const bg=ctx.createLinearGradient(0,0,W,H);
    bg.addColorStop(0,"#3b4651");
    bg.addColorStop(.52,"#5a6672");
    bg.addColorStop(1,"#2d3742");
    ctx.fillStyle=bg;
    ctx.fillRect(0,0,W,H);
    for(let i=0;i<20;i++){
      ctx.strokeStyle="rgba(255,255,255,.055)";
      ctx.beginPath();
      ctx.moveTo((i*83)%W,(i*47)%H);
      ctx.lineTo(((i*83)%W)+55,((i*47)%H)+8);
      ctx.stroke();
    }
    drawNut(ctx,W*.28,H*.43,35);
    drawBolt(ctx,W*.55,H*.48,30);
    drawWasher(ctx,W*.7,H*.3,28);
    drawNut(ctx,W*.72,H*.69,25);
    const sx=W/640,sy=H/480;
    const drawCandidate=(label,color,scale=1)=>{
      drawBBox(ctx,box.x*sx,box.y*sy,box.w*sx*scale,box.h*sy*scale,label,color);
    };
    if(step.phase==="SEARCH"){
      ctx.strokeStyle="#20d5e8";
      ctx.setLineDash([7,6]);
      ctx.lineWidth=2;
      ctx.strokeRect(W*.22,H*.20,W*.56,H*.52);
      ctx.setLineDash([]);
      ctx.fillStyle="#20d5e8";
      ctx.font="bold 12px Malgun Gothic, Segoe UI";
      ctx.fillText("탐색 중: 후보 없음",W*.22+8,H*.20+20);
    }else if(step.phase==="CAMERA_FOUND"){
      drawCandidate(`${ITEM_LABELS[step.item]} 후보 ${(step.confidence*100).toFixed(0)}%`,"#f7c948",1.04);
    }else if(["CLASSIFY","APPROACH","GRASP","LIFT","CARRY","PLACE"].includes(step.phase)){
      drawCandidate(`${ITEM_LABELS[step.item]} ${(step.confidence*100).toFixed(0)}%`,"#4ade80");
      ctx.strokeStyle=step.item==="BOLT"?"#4ade80":"#60a5fa";
      ctx.beginPath();
      ctx.arc((box.x+box.w/2)*sx,(box.y+box.h/2)*sy,16,0,Math.PI*2);
      ctx.stroke();
    }
    const grad=ctx.createRadialGradient(W/2,H/2,H*.1,W/2,H/2,H*.8);
    grad.addColorStop(0,"rgba(0,0,0,0)");
    grad.addColorStop(1,"rgba(0,0,0,.48)");
    ctx.fillStyle=grad;
    ctx.fillRect(0,0,W,H);
    ctx.fillStyle="rgba(5,12,20,.82)";
    ctx.fillRect(0,H-34,W,34);
    ctx.font="11px Consolas";
    ctx.fillStyle="#20d5e8";
    ctx.fillText(`TOF ${Number(state.gripper.tof_mm||0).toFixed(1)} mm`,12,H-13);
    ctx.fillStyle="#f7c948";
    ctx.fillText(`${PHASE_LABELS[step.phase]} · X ${signed(state.vision.correction.dx_mm)} / Y ${signed(state.vision.correction.dy_mm)} mm`,W*.28,H-13);
    ctx.fillStyle="#f7c948";
    ctx.fillText("PRESENTATION DEMO",W-150,H-13);
  }

  function makeThreeLabel(T,text,color){
    const canvas=document.createElement("canvas"),ctx=canvas.getContext("2d");
    canvas.width=360;canvas.height=92;
    ctx.fillStyle="rgba(5,12,20,.82)";
    ctx.fillRect(0,0,360,92);
    ctx.strokeStyle=color;
    ctx.lineWidth=4;
    ctx.strokeRect(2,2,356,88);
    ctx.fillStyle="#fff";
    ctx.font="bold 26px Malgun Gothic, Segoe UI, sans-serif";
    ctx.textAlign="center";
    ctx.fillText(text,180,38);
    ctx.fillStyle=color;
    ctx.font="bold 18px Malgun Gothic, Segoe UI, sans-serif";
    ctx.fillText("탐색/집기/분류 시연",180,68);
    const texture=new T.CanvasTexture(canvas);
    const sprite=new T.Sprite(new T.SpriteMaterial({map:texture,transparent:true,depthTest:false}));
    sprite.scale.set(.9,.23,1);
    return sprite;
  }

  function ensureThreeSortMarker(){
    if(typeof threeRobot==="undefined"||!threeRobot||!threeRobot.scene||!window.THREE)return null;
    const T=window.THREE;
    if(threeRobot.emergencySortDemo)return threeRobot.emergencySortDemo;
    const group=new T.Group();
    group.name="EmergencyPresentationSortMarker";
    const carrier=new T.Group();
    const metal=new T.MeshStandardMaterial({color:0xe5edf4,metalness:.42,roughness:.46});
    const bolt=new T.Group();
    const shaft=new T.Mesh(new T.CylinderGeometry(.035,.035,.34,18),metal);
    const head=new T.Mesh(new T.CylinderGeometry(.105,.105,.055,6),metal);
    shaft.rotation.z=Math.PI/2;
    head.rotation.z=Math.PI/2;
    head.position.x=-.19;
    bolt.add(shaft,head);
    const nut=new T.Mesh(new T.TorusGeometry(.14,.038,14,6),metal);
    nut.rotation.x=Math.PI/2;
    carrier.add(bolt,nut);
    const glow=new T.Mesh(new T.SphereGeometry(.19,32,16),new T.MeshBasicMaterial({color:0xf7c948,transparent:true,opacity:.18}));
    carrier.add(glow);
    const label=makeThreeLabel(T,"작업물 이동","#f7c948");
    label.position.set(0,.34,0);
    carrier.add(label);
    group.add(carrier);
    threeRobot.scene.add(group);
    threeRobot.emergencySortDemo={group,carrier,bolt,nut,label};
    return threeRobot.emergencySortDemo;
  }

  function markerPosition(step){
    if(step.motion==="CARRY_BOLT")return SORT_POSITIONS.CARRY_BOLT;
    if(step.motion==="CARRY_NUT")return SORT_POSITIONS.CARRY_NUT;
    if(step.motion==="PLACE_BOLT")return SORT_POSITIONS.PLACE_BOLT;
    if(step.motion==="PLACE_NUT")return SORT_POSITIONS.PLACE_NUT;
    return SORT_POSITIONS[step.motion]||SORT_POSITIONS.NONE;
  }

  function syncThreeSortMarker(){
    const marker=ensureThreeSortMarker();
    if(!marker)return;
    const step=runtime.currentStep||DEMO_STEPS[0];
    marker.group.visible=runtime.active||runtime.done;
    if(!marker.group.visible)return;
    const pos=markerPosition(step);
    marker.carrier.position.set(pos.x,pos.y,pos.z);
    marker.carrier.rotation.y+=(step.item==="NUT"?.08:.05);
    marker.bolt.visible=step.item==="BOLT";
    marker.nut.visible=step.item==="NUT";
    marker.label.visible=step.item!=="NONE";
  }

  function hideThreeSortMarker(){
    if(typeof threeRobot!=="undefined"&&threeRobot&&threeRobot.emergencySortDemo){
      threeRobot.emergencySortDemo.group.visible=false;
    }
  }

  function init(){
    injectStyle();
    injectButton();
    injectOverlay();
    injectSortPanel();
    updateButton();
  }

  if(document.readyState==="loading")document.addEventListener("DOMContentLoaded",init);
  else init();

  window.samboEmergencyPresentationDemo={start,stop,toggle};
})();
