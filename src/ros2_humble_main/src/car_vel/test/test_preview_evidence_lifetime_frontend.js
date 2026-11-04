'use strict';
const assert=require('assert');
const fs=require('fs');
const path=require('path');
const vm=require('vm');
const root=path.join(__dirname,'../car_vel/web_frontend');
const nav=fs.readFileSync(path.join(root,'navigation.html'),'utf8');
const inspection=fs.readFileSync(path.join(root,'inspection.html'),'utf8');
function fn(source,name){
  const found=source.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n    }`));
  assert(found,name);return found[0];
}
{
  const ctx={Date:{now:()=>600000},
    threeDRoutePreview:{previewId:'p',fingerprint:'same',atMs:0},
    threeDPreviewFingerprint:()=> 'same',threeDRouteIssue:()=>''};
  vm.createContext(ctx);vm.runInContext(fn(inspection,'currentThreeDPreview'),ctx);
  assert(ctx.currentThreeDPreview(),'inspection preview expired merely from human waiting');
}
{
  const controls={'route-loop-count':{value:'1'},'route-continuous':{checked:false}};
  const ctx={Date:{now:()=>600000},document:{getElementById:id=>controls[id]},
    threeDRoutePreview:{previewId:'p',previewedAtMs:0,mapName:'map',mapLoadId:'load',
      routeId:8,loopCount:1,continuous:false,bodyGeneration:1,localizationGeneration:1,refreshState:''},
    threeDRoutePreviewIssue:()=>'',loadedMapName:'map',currentMapLoadId:'load',routeDraft:{id:8},
    threeDRobotBodyGeneration:1,localizationGeneration:1,singleGoalRefreshState:''};
  vm.createContext(ctx);vm.runInContext(fn(nav,'threeDRoutePreviewMatches')+'\n'+fn(nav,'threeDRoutePreviewFresh'),ctx);
  assert(ctx.threeDRoutePreviewFresh(),'multi-point preview expired merely from waiting');
  ctx.currentMapLoadId='new';
  assert(!ctx.threeDRoutePreviewFresh(),'same-name map reload must still invalidate preview');
}
console.log('PASS: unchanged inspection and route previews survive human waiting; map reload invalidates');
{
  const controls=Object.fromEntries(['three-d-pose-panel','three-d-pose-message','btn-three-d-confirm-initial',
    'btn-three-d-preview-goal','btn-three-d-start-goal','btn-three-d-clear'].map(id=>[id,{}]));
  const ctx={Date:{now:()=>600000},document:{getElementById:id=>controls[id]},
    navigationMapView:'3d',clickMode:'goal',threeDInteractionIssue:()=>'',goalReadinessIssue:()=>'',
    threeDNotice:'',threeDRouteNotice:'',singleGoalRefreshState:'',navigationUiMode:'single',
    loadedMapName:'map',currentMapLoadId:'load',threeDRobotBodyGeneration:1,localizationGeneration:1,
    threeDPoseDraft:{kind:'goal',groundValid:true,busy:false,previewId:'p',previewedAtMs:0,
      mapName:'map',mapLoadId:'load',bodyGeneration:1,localizationGeneration:1},
    navigationPointCloudViewer:{setGroundPoseInteraction(){}},handleThreeDGroundPose(){}};
  ctx.clearThreeDPoseDraft=()=>{ctx.threeDPoseDraft=null;};
  vm.createContext(ctx);vm.runInContext(fn(nav,'updateThreeDPoseUi'),ctx);
  ctx.updateThreeDPoseUi();
  assert.equal(controls['btn-three-d-start-goal'].disabled,false,'single goal preview expired from waiting');
  ctx.goalReadinessIssue=()=> '定位正在复核';ctx.updateThreeDPoseUi();
  assert(ctx.threeDPoseDraft,'temporary readiness loss erased evidence');
  assert.equal(controls['btn-three-d-start-goal'].disabled,true);
  ctx.goalReadinessIssue=()=>'';ctx.threeDRobotBodyGeneration=2;ctx.updateThreeDPoseUi();
  assert.equal(ctx.threeDPoseDraft,null,'changed body generation must invalidate single goal');
}
(async()=>{
  const ctx={inspectionStartPending:true,inspectionStartCancelPending:false,serviceBusy:true,
    taskState:{active:false},UI:{TYPES:{controlInspectionTask:'control'}},
    session:{call:async()=>({success:true,message:'启动已取消'})},
    updateUi(){},log(){},invalidateThreeDPreview(){ctx.invalidated=true;}};
  vm.createContext(ctx);vm.runInContext(fn(inspection,'controlTask'),ctx);
  await ctx.controlTask('cancel');
  assert(ctx.invalidated,'pending start cancellation must revoke the displayed evidence');
  assert.equal(ctx.serviceBusy,true,'cancel reply must not unlock a still-pending start');
  assert.equal(ctx.inspectionStartCancelPending,false);
})().catch(error=>{console.error(error);process.exitCode=1;});
