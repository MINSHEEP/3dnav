/* Real initialization UI during Nav2 activation, without robot/network access. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../car_vel/web_frontend/navigation.html'), 'utf8');
function extract(name) {
    const match = html.match(new RegExp(`(?:async )?function ${name}\\([^]*?\\n    \\}`));
    assert(match, name); return match[0];
}
const nodes = new Map();
const context = {
    Date:{now:()=>10000},
    document:{getElementById(id){if(!nodes.has(id))nodes.set(id,{});return nodes.get(id);}},
    navigationStartupStatus:{pid:12,state:'activating',elapsed_sec:20,
        message:'等待节点激活：planner_server、bt_navigator',
        lifecycle:{map_server:'active',amcl:'active',planner_server:'状态查询无响应',bt_navigator:'inactive'}},
    navigationStartupReceivedAt:10000, navigationStartRequest:null,
    session:{connected:true}, navigationActiveKnown:true,navigationActive:false,
    routeIsActive:()=>false,relocalizationActive:false,navigationSafetyPending:false,
    nav2Ready:false, navigationMapReady:true,loadedMapName:'map',pendingMapName:'',
    navigationMapView:'3d',threeDNavigationMode:'terrain_3d',localizationReady:false,
    lioFaultLatched:false,isGangbenThreeD:()=>false,
};
vm.createContext(context);
if (html.includes('function initialLocalizationStartupHint(')) {
    vm.runInContext(extract('initialLocalizationStartupHint'),context);
}
vm.runInContext(extract('renderNavigationStartup'),context);
context.renderNavigationStartup();
assert.match(nodes.get('navigation-startup-detail').textContent,/设置.*初始点/,
    'loaded map with active localization must explain initialization before full Nav2 activation');
assert.equal(context.nav2Ready,false,'initialization advice must not grant motion readiness');
context.threeDNavigationMode='map_2d';
context.renderNavigationStartup();
assert.doesNotMatch(nodes.get('navigation-startup-detail').textContent,/设置.*初始点/,
    'original 2D startup text remains unchanged');
context.threeDNavigationMode='terrain_3d';
context.navigationStartupReceivedAt=1;
context.renderNavigationStartup();
assert.doesNotMatch(nodes.get('navigation-startup-detail').textContent,/设置.*初始点/,
    'stale service diagnostics do not authorize initialization advice');
console.log('PASS: 3D initialization phase explained before Nav2 activation; legacy/stale states preserved');

// Exercise actual ground interaction/confirmation, not only the hint text.
Object.assign(context, {
    navigationStartupReceivedAt:10000,
    terrainReloadRequired:null,currentMapLoadName:'map',currentMapLoadId:'load',
    navigationPointCloudViewer:{cloudTransform:{},isTopView:()=>true},
    latestNavigationGlobalCloud:{header:{frame_id:'map'}},
    pointCloudStampNanoseconds:()=> '100',
    latestNavigationGlobalCloudIdentity:{map_name:'map',load_id:'load',stamp_ns:'100'},
    threeDNavigationModeAtMs:10000,threeDRobotBodyAtMs:10000,THREE_D_STATUS_MAX_AGE_MS:5000,
    threeDRobotBodyStatus:{connected:true,active_body:'xingji',status:{},
        config:{selected:'xingji',profiles:{xingji:{navigation_verified:true}}}},
    threeDPoseDraft:{kind:'initial',groundValid:true,busy:false,mapLoadId:'load',x:1,y:2,yaw:.1},
    clearThreeDPoseDraft(){},updateThreeDPoseUi(){},updateMapReadyUi(){},
    goalStatusHydrated:true,singleGoalRefreshState:'',navigationSafetyManualHold:false,
});
let confirmed = null;
context.publishInitialPose=async(...args)=>{confirmed=args;};
vm.runInContext(extract('threeDInteractionIssue')+'\n'+extract('confirmThreeDInitialPose')
    +'\n'+extract('goalReadinessIssue'),context);
assert.equal(context.threeDInteractionIssue(),'','valid initial pick is allowed before Nav2 readiness');
assert.match(context.goalReadinessIssue(),/Nav2/,'motion/preview remains blocked');
context.confirmThreeDInitialPose().then(()=>{
    assert.deepEqual(confirmed,[1,2,.1,'load']);
    context.threeDRobotBodyStatus.connected=false;
    assert.match(context.threeDInteractionIssue(),/失联|未确认/,'body connection requirement is retained');
    console.log('PASS: actual initial-point confirmation works before Nav2 activation; goals and disconnected body remain blocked');
}).catch(error=>{console.error(error);process.exitCode=1;});
