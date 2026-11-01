'use strict';
const assert=require('node:assert/strict'),fs=require('fs'),path=require('path'),vm=require('vm'),crypto=require('crypto');
const web=path.resolve(__dirname,'../car_vel/web_frontend');
const events=new Map(),states=[],received=[];
const document={hidden:false,addEventListener:(name,cb)=>events.set(name,cb),removeEventListener(){}};
const window={document,location:{href:'http://robot.test/navigation.html'},setTimeout,clearTimeout,
    addEventListener(){},removeEventListener(){}};
const context=vm.createContext({window,TextDecoder,URL,URLSearchParams,AbortController,console});
for(const name of ['saved_map_cloud.js','pointcloud_viewer.js'])
    vm.runInContext(fs.readFileSync(path.join(web,name),'utf8'),context);
const buffer=raw=>raw.buffer.slice(raw.byteOffset,raw.byteOffset+raw.byteLength);
const header=(count,mode='binary')=>Buffer.from(`VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ${count}\nHEIGHT 1\nPOINTS ${count}\nDATA ${mode}\n`);
const count=150001,body=Buffer.alloc(count*16);
for(let i=0;i<count;i++){body.writeFloatLE(i,i*16);body.writeFloatLE(i%3,i*16+4);body.writeFloatLE(1,i*16+8);body.writeFloatLE(9,i*16+12);}
const raw=Buffer.concat([header(count),body]);
const digest=crypto.createHash('sha256').update(raw).digest('hex');
const identity={map_name:'room',load_id:'load-1',stamp_ns:'1000000001'};
const parsed=window.SavedMapPcd.parsePCD(buffer(raw),{...identity,sha256:digest});
assert.equal(parsed.width,count);assert.equal(parsed.point_step,16);
assert(Buffer.from(parsed.data).equals(body),'all original binary fields/points preserved');
const decode=window.PointCloudViewer.prototype.decodePointCloud;
const all=decode.call({maxPoints:120000,fullResolution:true},parsed);
assert.equal(all.stats.renderedPoints,count);assert.equal(all.stats.invalidPoints,0);
assert.equal(all.stats.bounds.maxX,count-1,'last saved point must not be truncated');
const preview=decode.call({maxPoints:120000,fullResolution:false},parsed);
assert(preview.stats.renderedPoints<=120000,'mapping preview retains original point cap');
const ascii=Buffer.concat([header(2,'ascii'),Buffer.from('0 1 2 8\nnan 2 3 7\n')]);
const asciiStats=decode.call({fullResolution:true},window.SavedMapPcd.parsePCD(buffer(ascii),identity)).stats;
assert.equal(asciiStats.renderedPoints,1);assert.equal(asciiStats.invalidPoints,1);
const uncompressed=Buffer.alloc(32);
for(let field=0;field<4;field++)for(let point=0;point<2;point++)uncompressed.writeFloatLE(field+point,field*8+point*4);
const compressed=Buffer.concat([Buffer.from([31]),uncompressed]);
const length=Buffer.alloc(8);length.writeUInt32LE(compressed.length,0);length.writeUInt32LE(32,4);
const pcl=window.SavedMapPcd.parsePCD(buffer(Buffer.concat([header(2,'binary_compressed'),length,compressed])),identity);
assert.equal(new DataView(pcl.data.buffer).getFloat32(16,true),1,'SoA must be restored without dropping points');
const referenceHeader=Buffer.from('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary_compressed\n');
const refLength=Buffer.alloc(8);refLength.writeUInt32LE(5,0);refLength.writeUInt32LE(24,4);
assert.equal(window.SavedMapPcd.parsePCD(buffer(Buffer.concat([referenceHeader,refLength,Buffer.from([0,0,224,14,0])])),identity).data.length,24);
assert.throws(()=>window.SavedMapPcd.parsePCD(buffer(raw.subarray(0,raw.length-1)),identity),/长度/);
const gl={ARRAY_BUFFER:1,DYNAMIC_DRAW:2,NO_ERROR:0,bindBuffer(){},bufferData(){},getError:()=>1285};
assert.throws(()=>window.PointCloudViewer.prototype.setPointCloud.call({decodePointCloud:()=>all,gl,fullResolution:true},parsed),/显存/);
(async()=>{
    for(const n of [0,1,55,56,63,64,1000]){
        const bytes=Buffer.alloc(n,97);
        assert.equal(await window.SavedMapPcd.sha256(buffer(bytes)),crypto.createHash('sha256').update(bytes).digest('hex'));
    }
    assert.equal(await window.SavedMapPcd.sha256(buffer(raw)),digest,'HTTP-address SHA256 fallback');
    let files=0,manifests=0;
    window.fetch=async url=>{
        const u=new URL(url);assert.equal(u.port,'8769');
        if(u.pathname.endsWith('/manifest')){
            manifests++;return {ok:true,json:async()=>({...identity,load_id:u.searchParams.get('load_id'),
                frame_id:'map',complete:true,point_count:count,size_bytes:raw.length,sha256:digest})};
        }
        files++;let delivered=false;
        return {ok:true,headers:{get:()=>String(raw.length)},body:{getReader:()=>({read:async()=>
            delivered?{done:true}:(delivered=true,{done:false,value:new Uint8Array(buffer(raw))}),cancel:async()=>{}})}};
    };
    const session={connected:true,url:'ws://robot.test:9090'};
    const loader=new window.SavedMapCloud(session,msg=>received.push(msg),(kind,text)=>states.push({kind,text}));
    loader.request(identity);await loader.pending;assert.equal(received.length,1);assert.equal(files,1);
    loader.request({...identity,load_id:'load-2'});await loader.pending;
    assert.equal(received.length,2);assert.equal(files,1,'same verified PCD reused across load identities');assert.equal(manifests,2);
    document.hidden=true;events.get('visibilitychange')();
    loader.request({...identity,load_id:'load-3'});assert.equal(manifests,2);
    document.hidden=false;events.get('visibilitychange')();await loader.pending;
    assert.equal(received.length,3);assert.equal(files,1);
    loader.dispose();
    const bootstrap=[];
    const late=new window.SavedMapCloud(session,(msg,info)=>bootstrap.push({msg,info}),()=>{});
    late.request({map_name:'room',load_id:'late-load'});await late.pending;
    assert.equal(bootstrap.length,1,'HTTP supplies exact source stamp even if browser missed ROS status');
    assert.equal(bootstrap[0].info.stamp_ns,identity.stamp_ns);assert.equal(files,1);late.dispose();
    const privateWindow={...window,indexedDB:{open(){throw new Error('fixture denied storage');}}};
    vm.runInNewContext(fs.readFileSync(path.join(web,'saved_map_cloud.js'),'utf8'),
        {window:privateWindow,TextDecoder,URL,URLSearchParams,AbortController,console});
    const uncached=[];
    const privateLoader=new privateWindow.SavedMapCloud(session,msg=>uncached.push(msg),()=>{});
    privateLoader.request(identity);await privateLoader.pending;
    assert.equal(uncached.length,1,'storage denial must not drop points or prevent full loading');privateLoader.dispose();
    let release;
    window.fetch=()=>new Promise(resolve=>{release=resolve;});
    const stopped=new window.SavedMapCloud(session,msg=>received.push(msg),()=>{});
    stopped.request(identity);stopped.cancel();release({ok:true,json:async()=>({})});await stopped.pending;
    assert.equal(received.length,3,'cancelled HTTP response ignored');stopped.dispose();
    console.log('PASS: 150001 complete points, mapping cap unchanged, ASCII/LZF/fields/SHA256, cache, visibility/cancel and explicit GPU failure');
})().catch(error=>{console.error(error);process.exitCode=1;});
