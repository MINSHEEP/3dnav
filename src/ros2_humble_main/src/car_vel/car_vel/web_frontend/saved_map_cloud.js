/* Complete original saved PCD, separate from live ROS preview and control. */
(function(global){
    'use strict';
    const MAX_BYTES=128*1024*1024,MAX_POINTS=4000000,MAX_DECODED=256*1024*1024;
    const TYPES={'F4':7,'F8':8,'I1':1,'U1':2,'I2':3,'U2':4,'I4':5,'U4':6};
    let memory=null;

    function lzf(input,length){
        const output=new Uint8Array(length);let i=0,o=0;
        while(i<input.length){
            const control=input[i++];
            if(control<32){
                const n=control+1;
                if(i+n>input.length||o+n>length)throw new Error('PCD LZF字面量越界');
                output.set(input.subarray(i,i+n),o);i+=n;o+=n;
            }else{
                let n=control>>5,reference=o-((control&31)<<8)-1;
                if(n===7){if(i>=input.length)throw new Error('PCD LZF长度截断');n+=input[i++];}
                if(i>=input.length)throw new Error('PCD LZF引用截断');
                reference-=input[i++];n+=2;
                if(reference<0||o+n>length)throw new Error('PCD LZF引用越界');
                for(let j=0;j<n;j++)output[o++]=output[reference++];
            }
        }
        if(o!==length)throw new Error('PCD解压长度不完整');return output;
    }

    function parsePCD(buffer,identity){
        const bytes=new Uint8Array(buffer),header={};let start=0,offset=0;
        if(!bytes.length||bytes.length>MAX_BYTES)throw new Error('完整PCD超过128MiB限额；未抽稀显示');
        for(let i=0;i<Math.min(bytes.length,65536);i++){
            if(bytes[i]!==10)continue;
            const line=new TextDecoder('ascii').decode(bytes.subarray(start,i)).trim();start=i+1;
            if(!line||line.startsWith('#'))continue;
            const [key,...values]=line.split(/\s+/);header[key.toUpperCase()]=values;
            if(key.toUpperCase()==='DATA'){offset=i+1;break;}
        }
        if(!offset)throw new Error('PCD文件头不完整');
        const names=header.FIELDS||header.FIELD||[],sizes=(header.SIZE||[]).map(Number),types=header.TYPE||[];
        const counts=(header.COUNT||names.map(()=>1)).map(Number);
        const count=Number(header.POINTS&&header.POINTS[0]),width=Number(header.WIDTH&&header.WIDTH[0]),height=Number(header.HEIGHT&&header.HEIGHT[0]);
        if(!Number.isSafeInteger(count)||count<1||count>MAX_POINTS||width*height!==count
            ||!Number.isSafeInteger(width)||!Number.isSafeInteger(height)||width<1||height<1
            ||!names.length||names.length!==sizes.length||names.length!==types.length||names.length!==counts.length
            ||new Set(names).size!==names.length)throw new Error('PCD点数或字段声明无效；未使用简化地图');
        let step=0;
        const descriptors=names.map((name,i)=>{
            const size=sizes[i],n=counts[i],type=types[i];
            if(!Number.isInteger(n)||n<1||!([1,2,4,8].includes(size))||!['F','I','U'].includes(type)
                ||(type==='F'&&![4,8].includes(size)))throw new Error('PCD字段类型无效');
            const field={name,size,type,count:n,offset:step,datatype:TYPES[type+size]||0};step+=size*n;return field;
        });
        for(const name of ['x','y','z']){
            const f=descriptors.find(field=>field.name===name);
            if(!f||f.type!=='F'||f.count!==1)throw new Error('PCD需要浮点XYZ坐标');
        }
        if(step<12||step>4096||count*step>MAX_DECODED)throw new Error('完整地图解码超过资源限额；未截断点云');
        const mode=header.DATA[0].toLowerCase();let data;
        if(mode==='binary'){
            data=bytes.subarray(offset);
            if(data.length!==count*step)throw new Error('PCD二进制数据长度不完整');
        }else if(mode==='binary_compressed'){
            const view=new DataView(buffer,offset);
            if(view.byteLength<8)throw new Error('PCD压缩头不完整');
            const compressed=view.getUint32(0,true),expanded=view.getUint32(4,true);
            if(expanded!==count*step||compressed!==view.byteLength-8)throw new Error('PCD压缩长度声明不一致');
            const soa=lzf(bytes.subarray(offset+8),expanded);data=new Uint8Array(expanded);let fieldBase=0;
            for(const field of descriptors){
                const size=field.size*field.count;
                for(let point=0;point<count;point++)for(let j=0;j<size;j++)
                    data[point*step+field.offset+j]=soa[fieldBase+point*size+j];
                fieldBase+=count*size;
            }
        }else if(mode==='ascii'){
            const text=new TextDecoder().decode(bytes.subarray(offset));let cursor=0;
            const token=()=>{
                while(cursor<text.length&&/\s/.test(text[cursor]))cursor++;
                const begin=cursor;while(cursor<text.length&&!/\s/.test(text[cursor]))cursor++;
                return text.slice(begin,cursor);
            };
            data=new Uint8Array(count*step);const view=new DataView(data.buffer);
            for(let point=0;point<count;point++)for(const f of descriptors)for(let j=0;j<f.count;j++){
                const t=token();if(!t)throw new Error('PCD文本点数不足');
                const at=point*step+f.offset+j*f.size;
                if(f.type==='F'){
                    if(!/^[+-]?(?:(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?|nan|inf(?:inity)?)$/i.test(t))
                        throw new Error('PCD文本数值无效');
                    const n=/^[+]?inf(?:inity)?$/i.test(t)?Infinity:/^-inf(?:inity)?$/i.test(t)?-Infinity:Number(t);
                    view[f.size===4?'setFloat32':'setFloat64'](at,n,true);
                }else{
                    if(!/^[+-]?\d+$/.test(t))throw new Error('PCD整数字段无效');
                    const method='set'+(f.size===8?'Big':'')+(f.type==='I'?'Int':'Uint')+(f.size*8);
                    view[method](at,f.size===8?BigInt(t):Number(t),true);
                }
            }
            if(token())throw new Error('PCD文本点数超过声明');
        }else throw new Error('不支持的PCD编码；未回退到抽稀点云');
        const stamp=BigInt(identity.stamp_ns);
        return {header:{frame_id:'map',stamp:{sec:Number(stamp/1000000000n),nanosec:Number(stamp%1000000000n)}},
            width,height,point_step:step,row_step:width*step,fields:descriptors,is_bigendian:false,
            is_dense:false,data,completeSavedMap:true,savedMapSha256:identity.sha256};
    }

    // Native WebCrypto is unavailable on ordinary HTTP robot addresses.
    // This bounded fallback also verifies IndexedDB bytes before reusing them.
    async function sha256(buffer){
        if(global.crypto&&global.crypto.subtle){
            const hash=await global.crypto.subtle.digest('SHA-256',buffer);
            return Array.from(new Uint8Array(hash),n=>n.toString(16).padStart(2,'0')).join('');
        }
        const k=[0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
            0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
            0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
            0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
            0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
            0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
            0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
            0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
        const h=[0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19];
        const bytes=new Uint8Array(buffer),length=bytes.length,blocks=Math.ceil((length+9)/64),w=new Uint32Array(64);
        const rotate=(n,b)=>(n>>>b)|(n<<(32-b));
        const byte=index=>index<length?bytes[index]:index===length?128:0;
        for(let block=0;block<blocks;block++){
            for(let j=0;j<16;j++){const at=block*64+j*4;w[j]=((byte(at)<<24)|(byte(at+1)<<16)|(byte(at+2)<<8)|byte(at+3))>>>0;}
            if(block===blocks-1){w[14]=Math.floor(length/0x20000000);w[15]=(length*8)>>>0;}
            for(let j=16;j<64;j++){
                const a=w[j-15],b=w[j-2];
                w[j]=(w[j-16]+(rotate(a,7)^rotate(a,18)^(a>>>3))+w[j-7]+(rotate(b,17)^rotate(b,19)^(b>>>10)))>>>0;
            }
            let [a,b,c,d,e,f,g,h0]=h;
            for(let j=0;j<64;j++){
                const t1=(h0+(rotate(e,6)^rotate(e,11)^rotate(e,25))+((e&f)^(~e&g))+k[j]+w[j])>>>0;
                const t2=((rotate(a,2)^rotate(a,13)^rotate(a,22))+((a&b)^(a&c)^(b&c)))>>>0;
                h0=g;g=f;f=e;e=(d+t1)>>>0;d=c;c=b;b=a;a=(t1+t2)>>>0;
            }
            for(const [j,n]of[a,b,c,d,e,f,g,h0].entries())h[j]=(h[j]+n)>>>0;
        }
        return h.map(n=>n.toString(16).padStart(8,'0')).join('');
    }

    async function diskCache(digest,bytes){
        if(!global.indexedDB)return null;
        return new Promise(resolve=>{
            let db,open,finished=false;
            const finish=result=>{if(finished)return;finished=true;global.clearTimeout(timer);if(db)db.close();resolve(result);};
            const timer=global.setTimeout(()=>finish(null),2500);
            try{open=global.indexedDB.open('robot.complete-saved-map',1);}catch(_error){finish(null);return;}
            open.onupgradeneeded=()=>open.result.createObjectStore('pcd');
            open.onerror=open.onblocked=()=>finish(null);
            open.onsuccess=()=>{
                db=open.result;
                if(finished){db.close();return;}
                try{
                    const tx=db.transaction('pcd',bytes?'readwrite':'readonly'),store=tx.objectStore('pcd');
                    const request=bytes?store.put({digest,bytes},'latest'):store.get('latest');
                    let result=null;
                    request.onsuccess=()=>{if(!bytes&&request.result&&request.result.digest===digest)result=request.result.bytes;};
                    tx.oncomplete=()=>finish(result);
                    tx.onerror=tx.onabort=()=>finish(null);
                }catch(_error){finish(null);}
            };
        });
    }

    class SavedMapCloud {
        constructor(session,accept,state){
            this.session=session;this.accept=accept;this.state=state||(()=>{});
            this.visible=true;this.key='';this.generation=0;this.disposed=false;
            this.retryKey='';this.attempts=0;
            this.visibility=()=>this._visibility();
            this.pagehide=()=>this._abort();
            global.document.addEventListener('visibilitychange',this.visibility);
            global.addEventListener('pagehide',this.pagehide);
            global.addEventListener('pageshow',this.visibility);
        }
        _visibility(){
            if(global.document.hidden)this._abort();
            else if(this.wanted)this.request(this.wanted);
        }
        _abort(){this.generation++;this.key='';global.clearTimeout(this.retryTimer);global.clearTimeout(this.idleTimer);
            if(this.controller)this.controller.abort();this.controller=null;}
        cancel(){this.wanted=null;this.retryKey='';this.attempts=0;this._abort();}
        dispose(){this.disposed=true;this.cancel();
            global.document.removeEventListener('visibilitychange',this.visibility);
            global.removeEventListener('pagehide',this.pagehide);global.removeEventListener('pageshow',this.visibility);}
        setVisible(visible){this.visible=Boolean(visible);if(!this.visible)this._abort();else this._visibility();}
        request(identity){
            this.wanted=identity;
            if(this.disposed||!this.visible||global.document.hidden||!this.session.connected
                ||!identity||!identity.map_name||!identity.load_id
                ||(identity.stamp_ns!==undefined&&!/^\d+$/.test(identity.stamp_ns)))return;
            const key=JSON.stringify(identity);
            if(key===this.key)return;
            if(this.retryKey!==key){this.retryKey=key;this.attempts=0;}
            this.attempts++;
            this._abort();this.key=key;const generation=this.generation;
            this.controller=new AbortController();const signal=this.controller.signal;
            const active=()=>!signal.aborted&&!this.disposed&&generation===this.generation&&this.session.connected;
            this.touch=()=>{
                global.clearTimeout(this.idleTimer);
                this.idleTimer=global.setTimeout(()=>{
                    if(active()){this.state('error','完整地图传输30秒无进展，请检查8769端口/网络后切换视图重试；未显示简化地图。');this.controller.abort();}
                },30000);
            };
            this.touch();
            this.pending=this._load(identity,signal,active).catch(error=>{
                if(!active())return;
                const retry=(error.retryable||error instanceof TypeError)&&this.attempts<5;
                this.state(retry?'loading':'error',`完整地图加载失败：${error.message}；${retry?'正在有界重试':'未使用简化点云代替'}。`);
                if(retry)this.retryTimer=global.setTimeout(()=>{
                    if(active()){this.key='';this.request(identity);}
                },Math.min(4000,this.attempts*1000));
            }).finally(()=>{if(generation===this.generation)global.clearTimeout(this.idleTimer);});
        }
        async _load(identity,signal,active){
            const url=new URL(this.session.url);url.protocol='http:';
            const parameters=new URL(global.location.href).searchParams;
            const port=Number(parameters.get('saved_map_port')||8769);
            if(!Number.isInteger(port)||port<1||port>65535)throw new Error('完整地图端口无效');
            url.port=String(port);url.pathname='/api/saved-map/manifest';
            url.search=new URLSearchParams(identity).toString();url.hash='';
            this.state('loading','正在校验原始PCD完整地图…');
            const response=await global.fetch(url,{signal,cache:'no-store'});
            if(!response.ok){const error=new Error((await response.json()).error||`HTTP ${response.status}`);
                error.retryable=[409,429,503].includes(response.status);throw error;}
            this.touch();
            const info=await response.json();if(!active())return;
            if(!info.complete||info.frame_id!=='map'||!Number.isSafeInteger(info.point_count)||info.point_count<1||info.point_count>MAX_POINTS
                ||!Number.isSafeInteger(info.size_bytes)||info.size_bytes<1||info.size_bytes>MAX_BYTES
                ||typeof info.stamp_ns!=='string'||!/^\d+$/.test(info.stamp_ns)
                ||!/^[0-9a-f]{64}$/.test(info.sha256)||Object.keys(identity).some(k=>info[k]!==identity[k]))
                throw new Error('完整地图清单身份、点数或文件大小无效');
            let bytes=memory&&memory.digest===info.sha256?memory.bytes:await diskCache(info.sha256);
            if(!active())return;
            if(bytes&&(bytes.byteLength!==info.size_bytes||await sha256(bytes)!==info.sha256))bytes=null;
            if(!bytes){
                url.pathname='/api/saved-map/file';url.searchParams.set('sha256',info.sha256);
                url.searchParams.set('stamp_ns',info.stamp_ns);
                this.state('loading',`正在下载完整PCD：${info.point_count.toLocaleString()} 点，${(info.size_bytes/1048576).toFixed(1)} MiB…`);
                const file=await global.fetch(url,{signal,cache:'no-store'});
                if(!file.ok){const error=new Error((await file.json()).error||`HTTP ${file.status}`);
                    error.retryable=[409,429,503].includes(file.status);throw error;}
                this.touch();
                const length=Number(file.headers.get('Content-Length'));
                if(length!==info.size_bytes)throw new Error('完整地图传输长度与清单不符');
                // Preallocate exactly once. No unbounded chunk list on a slow connection.
                bytes=new ArrayBuffer(info.size_bytes);const target=new Uint8Array(bytes);
                if(file.body&&file.body.getReader){
                    const reader=file.body.getReader();let offset=0;
                    while(true){const {done,value}=await reader.read();if(done)break;
                        if(!active()){await reader.cancel();return;}
                        if(offset+value.length>target.length){await reader.cancel();throw new Error('地图传输超过声明长度');}
                        target.set(value,offset);offset+=value.length;
                        this.touch();
                        this.state('loading',`完整地图下载 ${(offset/target.length*100).toFixed(0)}% · ${info.point_count.toLocaleString()} 点`);
                    }
                    if(offset!==target.length)throw new Error('完整地图传输中断，未收齐文件');
                }else{
                    const data=await file.arrayBuffer();if(data.byteLength!==target.length)throw new Error('PCD长度不完整');target.set(new Uint8Array(data));
                }
                if(await sha256(bytes)!==info.sha256)throw new Error('完整PCD的SHA256校验失败');
                if(!active())return;
                memory={digest:info.sha256,bytes};
                // Quota failure only disables disk caching, never removes points.
                void diskCache(info.sha256,bytes);
            }
            if(!active())return;
            this.state('loading','正在解析、绘制完整地图（不抽稀）…');
            const message=parsePCD(bytes,info);
            if(message.width*message.height!==info.point_count)throw new Error('PCD实际点数与清单不符');
            if(!active())return;
            this.accept(message,info);
        }
    }
    global.SavedMapCloud=SavedMapCloud;
    global.SavedMapPcd=Object.freeze({parsePCD,sha256});
})(window);
