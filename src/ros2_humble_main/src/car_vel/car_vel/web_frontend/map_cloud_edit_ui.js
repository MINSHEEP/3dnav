(function (global) {
    'use strict';
    class MapCloudEditUi {
        constructor(options) {
            this.options = options;
            this.cells = []; this.strokes = []; this.token = ''; this.expires = 0;
            this.viewers = []; this.enabled = false;
            this.el = id => document.getElementById(id);
            this.el('cloud-edit-enabled').addEventListener('change', () => {
                if (this.cells.length && !global.confirm('切换模式会清空尚未保存的联动选区，是否继续？')) {
                    this.el('cloud-edit-enabled').checked = this.enabled; return;
                }
                this.enabled = this.el('cloud-edit-enabled').checked;
                if (this.enabled && (!options.mapName() || options.dirty() || !options.stopped())) {
                    this.enabled = false;
                    options.log('请先确认核心已停止，打开地图并保存/撤销普通二维修改。');
                }
                this.el('cloud-edit-enabled').checked = this.enabled;
                this.reset(); options.configure();
            });
            ['cloud-edit-min', 'cloud-edit-max'].forEach(id =>
                this.el(id).addEventListener('input', () => {
                    this.invalidate();
                    this.el('cloud-edit-summary').textContent = '高度已改变，当前选区保留；请重新预览。尚未保存地图。';
                }));
            this.el('cloud-edit-confirm').addEventListener('change', () => this.update());
            this.el('cloud-edit-scope').addEventListener('change', () => {
                this.invalidate();
                this.el('cloud-edit-summary').textContent = '清理范围已改变，当前选区保留；请重新预览。尚未保存地图。';
            });
            this.el('btn-cloud-preview').addEventListener('click', () => this.request('preview'));
            this.el('btn-cloud-commit').addEventListener('click', () => this.request('commit'));
            this.el('btn-cloud-restore').addEventListener('click', () => this.request('restore'));
            this.el('btn-cloud-undo').addEventListener('click', () => {
                this.strokes.pop(); this.refreshSelection();
            });
            this.el('btn-cloud-clear').addEventListener('click', () => this.reset());
            this.timer = setInterval(() => this.update(), 500);
            this.update();
        }
        invalidate() {
            this.token = ''; this.expires = 0;
            this.el('cloud-edit-confirm').checked = false;
            this.el('cloud-edit-preview').hidden = true;
            this.el('cloud-edit-reasons').textContent = '';
            this.update();
        }
        reset() {
            this.invalidate(); this.cells = []; this.strokes = [];
            this.options.canvas.selectionRect = null;
            this.options.canvas.selectionCells = [];
            this.el('cloud-edit-summary').textContent = '开启联动后，用画笔或矩形累积选区；预览确认前不修改地图。';
            this.update();
            this.options.selectionChanged?.();
        }
        select(cells) {
            if (!this.enabled || this.options.busy() || !this.options.stopped()) return;
            const existing = new Set(this.cells.map(c=>c.serviceIndex));
            const union = new Map(this.cells.map(c=>[c.serviceIndex,c]));
            cells.forEach(c=>union.set(c.serviceIndex,c));
            const resolution = this.options.canvas.mapInfo.resolution;
            if (union.size > 10000 || union.size*resolution*resolution > 25) {
                this.options.log('累计选区超过10000格/25平方米；本笔未加入，请撤销或清空后分批处理。'); return;
            }
            const added = [...union.values()].filter(c=>!existing.has(c.serviceIndex));
            if (!added.length) return;
            // Store each selected pixel once across history, bounding memory
            // even when the same area is brushed repeatedly.
            this.strokes.push(added); this.refreshSelection();
        }
        refreshSelection() {
            this.invalidate();
            const union = new Map();
            this.strokes.forEach(stroke=>stroke.forEach(c=>union.set(c.serviceIndex,c)));
            this.cells = [...union.values()];
            // Show exact painted pixels, never the bounding box between strokes.
            this.options.canvas.selectionRect = null;
            this.options.canvas.selectionCells = this.cells;
            this.el('cloud-edit-summary').textContent = `累计 ${this.strokes.length} 笔 / ${this.cells.length} 格；`
                + (this.el('cloud-edit-scope').value==='black' ? '仅原二维黑格参与联动；' : '原工具按整个选区筛选三维点；')
                + '地面和范围外点保留。尚未保存，请先预览。';
            this.update();
            this.options.selectionChanged?.();
        }
        update() {
            const busy = this.options.busy(), ready = Boolean(this.options.mapName()) && this.options.stopped();
            this.el('cloud-edit-enabled').disabled = busy || !ready;
            this.el('btn-cloud-preview').disabled = busy || !ready || !this.enabled || !this.cells.length;
            this.el('btn-cloud-commit').disabled = busy || !ready || !this.token || Date.now() >= this.expires
                || !this.el('cloud-edit-confirm').checked;
            this.el('btn-cloud-restore').disabled = busy || !ready;
            this.el('btn-cloud-undo').disabled = busy || !this.strokes.length;
            this.el('btn-cloud-clear').disabled = busy || !this.cells.length;
            for (const id of ['cloud-edit-min', 'cloud-edit-max', 'cloud-edit-confirm','cloud-edit-scope']) this.el(id).disabled = busy;
        }
        cloudMessage(values) {
            const floats = new Float32Array(values), bytes = new Uint8Array(floats.buffer);
            return {width: floats.length/3, height:1, point_step:12, row_step:bytes.length,
                is_bigendian:false, fields:['x','y','z'].map((name,index)=>({name,offset:index*4,datatype:7,count:1})), data:bytes};
        }
        showPreview(result) {
            this.el('cloud-edit-preview').hidden = false;
            if (!this.viewers.length) {
                this.viewers = ['removed', 'kept'].map((name,index) => new global.PointCloudViewer(
                    this.el('cloud-'+name+'-canvas'), this.el('cloud-'+name+'-wrapper'),
                    {maxPoints:12000, pointColor:index ? [.16,.82,.96] : [1,.18,.24]}));
            }
            ['removed_xyz', 'kept_xyz'].forEach((key,index) => {
                if (result[key].length) this.viewers[index].setPointCloud(this.cloudMessage(result[key]));
                else this.viewers[index].clear();
                this.viewers[index].resetView(); this.viewers[index].resize();
            });
            this.el('cloud-edit-summary').textContent = `待删 ${result.delete_count}/${result.total_count} 点；`
                + `选区保留 ${result.protected_points} 点，二维可清理 ${result.cleared_pixels} 格。`
                + (result.delete_count===0 && result.cleared_pixels>0 ? '仅修正二维误黑格，PCD全部保留。' : '')
                + (result.committable===false ? '没有可提交修改。' : '')
                + '两图各自缩放；蓝图含全部高度的保留点，不是高度截取图。每组最多预览12000点（抽样）。';
            const labels = {ground:'地面保留',height:'不在删除高度',ground_unknown:'地面未确认',
                terrain:'台阶/坡度保护',no_floor:'缺少真实地面点',not_black:'非原二维黑格'};
            this.el('cloud-edit-reasons').textContent = Object.entries(labels)
                .map(([key,label])=>`${label} ${result.reason_counts?.[key] || 0} 点`).join('；');
            const range=result.relative_height_range_m;
            if (range?.length===2) this.el('cloud-edit-reasons').textContent +=
                `。选区内、地面已确认处的相对高度范围：${range[0].toFixed(3)}～${range[1].toFixed(3)} m（不是绝对Z）。`;
            if (!result.kept_xyz.length) this.el('cloud-edit-summary').textContent +=
                '框内无保留点显示；地面证据可能在同格邻近像素，建议扩大框选确认地面。';
        }
        async request(action) {
            const o = this.options, name = o.mapName();
            if (o.busy() || !o.stopped() || !name || o.dirty()) return;
            if (action === 'commit' && (!this.token || Date.now() >= this.expires
                    || !this.el('cloud-edit-confirm').checked)) return;
            if (action === 'restore' && !global.confirm('恢复这张地图最近一次三维清理前的五件套？将停止核心节点；恢复后需重新定位。')) return;
            const minimum = Number(this.el('cloud-edit-min').value), maximum = Number(this.el('cloud-edit-max').value);
            if (action === 'preview' && (!this.cells.length || !Number.isFinite(minimum)
                    || !Number.isFinite(maximum) || minimum < .15 || maximum <= minimum || maximum > 3)) {
                o.log('高度范围须为0.15m至3m内，且下限小于上限。'); return;
            }
            const token = this.token;
            this.invalidate(); o.setBusy(true);
            let changed = false;
            try {
                const response = await o.session().call('/edit_map_cloud', 'lzp_interface/srv/EditMapCloud', {
                    action:action === 'preview' && this.el('cloud-edit-scope').value==='black' ? 'preview_linked' : action,
                    map_name:name, indices: action === 'preview' ? this.cells.map(c=>c.serviceIndex) : [],
                    minimum_height_m:minimum, maximum_height_m:maximum, preview_id:token
                }, 180000);
                if (!response.success) throw new Error(response.message || '三维清理请求失败');
                if (o.mapName() !== name) throw new Error('地图已切换，丢弃旧结果');
                const result = JSON.parse(response.result_json);
                if (action === 'preview') {
                    this.showPreview(result); this.token=result.committable === false ? '' : result.preview_id;
                    // Server TTL is120s; leave a conservative client margin.
                    this.expires=Date.now()+110000;
                } else {
                    this.reset(); changed=true;
                    this.el('cloud-edit-summary').textContent=response.message;
                    if (result.backup_path) o.log(`备份保留于：${result.backup_path}`);
                }
                o.log(response.message);
            } catch(error) {
                this.invalidate(); o.log(`三维编辑失败：${error.message || error}`);
            } finally {
                o.setBusy(false); this.update();
                if (changed) await o.refresh();
            }
        }
    }
    global.MapCloudEditUi = MapCloudEditUi;
}(window));
