(function inspectionDraftStateModule(root, factory) {
    'use strict';
    const api = factory();
    if (typeof module === 'object' && module.exports) module.exports = api;
    if (root) root.InspectionDraftState = api;
})(typeof window !== 'undefined' ? window : globalThis, () => {
    'use strict';

    const STORAGE_KEY = 'robot.inspection.idle-draft.v1';
    const ALLOWED_DIRECTIONS = new Set(['current', 'front', 'left', 'right', 'back', 'full']);

    function normalizeCheckpoints(raw, routeId) {
        if (!routeId || !Array.isArray(raw)) return [];
        const seen = new Set();
        const checkpoints = [];
        raw.slice(0, 10000).forEach((item) => {
            if (!item || typeof item !== 'object') return;
            const pointId = Math.trunc(Number(item.route_point_id) || 0);
            if (pointId <= 0 || seen.has(pointId)) return;
            seen.add(pointId);
            const directions = Array.isArray(item.directions)
                ? item.directions.map((value) => String(value || '').trim())
                    .filter((value, index, values) => (
                        ALLOWED_DIRECTIONS.has(value) && values.indexOf(value) === index
                    ))
                : [];
            const zoom = Number(item.zoom_ratio);
            checkpoints.push({
                route_point_id: pointId,
                directions: directions.length ? directions : ['current'],
                zoom_ratio: Number.isFinite(zoom) && zoom >= 0.1 && zoom <= 10 ? zoom : null,
                save_images: Boolean(item.save_images)
            });
        });
        return checkpoints;
    }

    function normalize(raw) {
        if (!raw || typeof raw !== 'object') return null;
        const mapName = String(raw.mapName || '').trim();
        if (!mapName) return null;
        const routeId = Math.max(0, Math.trunc(Number(raw.routeId) || 0));
        return {
            mapName,
            routeId,
            taskName: String(raw.taskName || '').slice(0, 128),
            loopCount: Math.min(10000, Math.max(
                1, Math.trunc(Number(raw.loopCount) || 1))),
            loopInterval: Math.min(100000, Math.max(
                0, Number(raw.loopInterval) || 0)),
            bidirectional: Boolean(raw.bidirectional),
            checkpoints: normalizeCheckpoints(raw.checkpoints, routeId)
        };
    }

    function createInspectionDraftStore(storage) {
        return {
            load() {
                try {
                    return normalize(JSON.parse(storage.getItem(STORAGE_KEY) || 'null'));
                } catch (_error) {
                    return null;
                }
            },
            save(raw) {
                const draft = normalize(raw);
                try {
                    if (draft) storage.setItem(STORAGE_KEY, JSON.stringify(draft));
                    else storage.removeItem(STORAGE_KEY);
                } catch (_error) {
                    // Storage can be disabled by browser privacy settings.  The
                    // inspection page remains usable without draft recovery.
                }
                return draft;
            },
            clear() {
                try {
                    storage.removeItem(STORAGE_KEY);
                } catch (_error) {
                    // See the storage note in save().
                }
            }
        };
    }

    return { STORAGE_KEY, createInspectionDraftStore };
});
