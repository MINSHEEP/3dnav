"""Offline cleanup interface tests. Never edits a user's saved map."""
from pathlib import Path
import sys
import unittest
import tempfile
import json
import ast
import os
import threading
from unittest.mock import patch
from unittest.mock import Mock
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PIL import Image
from lzp_service.map_cloud_editor import plan_cleanup, MapCloudEditor, write_records, read_records, sha
from lzp_service.terrain_grid import build_terrain_grid, GroundSupportGrid
from lzp_service.terrain_grid_io import write_terrain_grid, create_terrain_grid_metadata, load_bound_terrain_grid
from lzp_service.saved_map_lock import saved_map_lock, saved_map_write_busy


class CleanupSelectionTests(unittest.TestCase):
    def fixture(self):
        points = np.array([[.15, .05, 1], [.15, .05, 1.03], [.15, .05, 1.2],
                           [.15, .05, 1.4], [.15, .05, 3], [.65, .05, 1.4]], np.float32)
        grid = NS(width=2, height=1, origin_x=0, origin_y=0, resolution=.5,
                  elevation=np.array([1, 1], np.float32),
                  confidence=np.ones(2), ground_measured=np.ones(2),
                  step_height_m=np.zeros(2), slope_deg=np.zeros(2))
        spec = dict(resolution=.1, origin=[0, 0, 0], width=10, height=5)
        image = np.full((5, 10), 254, np.uint8)
        image[4, 1] = 0
        return points, grid, spec, image

    def test_relative_height_and_ground_protection(self):
        pts, grid, spec, image = self.fixture()
        result = plan_cleanup(pts, grid, spec, image, [41], .15, .6)
        self.assertEqual(np.flatnonzero(result['remove']).tolist(), [2, 3])
        self.assertEqual(result['image'][4, 1], 254)
        self.assertEqual(image[4, 1], 0, 'preview must not mutate source')

    def test_linked_ground_mislabel_clears_only_2d_and_keeps_floor(self):
        pts,grid,spec,image=self.fixture()
        result=plan_cleanup(pts[:2],grid,spec,image,[41],.15,.8,linked=True)
        self.assertFalse(result['remove'].any())
        self.assertEqual(result['image'][4,1],254)
        self.assertEqual(result['reason_counts']['ground'],2)

    def test_linked_only_targets_original_black_pixels(self):
        pts,grid,spec,image=self.fixture()
        for value in (254,205):
            with self.subTest(value=value):
                image[4,1]=value
                result=plan_cleanup(pts,grid,spec,image,[41],.15,.8,linked=True)
                self.assertFalse(result['remove'].any())
                self.assertEqual(result['image'][4,1],value)
                self.assertEqual(result['reason_counts']['not_black'],5)

    def test_retention_reasons_partition_selected_points(self):
        pts,grid,spec,image=self.fixture()
        result=plan_cleanup(pts,grid,spec,image,[41],.15,.6)
        self.assertEqual(result['reason_counts']['ground'],2)
        self.assertEqual(result['reason_counts']['height'],1)
        self.assertEqual(sum(result['reason_counts'].values()),3)
        for field,value,reason in [('ground_measured',0,'ground_unknown'),
                                   ('confidence',.1,'ground_unknown'),
                                   ('step_height_m',.15,'terrain'),
                                   ('slope_deg',float('nan'),'terrain')]:
            pts,grid,spec,image=self.fixture()
            getattr(grid,field)[0]=value
            result=plan_cleanup(pts,grid,spec,image,[41],.15,.6,linked=True)
            self.assertEqual(result['reason_counts'][reason],5)
            self.assertEqual(result['image'][4,1],0)
        pts,grid,spec,image=self.fixture()
        result=plan_cleanup(pts[2:],grid,spec,image,[41],.15,.6,linked=True)
        self.assertEqual(result['reason_counts']['no_floor'],3)

    def test_linked_remaining_body_points_veto_ground_only_clear(self):
        pts,grid,spec,image=self.fixture()
        result=plan_cleanup(pts,grid,spec,image,[41],.5,.8,linked=True)
        self.assertFalse(result['remove'].any())
        self.assertEqual(result['image'][4,1],0)

    def test_unknown_ground_never_authorizes_deletion(self):
        pts, grid, spec, image = self.fixture()
        grid.ground_measured[0] = 0
        result = plan_cleanup(pts, grid, spec, image, [41], .15, .6)
        self.assertFalse(result['remove'].any())
        self.assertEqual(result['image'][4, 1], 0)

    def test_remaining_obstacle_does_not_become_free(self):
        pts, grid, spec, image = self.fixture()
        result = plan_cleanup(pts, grid, spec, image, [41], .15, .3)
        self.assertEqual(np.flatnonzero(result['remove']).tolist(), [2])
        self.assertEqual(result['image'][4, 1], 0)

    def test_ground_intersection_or_bad_indices_rejected(self):
        pts, grid, spec, image = self.fixture()
        for indices, low, high in [([41], .01, .6), ([-1], .15, .6),
                                    ([50], .15, .6), ([41], .7, .6)]:
            with self.subTest(indices=indices, low=low):
                with self.assertRaises(ValueError):
                    plan_cleanup(pts, grid, spec, image, indices, low, high)

    def test_missing_floor_returns_no_deletion_even_with_stored_ground(self):
        pts, grid, spec, image = self.fixture()
        pts = pts[2:]
        result = plan_cleanup(pts, grid, spec, image, [41], .15, .6)
        self.assertFalse(result['remove'].any())

    def test_known_step_or_steep_ground_is_preserved(self):
        pts, grid, spec, image = self.fixture()
        for field, value in [('step_height_m', .15), ('slope_deg', 20)]:
            with self.subTest(field=field):
                old=getattr(grid, field).copy()
                getattr(grid, field)[0]=value
                result=plan_cleanup(pts,grid,spec,image,[41],.15,.6)
                self.assertFalse(result['remove'].any())
                setattr(grid, field, old)

    def test_unknown_2d_pixels_are_never_promoted_to_free(self):
        pts,grid,spec,image=self.fixture()
        image[4,1]=205
        result=plan_cleanup(pts,grid,spec,image,[41],.15,.6)
        self.assertEqual(result['image'][4,1],205)


class CleanupTransactionTests(unittest.TestCase):
    def setUp(self):
        workspace_tmp = Path(__file__).resolve().parents[6]/'tmp'
        workspace_tmp.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix='map-cloud-tests-', dir=workspace_tmp)
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.maps, self.clouds = root/'maps', root/'pcdmaps'
        self.maps.mkdir(); self.clouds.mkdir()
        self.engine = MapCloudEditor(self.maps, self.clouds,
            load_document=lambda p: json.loads(Path(p).read_text(encoding='utf-8')),
            write_document=lambda p, v: Path(p).write_text(json.dumps(v), encoding='utf-8'))
        self.paths = self.engine.paths('test')
        floor = np.array([[x*.1+.05,y*.1+.05,0] for x in range(52) for y in range(50)],np.float32)
        xyz = np.concatenate([floor, np.tile([2.15,2.15,.4],(10,1))]).astype(np.float32)
        records = np.zeros(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<u4')])
        for i,k in enumerate('xyz'): records[k]=xyz[:,i]
        records['intensity']=np.arange(len(records),dtype=np.uint32)+2**30
        self.records=records
        header=dict(FIELDS=['x','y','z','intensity'],SIZE=['4']*4,TYPE=['F','F','F','U'],COUNT=['1']*4)
        write_records(self.paths['pcd'],records,header)
        image=np.full((60,60),254,np.uint8); image[38,21]=0
        Image.fromarray(image).save(self.paths['image'])
        spec=dict(image='test.pgm',mode='trinary',negate=0,resolution=.1,origin=[0,0,0],occupied_thresh=.65,free_thresh=.196)
        self.engine.write_document(self.paths['yaml'],spec)
        support=GroundSupportGrid(.1,60,60,0,0,np.flipud(image==254))
        parameters=dict(resolution=.5)
        grid=build_terrain_grid(xyz,ground_support=support,**parameters)
        generation=dict(algorithm='clustered_measured_ground_v3',source='test',
            source_pcd_sha256=sha(self.paths['pcd']),parameters=parameters,
            quality={})
        summary=write_terrain_grid(self.paths['terrain'],map_name='test',frame_id='map',grid=grid,generation=generation)
        terrain=create_terrain_grid_metadata(self.paths['terrain'],summary=summary,source_pcd_sha256=sha(self.paths['pcd']))
        meta=dict(schema='ros2_mid360_localization_map',version=1,map_name='test',frame_id='map',pcd_file='test.pcd',
            point_count=len(records),pcd_size_bytes=self.paths['pcd'].stat().st_size,pcd_sha256=sha(self.paths['pcd']),
            map_geometry=dict(width=60,height=60,resolution=.1,origin=[0,0,0]),terrain_grid=terrain)
        self.engine.write_document(self.paths['metadata'],meta)
        self.before=self.engine.fingerprints(self.paths)

    def preview(self):
        return self.engine.preview('test',[38*60+21],.15,.8)

    def test_preview_has_no_file_side_effect_and_exact_points(self):
        result=self.preview()
        self.assertEqual(result['delete_count'],10)
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)
        self.assertTrue(all(z==0 for z in result['kept_xyz'][2::3]))

    def test_zero_change_preview_explains_reasons_without_commit_token(self):
        result=self.engine.preview('test',[18*60+12],.15,.8,linked=True)
        self.assertFalse(result['committable'])
        self.assertEqual(result['preview_id'],'')
        self.assertEqual(result['delete_count'],0)
        self.assertGreater(result['reason_counts']['not_black'],0)
        self.assertTrue(result['kept_xyz'])
        with self.assertRaises(ValueError):self.engine.commit('test',result['preview_id'])
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)

    def test_ground_only_linked_transaction_preserves_pcd_bytes_and_restores(self):
        image=np.asarray(Image.open(self.paths['image'])).copy()
        image[18,12]=0
        Image.fromarray(image).save(self.paths['image'])
        before=self.engine.fingerprints(self.paths)
        result=self.engine.preview('test',[18*60+12],.15,.8,linked=True)
        self.assertEqual(result['delete_count'],0)
        self.assertEqual(result['cleared_pixels'],1)
        self.assertTrue(result['committable'])
        self.engine.commit('test',result['preview_id'])
        self.assertEqual(sha(self.paths['pcd']),before['pcd'])
        self.assertEqual(np.asarray(Image.open(self.paths['image']))[18,12],254)
        self.engine.restore('test')
        self.assertEqual(self.engine.fingerprints(self.paths),before)

    def test_commit_preserves_fields_validates_metadata_and_restores_all_files(self):
        result=self.preview()
        saved=self.engine.commit('test',result['preview_id'])
        records,_,_=read_records(self.paths['pcd'])
        np.testing.assert_array_equal(records,self.records[:-10])
        metadata=self.engine.load_document(self.paths['metadata'])
        self.assertEqual(metadata['point_count'],2600)
        load_bound_terrain_grid(self.clouds,map_name='test',localization_metadata=metadata)
        self.assertTrue(Path(saved['backup_path']).is_dir())
        self.assertEqual(np.asarray(Image.open(self.paths['image']))[38,21],254)
        self.engine.restore('test')
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)
        self.assertFalse(self.engine.marker('test').exists())

    def test_changed_files_and_used_tokens_cannot_commit(self):
        result=self.preview()
        self.paths['yaml'].write_text(self.paths['yaml'].read_text()+' ')
        with self.assertRaisesRegex(ValueError,'地图已变化'):
            self.engine.commit('test',result['preview_id'])
        with self.assertRaisesRegex(ValueError,'不存在'):
            self.engine.commit('test',result['preview_id'])

    def test_expired_preview_cannot_commit(self):
        result=self.preview()
        self.engine._pending['expires']=0
        with self.assertRaises(ValueError): self.engine.commit('test',result['preview_id'])
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)

    def test_partial_install_rolls_back_full_bundle(self):
        result=self.preview()
        real=self.engine._replace
        calls=[]
        def replace(source,destination):
            # Let durable backup writes finish; fail the third target install.
            if Path(destination) in self.paths.values():
                calls.append(destination)
                if len(calls)==3: raise OSError('injected install failure')
            return real(source,destination)
        with patch.object(self.engine,'_replace',side_effect=replace):
            with self.assertRaises(OSError): self.engine.commit('test',result['preview_id'])
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)
        self.assertFalse(self.engine.marker('test').exists())

    def test_pending_crash_marker_blocks_preview_and_restore_recovers(self):
        result=self.preview()
        self.engine.commit('test',result['preview_id'])
        root=self.engine._backup_root('test')
        backup=root/json.loads((root/'latest.json').read_text())['id']
        manifest=json.loads((backup/'manifest.json').read_text())
        manifest['status']='pending'
        (backup/'manifest.json').write_text(json.dumps(manifest))
        self.engine._json(self.engine.marker('test'),{'backup_id':manifest['id']})
        with self.assertRaisesRegex(ValueError,'事务未完成'): self.preview()
        self.engine.restore('test')
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)

    def test_restore_refuses_to_overwrite_later_map_changes(self):
        result=self.preview(); self.engine.commit('test',result['preview_id'])
        self.paths['yaml'].write_text(self.paths['yaml'].read_text()+' ')
        with self.assertRaisesRegex(ValueError,'又被修改'): self.engine.restore('test')

    def test_interrupted_restore_is_retryable_and_keeps_marker(self):
        result=self.preview(); self.engine.commit('test',result['preview_id'])
        real=self.engine._replace
        calls=[]
        def replace(source,destination):
            calls.append(destination)
            if len(calls)==2: raise OSError('injected recovery failure')
            return real(source,destination)
        with patch.object(self.engine,'_replace',side_effect=replace):
            with self.assertRaises(OSError): self.engine.restore('test')
        self.assertTrue(self.engine.marker('test').exists())
        self.engine.restore('test')
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)

    def test_rollback_failure_blocks_map_until_backup_recovery(self):
        result=self.preview(); real=self.engine._replace
        calls=[]
        def replace(source,destination):
            if Path(destination) in self.paths.values():
                calls.append(destination)
                if len(calls) >= 3: raise OSError('injected install and rollback failure')
            return real(source,destination)
        with patch.object(self.engine,'_replace',side_effect=replace):
            with self.assertRaisesRegex(ValueError,'回滚不完整'):
                self.engine.commit('test',result['preview_id'])
        self.assertTrue(self.engine.marker('test').exists())
        with self.assertRaises(ValueError): self.preview()
        self.engine.restore('test')
        self.assertEqual(self.engine.fingerprints(self.paths),self.before)

    def test_pending_backup_without_matching_marker_cannot_overwrite_map(self):
        result=self.preview(); self.engine.commit('test',result['preview_id'])
        root=self.engine._backup_root('test')
        backup=root/json.loads((root/'latest.json').read_text())['id']
        manifest=json.loads((backup/'manifest.json').read_text())
        manifest['status']='pending'
        (backup/'manifest.json').write_text(json.dumps(manifest))
        after=self.engine.fingerprints(self.paths)
        with self.assertRaisesRegex(ValueError,'事务标记缺失'): self.engine.restore('test')
        self.assertEqual(self.engine.fingerprints(self.paths),after)

    def test_write_lease_is_exclusive_and_released(self):
        self.assertFalse(saved_map_write_busy(self.clouds))
        with saved_map_lock(self.clouds):
            self.assertTrue(saved_map_write_busy(self.clouds))
            with self.assertRaises(ValueError):
                with saved_map_lock(self.clouds): pass
        self.assertFalse(saved_map_write_busy(self.clouds))


class CleanupAdapterTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).resolve().parents[1]/'lzp_service/map_switcher_service.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MapSwitcherService')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='handle_edit_map_cloud')
        namespace=dict(threading=threading,json=json,saved_map_lock=saved_map_lock,
                       ControlLaunch=NS(Request=NS))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'adapter','exec'),namespace)
        self.method=namespace[method.name]
        root=Path(__file__).resolve().parents[6]/'tmp'
        self.temp=tempfile.TemporaryDirectory(prefix='map-adapter-tests-',dir=root)
        self.addCleanup(self.temp.cleanup)
        future=NS(add_done_callback=lambda cb:cb(None),result=lambda:NS(success=True))
        self.engine=Mock()
        self.engine.preview.return_value={'message':'preview only','preview_id':'token'}
        self.engine.commit.return_value={'message':'saved'}
        self.engine.restore.return_value={'message':'restored'}
        self.node=NS(cloud_edit_lock=threading.Lock(),cloud_edit_busy=False,
            pcdmap_directory=self.temp.name,navigation_active=False,pending_map_name=None,
            _safe_map_name=lambda n:n,map_cache_lock=threading.RLock(),map_load_lock=threading.RLock(),
            loaded_maps_cache={},active_map_name='old',active_map_load_id='old-id',
            publish_current_map_info=Mock(),cloud_editor=self.engine,get_logger=lambda:Mock(),
            restart_launch_cli=NS(service_is_ready=lambda:True,call_async=Mock(return_value=future)))
        self.request=NS(action='preview',map_name='test',indices=[1],minimum_height_m=.15,
                        maximum_height_m=.8,preview_id='token')

    def test_stop_ack_and_exclusive_lease_before_engine_preview(self):
        self.engine.preview.side_effect=lambda *a: {'message':'ok','leased':saved_map_write_busy(self.temp.name)}
        response=self.method(self.node,self.request,NS())
        self.assertTrue(response.success,response.message)
        self.assertTrue(json.loads(response.result_json)['leased'])
        self.assertIsNone(self.node.active_map_name)
        self.assertEqual(self.node.active_map_load_id,'')
        self.node.restart_launch_cli.call_async.assert_called_once()
        self.engine.commit.assert_not_called()
        self.assertFalse(saved_map_write_busy(self.temp.name))

    def test_stop_failure_dirty_edit_active_navigation_or_pending_load_does_not_edit(self):
        for condition in ('stop','dirty','active','load'):
            with self.subTest(condition=condition):
                self.node.navigation_active=condition=='active'
                self.node.pending_map_name='test' if condition=='load' else None
                self.node.loaded_maps_cache={'test':{'has_changes':True}} if condition=='dirty' else {}
                self.node.restart_launch_cli.call_async.return_value=NS(
                    add_done_callback=lambda cb:cb(None),result=lambda:NS(success=condition!='stop'))
                response=self.method(self.node,self.request,NS())
                self.assertFalse(response.success)
                self.engine.preview.assert_not_called()
                self.assertFalse(self.node.cloud_edit_busy)

    def test_commit_invalidates_cache_and_live_map_identity(self):
        self.request.action='commit'
        self.node.loaded_maps_cache={'test':{'has_changes':False}}
        response=self.method(self.node,self.request,NS())
        self.assertTrue(response.success,response.message)
        self.assertNotIn('test',self.node.loaded_maps_cache)
        self.assertIsNone(self.node.active_map_name)
        self.engine.commit.assert_called_once_with('test','token')

    def test_linked_preview_routes_to_engine_without_invalidating_cached_map(self):
        self.request.action='preview_linked'
        self.node.loaded_maps_cache={'test':{'has_changes':False}}
        response=self.method(self.node,self.request,NS())
        self.assertTrue(response.success,response.message)
        self.engine.preview.assert_called_once_with('test',[1],.15,.8,linked=True)
        self.assertIn('test',self.node.loaded_maps_cache)

    def test_pending_marker_blocks_production_map_qualification(self):
        source=Path(__file__).resolve().parents[1]/'lzp_service/map_switcher_service.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MapSwitcherService')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='validate_localization_map')
        namespace={'os':os}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'qualification','exec'),namespace)
        (Path(self.temp.name)/'.test.cloud-edit-pending').write_text('{}')
        with self.assertRaisesRegex(ValueError,'事务未完成'):
            namespace[method.name](self.node,'test','unused')


if __name__ == '__main__':
    unittest.main()
