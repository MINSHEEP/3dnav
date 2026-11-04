"""Offline production preview contracts; fake cameras/ROS, no robot I/O."""
import ast
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import copy
from http.server import ThreadingHTTPServer
import io
import json
from pathlib import Path
import struct
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import urlopen
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'hikvision_camera')]
from web_preview import PreviewCache, PROFILES, encode_preview
from car_vel.web_cloud_preview import thin_cloud, CLOUD_TOPICS
from camera_web import CameraWebServer, CameraRequestHandler


def jpeg(width=1280, height=720):
    output = io.BytesIO()
    Image.new('RGB', (width, height), 'green').save(output, format='JPEG')
    return output.getvalue()


def pil_encoder(data, profile):
    image = Image.open(io.BytesIO(data))
    image.thumbnail((profile.width, profile.height))
    output = io.BytesIO()
    image.save(output, format='JPEG', quality=profile.quality)
    return output.getvalue()


def cloud(points, *, endian='<', padded=False):
    fields = [NS(name=name, datatype=7, count=1, offset=i*4)
              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    data = b''.join(struct.pack(endian+'ffff', *p) for p in points)
    width, height, row = len(points), 1, len(points)*16
    if padded:
        width, height, row = 2, len(points)//2, 36
        data = b''.join(data[i:i+32] + b'PAD!' for i in range(0, len(data), 32))
    return NS(width=width, height=height, point_step=16, row_step=row,
              fields=fields, data=data, is_bigendian=endian=='>', is_dense=False,
              header=NS(frame_id='map', stamp=NS(sec=17, nanosec=42)))


class PreviewTests(unittest.TestCase):
    def test_shared_encode_is_bounded_by_profile_rate(self):
        clock = NS(now=0.)
        read, encode = Mock(return_value=b'original'), Mock(return_value=b'preview')
        cache = PreviewCache({'raw':read}, encoder=encode, clock=lambda:clock.now)
        with ThreadPoolExecutor(8) as pool:
            results = list(pool.map(lambda _: cache.frame('raw', 'remote'), range(20)))
        self.assertEqual(results, [(1, b'preview')]*20)
        self.assertEqual(read.call_count, 1)
        clock.now = .201
        self.assertEqual(cache.frame('raw', 'remote')[0], 2)
        self.assertEqual(encode.call_count, 2)
        self.assertEqual(len(cache.entries), 2)

    def test_switch_invalidates_cache_immediately_and_drops_inflight_old_frame(self):
        state = NS(generation=0, image=b'legacy')
        cache = PreviewCache({'thermal':lambda:state.image}, clock=lambda:0.,
            encoder=lambda image,_profile:image, generation=lambda:state.generation)
        self.assertEqual(cache.frame('thermal','remote'), (1,b'legacy'))
        state.generation, state.image = 1, b'c12'
        self.assertEqual(cache.frame('thermal','remote'), (2,b'c12'))
        def switching_encoder(image, _profile):
            state.generation += 1
            return image
        cache.encoder = switching_encoder
        state.generation += 1
        with self.assertRaisesRegex(RuntimeError, '相机链路已切换'):
            cache.frame('thermal','remote')
        self.assertEqual(cache.entries['thermal','remote']['sequence'], 2)

    def test_three_sources_and_two_profiles_do_not_change_originals(self):
        original = jpeg()
        cache = PreviewCache({key:lambda:original for key in ('raw','yolo','thermal')}, encoder=pil_encoder)
        for source in ('raw','yolo','thermal'):
            self.assertEqual(Image.open(io.BytesIO(cache.frame(source,'remote')[1])).size, (640,360))
            self.assertEqual(Image.open(io.BytesIO(cache.frame(source,'lan')[1])).size, (1280,720))
        self.assertEqual(Image.open(io.BytesIO(original)).size, (1280,720))
        thermal = PreviewCache({'thermal':lambda:jpeg(256,192)}, encoder=pil_encoder)
        self.assertEqual(Image.open(io.BytesIO(thermal.frame('thermal','remote')[1])).size,(256,192))

    def test_production_codec_keeps_aspect_ratio_and_does_not_upscale(self):
        for height,width,expected in ((720,1280,(640,360)),(192,256,None),(1280,720,(202,360))):
            fake = NS(IMREAD_COLOR=1, INTER_AREA=3, IMWRITE_JPEG_QUALITY=1,
                      imdecode=lambda *args:np.zeros((height,width,3),dtype=np.uint8),
                      resize=Mock(return_value=np.zeros((10,10,3),dtype=np.uint8)),
                      imencode=Mock(return_value=(True,np.frombuffer(b'jpeg',dtype=np.uint8))))
            with patch.dict(sys.modules, {'cv2':fake}):
                self.assertEqual(encode_preview(b'original',PROFILES['remote']),b'jpeg')
            if expected: self.assertEqual(fake.resize.call_args.args[1],expected)
            else: fake.resize.assert_not_called()
            self.assertEqual(fake.imencode.call_args.args[2], [1,55])

    def test_error_does_not_refresh_stale_cache_and_unknown_profiles_rejected(self):
        now = NS(value=0.)
        reader = Mock(return_value=b'full')
        cache = PreviewCache({'raw':reader},encoder=lambda data,p:data,clock=lambda:now.value)
        cache.frame('raw','remote'); now.value=1.
        reader.side_effect=RuntimeError('offline')
        with self.assertRaises(RuntimeError): cache.frame('raw','remote')
        self.assertEqual(cache.entries['raw','remote']['sequence'],1)
        with self.assertRaises(ValueError): cache.frame('raw','arbitrary')


class CloudTests(unittest.TestCase):
    def test_voxels_preserve_source_header_and_intensity_without_mutation(self):
        source=cloud([(0,0,0,10),(.01,.01,.01,20),(1,2,3,30)])
        before=copy.deepcopy(source)
        result=thin_cloud(source)
        self.assertEqual(result.width,2)
        self.assertEqual(result.header,before.header)
        self.assertEqual(source,before)
        self.assertEqual(struct.unpack('<ffff',result.data[:16]),(0,0,0,10))
        self.assertEqual(struct.unpack('<ffff',result.data[16:]),(1,2,3,30))

    def test_global_point_and_byte_caps(self):
        source=cloud([(float(i),0,0,1) for i in range(30000)])
        result=thin_cloud(source)
        self.assertEqual(result.width,20000)
        self.assertEqual(len(result.data),20000*16)
        self.assertEqual(result.row_step,len(result.data))

    def test_padded_big_endian_and_nonfinite_roi(self):
        source=cloud([(0,0,0,1),(1,0,0,2),(100,0,0,3),(float('nan'),0,0,4)],endian='>',padded=True)
        result=thin_cloud(source,center=(0,0),radius=15)
        self.assertEqual(result.width,2)
        self.assertTrue(result.is_bigendian)
        self.assertEqual(struct.unpack('>ffff',result.data[16:]),(1,0,0,2))

    def test_wide_records_have_byte_limit_in_addition_to_point_limit(self):
        source=cloud([(float(i),0,0,1) for i in range(12000)])
        source.data=b''.join(source.data[i:i+16]+b'\x00'*48 for i in range(0,len(source.data),16))
        source.point_step=64;source.row_step=source.width*64
        result=thin_cloud(source)
        self.assertEqual(result.width,8192)
        self.assertLessEqual(len(result.data),512*1024)

    def test_empty_and_invalid_do_not_fabricate_ground(self):
        self.assertEqual(thin_cloud(cloud([])).width,0)
        source=cloud([(1,2,3,4)]);source.data=b''
        with self.assertRaises(ValueError): thin_cloud(source)
        source=cloud([(1,2,3,4)]);source.fields[0].offset=100
        with self.assertRaises(ValueError): thin_cloud(source)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.original=jpeg()
        self.gateway=NS(next_frame=Mock(return_value=(4,self.original)),
            snapshot=Mock(return_value=self.original),
            get_yolo_frame=Mock(return_value=(3,self.original)),
            thermal_frame=Mock(return_value=jpeg(256,192)))
        self.gateway.display_thermal_frame = self.gateway.thermal_frame
        self.gateway.display_generation = lambda: 0
        self.server=CameraWebServer(('127.0.0.1',0),CameraRequestHandler)
        self.server.gateway=self.gateway
        self.server.browser_previews.encoder=pil_encoder
        self.worker=threading.Thread(target=lambda:self.server.serve_forever(.02),daemon=True)
        self.worker.start()
        self.base=f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.worker.join(2)

    def test_original_and_preview_endpoints_are_independent(self):
        with urlopen(self.base+'/api/snapshot',timeout=2) as r:
            self.assertEqual(r.read(),self.original)
        for source in ('raw','yolo','thermal'):
            with urlopen(self.base+'/api/preview/frame.jpg?source='+source,timeout=2) as r:
                size=Image.open(io.BytesIO(r.read())).size
                self.assertEqual(size,(256,192) if source=='thermal' else (640,360))
        self.gateway.next_frame.assert_called_with(-1)
        self.gateway.get_yolo_frame.assert_called_with(0)

    def test_unavailable_source_returns_http_error_before_stream_headers(self):
        self.gateway.thermal_frame.side_effect=RuntimeError('thermal offline')
        with self.assertRaises(HTTPError) as error:
            urlopen(self.base+'/api/preview/stream?source=thermal',timeout=2)
        self.assertEqual(error.exception.code,503)
        self.assertIn('thermal offline',error.exception.read().decode())
        self.assertTrue(self.server.stream_slots.acquire(blocking=False))
        self.server.stream_slots.release()

    def test_actual_mjpeg_response_contains_processed_jpeg(self):
        with urlopen(self.base+'/api/preview/stream?source=raw',timeout=2) as response:
            self.assertIn('multipart/x-mixed-replace',response.headers['Content-Type'])
            self.assertEqual(response.readline(),b'--frame\r\n')
            self.assertEqual(response.readline(),b'Content-Type: image/jpeg\r\n')
            length=int(response.readline().split(b':')[1])
            self.assertEqual(response.readline(),b'\r\n')
            image=response.read(length)
            self.assertEqual(Image.open(io.BytesIO(image)).size,(640,360))


class CloudNodeTests(unittest.TestCase):
    def setUp(self):
        self.now=0.
        self.retained=None
        class Node:
            def __init__(node,*args): node.publishers={};node.subscriptions={}
            def create_publisher(node, _type,name,_qos):
                publisher=NS(count=0,values=[],qos=_qos)
                publisher.get_subscription_count=lambda:publisher.count
                publisher.publish=lambda msg:publisher.values.append(msg)
                node.publishers[name]=publisher;return publisher
            def create_subscription(node,_type,name,callback,_qos):
                if (name=='/global_localization_map' and self.retained is not None
                        and getattr(_qos,'durability',0)==1):
                    callback(self.retained)
                node.subscriptions[name]=callback;return NS(topic=name,qos=_qos)
            def destroy_subscription(node,subscription): node.subscriptions.pop(subscription.topic,None)
            def create_timer(node,*args): pass
            def get_logger(node): return Mock()
        source=ast.parse((ROOT/'car_vel/web_cloud_preview_node.py').read_text(encoding='utf-8'))
        definition=next(n for n in source.body if isinstance(n,ast.ClassDef))
        from car_vel.web_cloud_preview import PROFILES as cloud_profiles
        scope=dict(Node=Node, PointCloud2=object,Odometry=object,CurrentMap=object,threading=threading,
            String=lambda **kw:NS(**kw),json=json,
            QoSProfile=lambda **kw:NS(**kw),ReliabilityPolicy=NS(RELIABLE=1,BEST_EFFORT=2),
            DurabilityPolicy=NS(TRANSIENT_LOCAL=1,VOLATILE=0),
            time=NS(monotonic=lambda:self.now),CLOUD_TOPICS=CLOUD_TOPICS,PROFILES=cloud_profiles,
            thin_cloud=thin_cloud)
        exec(compile(ast.Module(body=[definition],type_ignores=[]),'<production web cloud node>','exec'),scope)
        self.node=scope['WebCloudPreview']()

    def demand(self,topic='/global_mapping_cloud',profile='remote'):
        publisher=self.node.publishers['/web_preview/'+profile+topic]
        publisher.count=1;self.node._tick();return publisher

    def test_no_viewers_no_cloud_inputs_and_release_unsubscribes(self):
        self.node._tick()
        self.assertEqual(set(self.node.subscriptions),{'/Odometry','/global_localization_map',
            '/current_map_info','/global_localization_map/status'})
        out=self.demand();self.assertIn('/global_mapping_cloud',self.node.subscriptions)
        self.node._receive('/global_mapping_cloud',cloud([(0,0,0,1)]));self.node._tick()
        self.assertEqual(len(out.values),1)
        out.count=0;self.node._tick()
        self.assertNotIn('/global_mapping_cloud',self.node.subscriptions)
        self.assertEqual(self.node.latest,{})
        self.assertTrue(all(name.startswith('/web_preview/') for name in self.node.publishers))

    def test_saved_cloud_requests_history_even_before_any_browser(self):
        self.assertIn('/global_localization_map',self.node.inputs)
        qos=self.node.inputs['/global_localization_map'].qos
        self.assertEqual(qos.durability,1)
        self.assertEqual(qos.reliability,1)

    def test_late_node_consumes_retained_source_without_new_publish(self):
        self.retained=cloud([(1,2,0,7)])
        self.node=type(self.node)()
        out=self.demand('/global_localization_map')
        self.assertEqual(len(out.values),1)
        self.assertEqual(out.values[0].header.stamp,self.retained.header.stamp)

    def test_full_map_http_identity_requires_current_map_and_exact_icp_load(self):
        self.node._saved_map_current(NS(success=True,map_name='room',load_id='a'))
        self.assertEqual(self.node.saved_map_state(),{})
        self.node._saved_map_identity(NS(data=json.dumps(dict(map_name='room',load_id='a',stamp_ns='1000000001'))))
        self.assertEqual(self.node.saved_map_state()['stamp_ns'],'1000000001')
        self.node._saved_map_current(NS(success=True,map_name='room',load_id='b'))
        self.assertEqual(self.node.saved_map_state(),{},'same name cannot reuse an old ICP identity')
        self.node._saved_map_identity(NS(data='not JSON'))
        self.assertEqual(self.node.saved_map_state(),{})

    def test_saved_cloud_replays_without_changing_stamp_or_reencoding(self):
        topic='/global_localization_map'
        original=cloud([(0,0,0,1)])
        self.node._receive(topic,original)
        out=self.demand(topic)
        self.assertEqual(len(out.values),1)
        first=out.values[0]
        self.now=3.;self.node._tick()
        self.assertEqual(len(out.values),2,'volatile rosbridge must get another opportunity')
        self.assertIs(out.values[-1],first,'static replay must reuse encoded preview')
        self.assertEqual(first.header.stamp,original.header.stamp)
        out.count=0;self.node._tick()
        self.assertIn(topic,self.node.inputs,'retain static input without viewers')
        self.now=9.;self.node._tick();self.assertEqual(len(out.values),2)
        out.count=1;self.node._tick()
        self.assertEqual(len(out.values),3,'returning viewer gets current saved map')

    def test_preview_health_exposes_missing_input_and_viewer_counts(self):
        topic='/global_localization_map';self.demand(topic)
        self.now=2.;self.node._tick()
        out=self.node.publishers['/web_preview/status']
        state=json.loads(out.values[-1].data)
        self.assertTrue(state['topics'][topic]['input_subscribed'])
        self.assertFalse(state['topics'][topic]['has_frame'])
        self.assertEqual(state['topics'][topic]['viewers']['remote'],1)

    def test_new_saved_map_replaces_cached_preview(self):
        topic='/global_localization_map';out=self.demand(topic)
        lan=self.demand(topic,'lan')
        self.node._receive(topic,cloud([(0,0,0,1)]));self.node._tick()
        first=out.values[-1]
        lan.count=0
        next_map=cloud([(4,5,0,2)]);next_map.header.stamp.sec=18
        self.now=3.;self.node._receive(topic,next_map);self.node._tick()
        self.assertIsNot(out.values[-1],first)
        self.assertEqual(out.values[-1].header.stamp.sec,18)
        self.assertEqual(first.header.stamp.sec,17)
        self.assertNotIn((topic,'lan'),self.node.previews,'unused profile cannot retain previous full map')

    def test_one_output_error_does_not_block_other_topic_or_health(self):
        bad=self.demand('/global_mapping_cloud')
        good=self.demand('/global_localization_map')
        bad.publish=Mock(side_effect=ValueError('fake output error'))
        self.node._receive('/global_mapping_cloud',cloud([(0,0,0,1)]))
        self.node._receive('/global_localization_map',cloud([(0,0,0,1)]))
        self.now=2.;self.node._tick()
        self.assertEqual(len(good.values),1)
        status=json.loads(self.node.publishers['/web_preview/status'].values[-1].data)
        self.assertIn('fake output error',status['topics']['/global_mapping_cloud']['error'])

    def test_latest_slot_and_remote_period_not_old_frame_replay(self):
        out=self.demand()
        self.node._receive('/global_mapping_cloud',cloud([(0,0,0,1)]));self.node._tick()
        for i in range(1,5):
            self.now=i*.5
            self.node._receive('/global_mapping_cloud',cloud([(i,0,0,1)]));self.node._tick()
        self.assertEqual(len(out.values),1)
        self.now=3.;self.node._tick()
        self.assertEqual(struct.unpack('<ffff',out.values[-1].data)[0],4)
        self.node._tick();self.assertEqual(len(out.values),2)

    def test_roi_only_uses_matching_fresh_odometry(self):
        topic='/cloud_mapping_dynamic';out=self.demand(topic)
        self.node.pose=('odom',0,0,0.)
        message=cloud([(0,0,0,1),(100,0,0,1)])
        self.node._receive(topic,message);self.node._tick()
        self.assertEqual(out.values[-1].width,2,'mismatched frames must not crop arbitrarily')
        self.now=.6;self.node.pose=('map',0,0,.6)
        self.node._receive(topic,message);self.node._tick()
        self.assertEqual(out.values[-1].width,1)
        self.now=3.;self.node._receive(topic,message);self.node._tick()
        self.assertEqual(out.values[-1].width,2,'stale pose cannot define ROI')


if __name__=='__main__': unittest.main()
