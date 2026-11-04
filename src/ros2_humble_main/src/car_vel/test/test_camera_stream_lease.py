"""Only camera demand changes; no remote or robot movement is enabled."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from car_vel.camera_stream_lease import CameraStreamLease


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.now=0.0
        self.sent=[]
        self.lease=CameraStreamLease(lambda enabled:self.sent.append(enabled),clock=lambda:self.now)
        self.a='a'*32; self.b='b'*32
    def request(self,operation,token=None):
        return self.lease.request(dict(operation=operation,token=token or self.a))
    def test_camera_alone_does_not_enable_remote(self):
        self.assertTrue(self.request('acquire')['success'])
        self.assertEqual(self.sent,[True])
        self.assertFalse(self.lease._remote)
        self.request('release'); self.assertEqual(self.sent,[True,False])
    def test_remote_exit_does_not_stop_managed_camera(self):
        self.lease.remote(True); self.request('acquire')
        self.lease.remote(False)
        self.assertEqual(self.sent,[True])
        self.request('release'); self.assertEqual(self.sent,[True,False])
    def test_release_does_not_stop_active_remote_camera(self):
        self.request('acquire'); self.lease.remote(True)
        self.request('release'); self.assertEqual(self.sent,[True])
        self.lease.remote(False); self.assertEqual(self.sent,[True,False])
    def test_expiry_recovers_after_gateway_crash_and_renew_is_idempotent(self):
        self.request('acquire'); self.now=5; self.request('acquire')
        self.now=9; self.lease.tick(); self.assertEqual(self.sent,[True])
        self.now=14; self.lease.tick(); self.assertEqual(self.sent,[True,False])
    def test_clients_are_independent_and_close_revokes_all(self):
        self.request('acquire'); self.request('acquire',self.b)
        self.request('release'); self.assertEqual(self.sent,[True])
        self.lease.close(); self.assertEqual(self.sent,[True,False])
        with self.assertRaises(RuntimeError): self.request('acquire')
    def test_invalid_payload_never_sends_and_failed_send_is_not_reported_enabled(self):
        with self.assertRaises(ValueError): self.request('velocity')
        self.assertEqual(self.sent,[])
        lease=CameraStreamLease(lambda enabled:False)
        result=lease.request(dict(operation='acquire',token=self.a))
        self.assertFalse(result['success']); self.assertFalse(result['enabled'])


if __name__=='__main__': unittest.main()
