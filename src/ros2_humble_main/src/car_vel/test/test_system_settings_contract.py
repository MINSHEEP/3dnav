#!/usr/bin/env python3
"""Deployment and frontend contracts for the host power settings page."""

from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FRONTEND_ROOT = PACKAGE_ROOT / 'car_vel' / 'web_frontend'
WORKSPACE_SRC = PACKAGE_ROOT.parent
LZP_ROOT = WORKSPACE_SRC / 'nav_service' / 'lzp_service'


class SystemSettingsContractTests(unittest.TestCase):
    def test_settings_page_has_fixed_services_and_typed_confirmation(self):
        page = (FRONTEND_ROOT / 'settings.html').read_text('utf-8')
        self.assertIn("'/system/reboot'", page)
        self.assertIn("'/system/poweroff'", page)
        self.assertIn("label: '重启'", page)
        self.assertIn("label: '关机'", page)
        self.assertIn('请输入“${action.label}”', page)
        self.assertIn('UI.TYPES.trigger', page)
        self.assertNotIn('window.confirm(', page)

    def test_every_current_topbar_links_to_settings(self):
        pages = [
            'mapping.html', 'map_editor.html', 'navigation.html',
            'campus_navigation.html', 'inspection.html',
            'inspection_reports.html', 'recognition_records.html',
            'remote_control.html', 'camera.html', 'settings.html',
        ]
        for name in pages:
            page = (FRONTEND_ROOT / name).read_text('utf-8')
            nav = re.search(r'<nav class="page-nav".*?</nav>', page, re.S)
            self.assertIsNotNone(nav, name)
            self.assertEqual(nav.group(0).count('href="settings.html"'), 1, name)
        mapping = (FRONTEND_ROOT / 'mapping.html').read_text('utf-8')
        self.assertRegex(
            mapping,
            r'<a[^>]*data-leave-mapping[^>]*href="settings\.html"|'
            r'<a[^>]*href="settings\.html"[^>]*data-leave-mapping',
        )

    def test_ros_node_is_installed_and_started(self):
        setup_text = (LZP_ROOT / 'setup.py').read_text('utf-8')
        service_text = (
            LZP_ROOT / 'lzp_service' / 'system_power_service.py'
        ).read_text('utf-8')
        launch_text = (
            PACKAGE_ROOT / 'launch' / 'launch_manager.launch.py'
        ).read_text('utf-8')
        self.assertIn(
            'system_power_service = lzp_service.system_power_service:main',
            setup_text,
        )
        self.assertIn("executable='system_power_service'", launch_text)
        self.assertIn("'system_power_enabled'", launch_text)
        self.assertIn("'system_power_delay_sec'", launch_text)
        self.assertIn('self.create_timer(', service_text)
        self.assertIn('self._controller.status()', service_text)
        self.assertNotRegex(
            launch_text,
            r"executable='system_power_service'[\s\S]{0,240}respawn\s*=\s*True",
        )

    def test_sudoers_installer_grants_only_two_exact_commands(self):
        installer = (
            PACKAGE_ROOT / 'scripts' / 'install_system_power_permissions.sh'
        ).read_text('utf-8')
        self.assertIn('/usr/bin/systemctl reboot', installer)
        self.assertIn('/usr/bin/systemctl poweroff', installer)
        self.assertIn('visudo -cf', installer)
        self.assertIn('0440', installer)
        self.assertNotIn('/usr/bin/systemctl *', installer)
        self.assertNotIn('NOPASSWD: ALL', installer)


if __name__ == '__main__':
    unittest.main()
