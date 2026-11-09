"""Exercise terminal setup using disposable PTYs; never open a physical adapter."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'connect.sh'


class SetupArguments(unittest.TestCase):
    def test_help_and_invalid_arguments_need_no_device(self):
        result = subprocess.run([str(SCRIPT), '--help'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn('No motor', result.stdout)
        for arguments in (['--domain', '999'], ['--wait', '-1'], ['--ros-distro', '../bad'], ['--port']):
            with self.subTest(arguments=arguments):
                result = subprocess.run([str(SCRIPT), *arguments], capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)


@unittest.skipUnless((ROOT/'install/setup.bash').exists() and (ROOT/'.venv/bin/python').exists()
                     and Path('/opt/ros/jazzy/setup.bash').exists(), 'Requires prepared Jazzy workspace')
class TerminalSetup(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)

    def tearDown(self):
        # No setup command may send any bytes to the simulated motor bus.
        self.assertFalse(select.select([self.master], [], [], 0)[0], 'Setup sent data to motor port')
        os.close(self.master); os.close(self.slave)

    def run_script(self, *arguments, env=None):
        return subprocess.run([str(SCRIPT), '--port', self.port, '--wait', '0', *arguments],
            capture_output=True, text=True, timeout=30, env=env)

    def test_executed_setup_exports_commands_and_ros_environment(self):
        result = self.run_script('--domain', '73', '--', '/bin/bash', '--noprofile', '--norc', '-c',
            'type -t servo chain arm calibrate; calibrate --status; arm --help; '
            '/usr/bin/python3 -c \'import os,json; print(json.dumps({k:os.environ[k] for k in '
            '["PORT","ROS_DOMAIN_ID","ROBOT_SOFTWARE_ROOT","AMENT_PREFIX_PATH"]}))\'')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('function\nfunction\nfunction\nfunction', result.stdout)
        data = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(data['PORT'], self.port)
        self.assertEqual(data['ROS_DOMAIN_ID'], '73')
        self.assertEqual(data['ROBOT_SOFTWARE_ROOT'], str(ROOT))
        self.assertIn('install/pinchy_bringup', data['AMENT_PREFIX_PATH'])

    def test_sourced_setup_is_repeatable_and_leaves_venv_out_of_ros_path(self):
        result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c',
            'source "$3/.venv/bin/activate"; '
            'source "$1" --port "$2" --wait 0 --domain 74; '
            'source "$1" --port "$2" --wait 0 --domain 74; '
            'test -z "${VIRTUAL_ENV:-}" && type -t servo && '
            'python3 -c \'import sys; print(sys.executable)\'',
            'setup-test', str(SCRIPT), self.port, str(ROOT)],
            capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertNotIn(str(ROOT/'.venv'), result.stdout.splitlines()[-1])
        self.assertEqual(result.stdout.count('Robot terminal ready:'), 2)

    def test_inherited_venv_does_not_break_ros_python(self):
        env = dict(os.environ, VIRTUAL_ENV=str(ROOT/'.venv'), PATH=str(ROOT/'.venv/bin')+':'+os.environ['PATH'])
        result = self.run_script('--', '/usr/bin/env', 'python3', '-c',
            'import os,sys; assert "VIRTUAL_ENV" not in os.environ; print(sys.executable)', env=env)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertNotIn(str(ROOT/'.venv'), result.stdout.splitlines()[-1])

    def test_missing_explicit_port_fails_without_selecting_another_adapter(self):
        result = subprocess.run([str(SCRIPT), '--port', '/dev/pinchy-no-such-port', '--wait', '0', '--no-shell'],
            capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 1)
        self.assertIn('Adapter not found', result.stderr)

    def test_ros_only_setup_does_not_require_an_adapter(self):
        result = subprocess.run([str(SCRIPT), '--port', '/dev/pinchy-no-such-port', '--no-port',
            '--domain', '0', '--no-shell'], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('USB discovery/access check skipped', result.stdout)

    @unittest.skipIf(os.geteuid() == 0, 'Root bypasses the permission check')
    def test_permission_fix_is_requested_only_when_needed(self):
        with tempfile.TemporaryDirectory(prefix='pinchy-connect-test-') as folder:
            log = Path(folder)/'sudo.txt'
            os.fchmod(self.slave, 0)
            result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c',
                'source "$1" --help >/dev/null; '
                '_robot_connect_sudo() { printf "%s\\n" "$*" >> "$TEST_SUDO_LOG"; chmod u+rw -- "${@: -1}"; }; '
                '_robot_connect_main --port "$2" --wait 0 --no-shell && '
                '_robot_connect_main --port "$2" --wait 0 --no-shell',
                'setup-test', str(SCRIPT), self.port], env=dict(os.environ, TEST_SUDO_LOG=str(log)),
                capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertEqual(len(log.read_text().splitlines()), 1)
            self.assertTrue(log.read_text().startswith('setfacl -m '))

    def test_executing_without_command_opens_a_ready_interactive_shell(self):
        master, slave = pty.openpty()
        process = subprocess.Popen([str(SCRIPT), '--port', self.port, '--wait', '0'],
            stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        output = bytearray()
        try:
            deadline = time.monotonic()+20
            while process.poll() is None and time.monotonic() < deadline:
                if select.select([master], [], [], 0.1)[0]: output.extend(os.read(master, 65536))
                if b'Opening a prepared robot shell' in output:
                    os.write(master, b'type -t servo; echo CONNECT_SHELL_OK; exit\n')
                    break
            process.wait(timeout=10)
            while select.select([master], [], [], 0)[0]: output.extend(os.read(master, 65536))
            self.assertEqual(process.returncode, 0, output.decode(errors='replace'))
            self.assertIn(b'function', output)
            self.assertIn(b'CONNECT_SHELL_OK', output)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5)
            os.close(master); os.close(slave)


if __name__ == '__main__': unittest.main()
