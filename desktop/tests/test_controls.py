"""Regresiones del control sin ventanas, conexiones ni hardware."""
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import spotmicro_gui as gui


def variable(value):
    return Mock(get=Mock(return_value=value))


def controller():
    app = gui.RobotGUI.__new__(gui.RobotGUI)
    app.calibration_vars = [[{
        'offset': variable(0), 'invert': variable(False),
        'min': variable(0), 'max': variable(180),
    } for _ in range(3)] for _ in range(4)]
    app.error_label = Mock()
    return app


class ServoCommands(unittest.TestCase):
    def setUp(self):
        self.app = controller()

    def packet(self, first):
        angles = [[math.pi / 2] * 3 for _ in range(4)]
        angles[0][0] = math.radians(first)
        return self.app.get_calibrated_angles_for_esp32(angles)

    def test_negative_angle_is_rejected_not_sent_as_180(self):
        self.assertIsNone(self.packet(-1))
        with patch.object(gui, 'connect_ws') as connect:
            self.assertFalse(gui.send_angles(None))
            connect.assert_not_called()

    def test_endpoints_and_equivalent_ik_turns(self):
        for angle, expected in [(0, 0), (180, 180), (450, 90), (-270, 90)]:
            with self.subTest(angle=angle):
                self.assertAlmostEqual(self.packet(angle)[0], expected)

    def test_nonfinite_or_out_of_range_packet_rejected(self):
        for angle in [181, math.nan, math.inf]:
            with self.subTest(angle=angle):
                self.assertIsNone(self.packet(angle))
        self.assertIsNone(self.app.get_calibrated_angles_for_esp32([[0, 0, 0]]))

    def test_per_servo_limits_apply_to_whole_packet(self):
        self.app.calibration_vars[0][0]['max'] = variable(80)
        self.assertIsNone(self.packet(90))

    def test_differential_control_does_not_send_outside_limits(self):
        app = self.app
        app.diff_platform_vars = {
            'position': {a: variable(0) for a in ['x', 'y', 'z']},
            'rotation': {a: variable(0) for a in ['roll', 'pitch', 'yaw']},
        }
        app.kin = Mock()
        app.kin.plot_robot.return_value = ([[math.pi / 2] * 3 for _ in range(4)], [])
        app.calibration_vars[2][1]['max'] = variable(80)
        app.diff_ax, app.diff_canvas = Mock(), Mock()
        app.current_leg_positions = [[0, 0, 0]] * 4
        app.update_differential_angle_display = Mock()
        socket = Mock()
        with patch.object(gui, 'send_to_esp32_flag', variable(True), create=True), \
                patch.object(gui, 'ws', socket), patch.object(gui, 'ws_connected', True):
            app.update_differential_visualization()
        socket.send.assert_not_called()

    def test_default_calibration_accepts_neutral_pose(self):
        from spot_dance import Pose
        calibration = json.loads((Path(__file__).resolve().parents[2] /
                                  'calibration/offset-default.json').read_text())
        self.app.calibration_vars = [[{key: variable(value) for key, value in servo.items()}
                                      for servo in leg['servos']] for leg in calibration['legs']]
        kin = gui.Kinematics()
        feet, rot, body = Pose().absolute()
        angles = [kin.leg_IK(feet[i], rot, i, False, body)[:3] for i in range(4)]
        self.assertIsNotNone(self.app.get_calibrated_angles_for_esp32(angles))


class FailsafeHeartbeat(unittest.TestCase):
    def test_ping_only_while_sending_and_connected(self):
        for enabled, connected, sent in [(True, True, True), (False, True, False),
                                         (True, False, False)]:
            with self.subTest(enabled=enabled, connected=connected):
                socket = Mock()
                with patch.object(gui, 'send_to_esp32_flag', variable(enabled), create=True), \
                        patch.object(gui, 'ws', socket), patch.object(gui, 'ws_connected', connected), \
                        patch.object(gui, 'connect_ws') as connect:
                    self.assertEqual(gui.send_heartbeat(), sent)
                connect.assert_not_called()
                if sent:
                    socket.send.assert_called_once_with('cmd:ping')
                else:
                    socket.send.assert_not_called()

    def test_heartbeat_faster_than_esp32_timeout(self):
        config = (Path(__file__).resolve().parents[2] /
                  'firmware/spotmicro_esp32/config.h').read_text(encoding='utf-8')
        timeout = int(config.split('FAILSAFE_TIMEOUT_MS =')[1].split(';')[0])
        self.assertLessEqual(gui.HEARTBEAT_MS * 3, timeout)


class ImuResponses(unittest.TestCase):
    def test_command_starts_receiver_without_enabling_telemetry(self):
        socket, handler = Mock(), Mock()
        with patch.object(gui, 'send_to_esp32_flag', variable(True), create=True), \
                patch.object(gui, 'ws', socket), patch.object(gui, 'ws_connected', True), \
                patch.object(gui, 'mpu_handler', handler):
            self.assertTrue(gui.send_command('cmd:get_offsets'))
        handler.start_mpu_listener.assert_called_once_with(request_data=False)
        socket.send.assert_called_once_with('cmd:get_offsets')

    def test_gui_tick_delivers_responses_on_main_thread(self):
        app = controller()
        app.root = Mock()
        app._process_mpu_data = Mock()
        handler = gui.MPU6050Handler()
        handler._process_mpu_data('{"status":"Offsets updated"}')
        with patch.object(gui, 'mpu_handler', handler):
            app.update_mpu_display()
        self.assertEqual(json.loads(app._process_mpu_data.call_args.args[0]),
                         {'status': 'Offsets updated'})
        app.root.after.assert_called_once()

    def test_offsets_and_acks_survive_telemetry_queue_drain(self):
        handler = gui.MPU6050Handler()
        reply = {'status': 'offsets_data', 'accel_offsets': [1, 2, 3], 'gyro_offsets': [4, 5, 6]}
        handler._process_mpu_data(json.dumps(reply))
        handler._process_mpu_data('{"status":"Offsets updated"}')
        handler.get_latest_data()
        self.assertEqual(handler.response_queue.get_nowait(), reply)
        self.assertEqual(handler.response_queue.get_nowait()['status'], 'Offsets updated')

    def test_reply_with_status_still_displays_offsets(self):
        app = controller()
        with patch.object(gui.messagebox, 'showinfo') as show:
            app._process_mpu_data(json.dumps({'status': 'offsets_data',
                'accel_offsets': [1, 2, 3], 'gyro_offsets': [4, 5, 6]}))
        show.assert_called_once()
        self.assertIn('X: 1', show.call_args.args[1])

    def test_set_offsets_uses_shared_sender_and_waits_for_ack(self):
        app = controller()
        app.offset_vars = {f'{kind}_{axis}': variable(0) for kind in ['acelerómetro', 'giroscopio']
                           for axis in ['x', 'y', 'z']}
        with patch.object(gui, 'send_to_esp32_flag', variable(True), create=True), \
                patch.object(gui, 'send_command', return_value=True) as send, \
                patch.object(gui.messagebox, 'showinfo') as info, \
                patch.object(gui.messagebox, 'showerror') as error:
            app.set_imu_offsets()
        send.assert_called_once_with('cmd:set_offsets:0,0,0,0,0,0')
        info.assert_not_called()
        error.assert_not_called()


if __name__ == '__main__':
    unittest.main()
