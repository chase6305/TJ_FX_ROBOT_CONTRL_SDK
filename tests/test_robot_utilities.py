"""Exercise pure helpers without loading a robot SDK or connecting to hardware."""
import ctypes
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
with mock.patch.object(ctypes, 'CDLL', side_effect=AssertionError('Native SDK must not load')):
    spec = importlib.util.spec_from_file_location('offline_robot', ROOT / 'SDK_PYTHON/fx_robot.py')
    robot = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(robot)


class RobotUtilitiesTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'tool.txt'
        self.left = [float(i + 1) for i in range(16)]
        self.right = [float(i + 21) for i in range(16)]

    @staticmethod
    def row(values):
        return ','.join(str(value) for value in values)

    def read_rows(self):
        return [[float(value) for value in line.split(',')]
                for line in self.path.read_text(encoding='utf-8').splitlines()]

    def test_first_save_either_arm_initializes_both_rows(self):
        for arm, expected in [('A', [self.left, [0.0] * 16]),
                              ('B', [[0.0] * 16, self.left])]:
            with self.subTest(arm=arm):
                self.path.unlink(missing_ok=True)
                self.assertTrue(robot.update_text_file_simple(arm, self.left, self.path))
                self.assertEqual(self.read_rows(), expected)

    def test_empty_file_can_be_updated(self):
        self.path.touch()
        self.assertTrue(robot.update_text_file_simple('B', self.right, self.path))
        self.assertEqual(self.read_rows(), [[0.0] * 16, self.right])

    def test_single_row_without_newline_preserves_other_arm(self):
        self.path.write_text(self.row(self.left), encoding='utf-8')
        self.assertTrue(robot.update_text_file_simple('B', self.right, self.path))
        self.assertEqual(self.read_rows(), [self.left, self.right])

    def test_existing_two_rows_preserve_other_arm(self):
        self.path.write_text(self.row(self.left) + '\n' + self.row(self.right) + '\n', encoding='utf-8')
        self.assertTrue(robot.update_text_file_simple('A', [3.0] * 16, self.path))
        self.assertEqual(self.read_rows(), [[3.0] * 16, self.right])

    def test_invalid_update_does_not_change_file(self):
        self.path.write_bytes(b'original\n')
        for arm, values in [('C', self.left), ('A', [1.0] * 15)]:
            self.assertFalse(robot.update_text_file_simple(arm, values, self.path))
            self.assertEqual(self.path.read_bytes(), b'original\n')

    def test_io_error_returns_false(self):
        self.assertFalse(robot.update_text_file_simple('A', self.left, self.path / 'missing.txt'))

    def test_valid_csv_preserves_all_columns(self):
        self.path.write_text(self.row(self.left) + '\n' + self.row(self.right), encoding='utf-8')
        self.assertEqual(robot.read_csv_file_to_float_strict(self.path), [self.left, self.right])

    def test_csv_rejects_empty_fields_including_extra_separators(self):
        for values in [self.left[:4] + [''] + self.left[4:],
                       self.left + [''], [''] + self.left,
                       self.left[:7] + [''] + self.left[8:]]:
            with self.subTest(values=values):
                self.path.write_text(self.row(values) + '\n' + self.row(self.right), encoding='utf-8')
                self.assertEqual(robot.read_csv_file_to_float_strict(self.path), -1)

    def test_zero_row_retains_unset_arm_convention(self):
        self.path.write_text(self.row([0.0] * 16) + '\n' + self.row(self.right), encoding='utf-8')
        self.assertEqual(robot.read_csv_file_to_float_strict(self.path), ('line2', self.right))

    def test_bytes_hex_and_text_remain_supported(self):
        for value, expected in [(b'\x01\xff', b'\x01\xff'), ('0x1 ff', b'\x01\xff'), ('测试', '测试'.encode())]:
            with self.subTest(value=value):
                parsed = robot.identify_and_calculate_length(value)
                self.assertEqual(parsed['bytes_representation'], expected)
                self.assertEqual(parsed['length_bytes'], len(expected))

    def test_both_byte_literal_quotes_are_supported(self):
        for literal in [r'b"\x01\xff"', r"b'\x01\xff'"]:
            self.assertEqual(robot.identify_and_calculate_length(literal)['bytes_representation'], b'\x01\xff')

    def test_literal_parsing_never_uses_eval(self):
        with mock.patch('builtins.eval', side_effect=AssertionError('eval must not be called')) as evaluate:
            parsed = robot.identify_and_calculate_length(r'b"\x01"')
        evaluate.assert_not_called()
        self.assertEqual(parsed['bytes_representation'], b'\x01')

    def test_nonliteral_expression_keeps_plain_text_semantics(self):
        text = 'b"ab" + b"cd"'
        parsed = robot.identify_and_calculate_length(text)
        self.assertEqual(parsed['type'], 'regular string')
        self.assertEqual(parsed['bytes_representation'], text.encode())

    def test_malformed_literal_remains_plain_text(self):
        text = r'b"\xZZ"'
        self.assertEqual(robot.identify_and_calculate_length(text)['bytes_representation'], text.encode())

    def test_unsupported_input_type_is_rejected(self):
        with self.assertRaises(TypeError):
            robot.identify_and_calculate_length(123)


if __name__ == '__main__':
    unittest.main()
