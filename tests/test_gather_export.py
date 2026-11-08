"""Exercise both CSV exports with a fake save function, without loading an SDK."""
import csv
import ctypes
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from test_robot_utilities import robot


class GatherExportTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.instances = []
        for cls, name in [(robot.Marvin_Robot, 'save_collected_data_as_csv_to_path'),
                          (robot.Concise_Marvin_Robot, 'save_gather_data_as_csv_to_path')]:
            instance = cls.__new__(cls)
            instance.robot = mock.Mock()
            self.instances.append((instance, getattr(instance, name)))
        # Isolate the old implementation's current-directory tmp.txt too.
        previous = os.getcwd()
        os.chdir(self.root)
        self.addCleanup(os.chdir, previous)
        self.saved_paths = []
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(robot.time, 'sleep').start()

    def save(self, path):
        path = Path(ctypes.cast(path, ctypes.c_char_p).value.decode('utf-8'))
        self.saved_paths.append(path)
        path.write_text('PoinType=2@2\nX 0$Y 10$Z 1.250000$A -2.500000$\n'
                        'X 1$Y 20$Z 3.000000$A 4.000000$\n', encoding='utf-8')
        return True

    def test_success_preserves_unrelated_tmp_and_removes_private_file(self):
        unrelated = self.root / 'tmp.txt'
        for instance, export in self.instances:
            with self.subTest(wrapper=type(instance).__name__):
                unrelated.write_bytes(b'user data')
                instance.robot.OnSaveGatherData.side_effect = self.save
                output = self.root / 'data.csv'
                self.assertTrue(export(str(output)))
                with output.open(newline='', encoding='utf-8') as stream:
                    self.assertEqual(list(csv.reader(stream)),
                                     [['1.250000', '-2.500000'], ['3.000000', '4.000000']])
                self.assertTrue(unrelated.exists(), 'Unrelated tmp.txt was removed')
                self.assertEqual(unrelated.read_bytes(), b'user data')
                self.assertFalse(self.saved_paths[-1].exists())
                self.assertFalse(self.saved_paths[-1].parent.exists())
        self.assertEqual(len(set(self.saved_paths)), 2)

    def test_sdk_failure_preserves_destination_and_unrelated_tmp(self):
        for instance, export in self.instances:
            with self.subTest(wrapper=type(instance).__name__):
                unrelated = self.root / 'tmp.txt'
                unrelated.write_text('header\nX 0$Y 0$Z 999$\n', encoding='utf-8')
                before = unrelated.read_bytes()
                output = self.root / 'existing.csv'
                output.write_bytes(b'original')
                instance.robot.OnSaveGatherData.return_value = False
                self.assertFalse(export(str(output)))
                self.assertEqual(output.read_bytes(), b'original')
                self.assertEqual(unrelated.read_bytes(), before)

    def test_save_exceptions_return_false(self):
        for instance, export in self.instances:
            with self.subTest(wrapper=type(instance).__name__):
                instance.robot.OnSaveGatherData.side_effect = OSError('save unavailable')
                self.assertFalse(export(str(self.root / 'data.csv')))

    def test_missing_raw_file_returns_false(self):
        for instance, export in self.instances:
            with self.subTest(wrapper=type(instance).__name__):
                instance.robot.OnSaveGatherData.return_value = True
                self.assertFalse(export(str(self.root / 'data.csv')))

    def test_output_error_cleans_up_raw_file(self):
        for instance, export in self.instances:
            with self.subTest(wrapper=type(instance).__name__):
                instance.robot.OnSaveGatherData.side_effect = self.save
                self.assertFalse(export(str(self.root / 'missing' / 'data.csv')))
                self.assertFalse(self.saved_paths[-1].exists())


if __name__ == '__main__':
    unittest.main()
