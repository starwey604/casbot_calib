import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


FIRMWARE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "check_usb_cdc", FIRMWARE / "tools/check_usb_cdc.py"
)
cdc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cdc)


class CdcExample(unittest.TestCase):
    def test_discovery_selects_example_not_bootloader_or_bridge(self):
        ports = [
            SimpleNamespace(device="bootloader", vid=0x239A, product="nice!nano"),
            SimpleNamespace(device="bridge", vid=0x2FE3, product="CASBOT Gauge Bridge"),
            SimpleNamespace(device="example", vid=0x2FE3, product="CASBOT USB CDC Example"),
        ]
        with patch.object(cdc.list_ports, "comports", return_value=ports):
            self.assertEqual(cdc.example_port(), "example")

    def test_discovery_requires_exactly_one_example(self):
        port = SimpleNamespace(device="example", vid=0x2FE3, product="CASBOT USB CDC Example")
        for ports in ([], [port, port]):
            with self.subTest(ports=ports):
                with patch.object(cdc.list_ports, "comports", return_value=ports):
                    with self.assertRaises(RuntimeError):
                        cdc.example_port()

    def test_text_binary_and_reopen(self):
        with patch.object(cdc.serial, "Serial") as factory:
            device = factory.return_value.__enter__.return_value
            device.write.side_effect = lambda payload: len(payload)
            device.read.side_effect = [b"CASBOT USB CDC echo\r\n", bytes(range(256))] * 2
            with patch.object(cdc.time, "sleep"), patch("sys.stdout", new_callable=io.StringIO):
                cdc.check_echo("example")
            self.assertEqual(factory.call_count, 2)
            self.assertEqual(device.write.call_count, 4)
            self.assertEqual(factory.return_value.__exit__.call_count, 2)
            self.assertTrue(device.dtr)

    def test_bad_or_missing_echo_fails_and_closes(self):
        for response in (b"", b"wrong bytes"):
            with self.subTest(response=response):
                with patch.object(cdc.serial, "Serial") as factory:
                    device = factory.return_value.__enter__.return_value
                    device.write.side_effect = lambda payload: len(payload)
                    device.read.return_value = response
                    with patch.object(cdc.time, "sleep"):
                        with self.assertRaisesRegex(RuntimeError, "回显不匹配"):
                            cdc.check_echo("example")
                    factory.return_value.__exit__.assert_called_once()


if __name__ == "__main__":
    unittest.main()
