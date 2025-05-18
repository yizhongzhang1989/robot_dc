import unittest
from unittest.mock import patch, MagicMock
from modbus_driver.modbus_manager_node import ModbusManagerNode

class DummyResponse:
    def __init__(self):
        self.result = []
        self.success = False
        self.error_message = ''

class TestModbusManager(unittest.TestCase):

    @patch('modbus_driver.modbus_rtu_interface.ModbusRTUInterface')
    def test_read_register_success(self, MockInterface):
        mock_iface = MockInterface.return_value
        mock_iface.connect.return_value = True
        mock_iface.read_register.return_value.isError.return_value = False
        mock_iface.read_register.return_value.registers = [100, 200]

        node = ModbusManagerNode()
        request = type('Request', (object,), {
            'function_code': 3,
            'slave_id': 1,
            'address': 0,
            'values': [0, 0]
        })()
        response = DummyResponse()
        response = node.handle_modbus_request(request, response)

        self.assertTrue(response.success)
        self.assertEqual(response.result, [100, 200])

if __name__ == '__main__':
    unittest.main()
