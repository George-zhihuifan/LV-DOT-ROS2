import unittest

from qcgaf_fusion.fusion_node import image_msg_to_numpy


class DummyImage:
    def __init__(self, h, w, enc, data):
        self.height = h
        self.width = w
        self.encoding = enc
        self.data = data


class TestImageMsgToNumpy(unittest.TestCase):
    def test_rgb8_valid(self):
        msg = DummyImage(2, 3, 'rgb8', bytes(2 * 3 * 3))
        arr = image_msg_to_numpy(msg)
        self.assertIsNotNone(arr)
        self.assertEqual(arr.shape, (2, 3, 3))

    def test_rgb8_size_mismatch(self):
        msg = DummyImage(2, 3, 'rgb8', bytes(2 * 3 * 2))
        arr = image_msg_to_numpy(msg)
        self.assertIsNone(arr)


if __name__ == '__main__':
    unittest.main()
