import unittest

class TestSanityCheck(unittest.TestCase):

    def test_math_works(self):
        """A simple test to make sure pytest is running."""
        print("\n[Sanity Check] Checking if 1 + 1 equals 2...")
        self.assertEqual(1 + 1, 2)

    # Uncomment the lines below to see what a FAILURE looks like
    # def test_intentional_fail(self):
    #     print("\n[Sanity Check] This test is supposed to fail.")
    #     self.assertEqual(1 + 1, 5)