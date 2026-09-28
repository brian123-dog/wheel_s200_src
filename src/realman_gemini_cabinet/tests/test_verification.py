import unittest

from rg_cabinet.verification import MockStateVerifier, VerificationRequest


class VerificationTests(unittest.TestCase):
    def test_mock_state_verifier_compares_expected_state(self):
        verifier = MockStateVerifier({"breaker": "on"})
        result = verifier.verify(VerificationRequest("breaker", "on"))

        self.assertTrue(result.passed)
        self.assertEqual(result.observed, "on")

    def test_mock_state_verifier_reports_mismatch(self):
        verifier = MockStateVerifier({"breaker": "off"})
        result = verifier.verify(VerificationRequest("breaker", "on"))

        self.assertFalse(result.passed)
        self.assertEqual(result.observed, "off")


if __name__ == "__main__":
    unittest.main()
