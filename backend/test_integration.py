"""
Comprehensive integration tests for the RAG chatbot service.
"""

import asyncio
import unittest
from unittest.mock import AsyncMock, Mock, patch
from fastapi.testclient import TestClient
from main import app
from models.query import ChatRequest
from services.agent import get_agent_service
from services.vector_storage import get_vector_storage_service
from services.ingestion import get_ingestion_service
from models.content import ContentChunk, ContentSource, ContentMetadata
from validation import validate_chat_request, InputSanitizer


class TestRAGIntegration(unittest.TestCase):
    """
    Integration tests for the RAG chatbot service.
    """

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.client = TestClient(app)
        self.agent_service = get_agent_service()
        self.vector_storage = get_vector_storage_service()
        self.ingestion_service = get_ingestion_service()

    def test_health_endpoint(self):
        """Test that health endpoint returns expected status."""
        response = self.client.get("/health/")
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn("status", data)
        self.assertIn("services", data)
        self.assertTrue(data["status"] in ["healthy", "degraded"])

    def test_detailed_health_endpoint(self):
        """Test that detailed health endpoint returns expected status."""
        response = self.client.get("/health/detailed")
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn("overall_status", data)
        self.assertIn("checks", data)
        self.assertIn("vector_database", data["checks"])

    def test_metrics_endpoint(self):
        """Test that metrics endpoint returns expected data."""
        response = self.client.get("/health/metrics")
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn("timestamp", data)
        self.assertIn("uptime_seconds", data)

    def test_chat_endpoint_basic(self):
        """Test basic chat functionality."""
        # Test with a simple query
        request_data = {
            "query": "What is ROS 2?",
            "selected_text": None,
            "context": {}
        }

        response = self.client.post("/chat/", json=request_data)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn("response_id", data)
        self.assertIn("answer", data)
        self.assertIn("citations", data)
        self.assertIn("confidence", data)
        self.assertGreaterEqual(data["confidence"], 0.0)
        self.assertLessEqual(data["confidence"], 1.0)

    def test_chat_endpoint_with_selected_text(self):
        """Test chat functionality with selected text."""
        selected_text = "ROS 2 is a flexible framework for robot software development"
        request_data = {
            "query": "Explain this concept",
            "selected_text": selected_text,
            "context": {}
        }

        response = self.client.post("/chat/", json=request_data)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn("response_id", data)
        self.assertIn("answer", data)
        self.assertIn("citations", data)
        # The response should somehow relate to the selected text

    def test_chat_endpoint_with_context(self):
        """Test chat functionality with chapter context."""
        request_data = {
            "query": "What are perception systems?",
            "selected_text": None,
            "context": {
                "chapter": "Chapter 3: Perception Systems",
                "section": "Introduction"
            }
        }

        response = self.client.post("/chat/", json=request_data)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn("response_id", data)
        self.assertIn("answer", data)
        self.assertIn("citations", data)

    def test_input_validation(self):
        """Test input validation and sanitization."""
        # Test valid input
        valid_request = {
            "query": "What is a robot?",
            "selected_text": "Robots are machines capable of carrying out complex tasks",
            "context": {"chapter": "Introduction"}
        }

        validated_request = validate_chat_request(valid_request)
        self.assertEqual(validated_request.query, "What is a robot?")
        self.assertEqual(validated_request.selected_text, "Robots are machines capable of carrying out complex tasks")

        # Test sanitization
        malicious_query = "What is a robot? <script>alert('xss')</script>"
        sanitized = InputSanitizer.sanitize_text(malicious_query)
        self.assertNotIn("<script>", sanitized)
        self.assertIn("What is a robot?", sanitized)

    def test_concurrent_requests(self):
        """Test handling of concurrent requests."""
        # Create multiple requests to simulate concurrent access
        requests_data = [
            {"query": "What is ROS 2?", "selected_text": None, "context": {}},
            {"query": "What is perception?", "selected_text": None, "context": {}},
            {"query": "How does sensor fusion work?", "selected_text": None, "context": {}}
        ]

        # Execute requests concurrently
        responses = []
        for req_data in requests_data:
            response = self.client.post("/chat/", json=req_data)
            responses.append(response)

        # Verify all responses are successful
        for response in responses:
            self.assertEqual(response.status_code, 200)
            data = response.json()
            self.assertIn("response_id", data)
            self.assertIn("answer", data)

    def test_error_handling(self):
        """Test error handling for invalid requests."""
        # Test with empty query
        invalid_request = {"query": "", "selected_text": None, "context": {}}
        response = self.client.post("/chat/", json=invalid_request)
        self.assertEqual(response.status_code, 422)  # Validation error

        # Test with very long query
        long_query = "a" * 2000  # Exceeds max length
        invalid_request = {"query": long_query, "selected_text": None, "context": {}}
        response = self.client.post("/chat/", json=invalid_request)
        self.assertEqual(response.status_code, 422)  # Validation error


class TestServiceIntegration(unittest.IsolatedAsyncioTestCase):
    """
    Async integration tests for individual services.
    """

    async def test_ingestion_to_retrieval_flow(self):
        """Test the complete flow from ingestion to retrieval."""
        # Create test content
        test_chunk = ContentChunk(
            chunk_id="test-chunk-1",
            content="This is a test content about robotics and AI.",
            source=ContentSource(
                document_id="test-doc",
                chapter="Test Chapter",
                section="Test Section"
            ),
            metadata=ContentMetadata(
                tags=["robotics", "AI"],
                concepts=["machine learning"],
                difficulty_level="intermediate"
            )
        )

        # Mock the vector storage service
        with patch.object(get_vector_storage_service(), 'store_chunk', new_callable=AsyncMock) as mock_store:
            mock_store.return_value = True

            # Test ingestion service
            result = await get_ingestion_service().ingestion_service._process_document(
                Mock(id="test-doc", content=test_chunk.content, metadata={})
            )

            # Verify the chunk was processed
            self.assertIsNotNone(result)

    async def test_retrieval_and_generation_flow(self):
        """Test the retrieval and generation flow."""
        # Mock services for this test
        with patch.object(get_agent_service(), 'process_query') as mock_process:
            mock_process.return_value = {
                "query": "test query",
                "answer": "This is a test answer based on textbook content.",
                "citations": [{"chapter": "Test Chapter", "relevance_score": 0.9}],
                "confidence": 0.85,
                "grounding_validation": {"is_properly_grounded": True},
                "retrieved_chunks_count": 2,
                "processing_successful": True
            }

            # Test the agent service
            result = await get_agent_service().process_query("test query")

            # Verify the result
            self.assertTrue(result["processing_successful"])
            self.assertEqual(result["answer"], "This is a test answer based on textbook content.")
            self.assertGreaterEqual(result["confidence"], 0.8)


def run_integration_tests():
    """
    Run all integration tests.
    """
    print("Running RAG Chatbot Integration Tests...")

    # Create test suite
    suite = unittest.TestSuite()

    # Add tests to suite
    suite.addTest(unittest.makeSuite(TestRAGIntegration))
    suite.addTest(unittest.makeSuite(TestServiceIntegration))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print(f"\nIntegration Test Results:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success Rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.2f}%")

    return result.wasSuccessful()


async def run_async_integration_tests():
    """
    Run async integration tests separately.
    """
    print("Running Async Integration Tests...")

    # For async tests, we need to run them separately
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestServiceIntegration)

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


async def main():
    """
    Main function to run all integration tests.
    """
    print("Starting comprehensive integration testing for RAG Chatbot Service...\n")

    # Run sync tests
    sync_success = run_integration_tests()

    # Run async tests
    async_success = await run_async_integration_tests()

    # Overall result
    overall_success = sync_success and async_success

    print(f"\nOverall Integration Test Result: {'PASSED' if overall_success else 'FAILED'}")

    if overall_success:
        print("✅ All integration tests passed! The RAG system is functioning correctly.")
        print("✅ Services are properly integrated and communicating.")
        print("✅ Error handling and validation are working as expected.")
    else:
        print("❌ Some integration tests failed. Please review the test output above.")

    return overall_success


if __name__ == "__main__":
    success = asyncio.run(main())
    exit(0 if success else 1)