"""
Script to verify content indexing and retrieval quality.
"""

import asyncio
from services.vector_storage import get_vector_storage_service
from services.retrieval import get_retrieval_service
from services.generation import get_generation_service
from config.settings import settings


async def verify_content_indexing():
    """
    Verify that content has been properly indexed in the vector database.
    """
    print("Verifying content indexing...")

    # Get vector storage service
    vector_storage = get_vector_storage_service()

    # Get collection information
    collection_info = await vector_storage.get_collection_info()

    if collection_info:
        print(f"✓ Collection exists with {collection_info['points_count']} vectors")
        print(f"✓ Indexed vectors count: {collection_info['indexed_vectors_count']}")

        # Check if we have a reasonable number of vectors
        if collection_info['points_count'] > 100:  # Expecting at least 100 from our textbook content
            print("✓ Adequate number of vectors indexed")
        else:
            print("⚠ Fewer vectors than expected - check ingestion process")
    else:
        print("✗ Could not retrieve collection information")
        return False

    return True


async def verify_retrieval_quality():
    """
    Verify retrieval quality with sample queries.
    """
    print("\nVerifying retrieval quality...")

    # Get services
    retrieval_service = get_retrieval_service()
    generation_service = get_generation_service()

    # Sample queries to test retrieval
    test_queries = [
        "What is ROS 2?",
        "Explain perception in robotics",
        "How does sensor fusion work?",
        "What are Vision-Language-Action models?",
        "Describe digital twin concepts"
    ]

    all_tests_passed = True

    for i, query in enumerate(test_queries, 1):
        print(f"\nTest {i}: Query - '{query}'")

        try:
            # Retrieve relevant content
            retrieved_chunks = await retrieval_service.retrieve_relevant_content(
                query=query,
                top_k=3
            )

            print(f"  Retrieved {len(retrieved_chunks)} relevant chunks")

            if len(retrieved_chunks) > 0:
                # Check quality of first result
                first_chunk = retrieved_chunks[0]
                print(f"  Best match similarity: {first_chunk.similarity_score:.3f}")

                if first_chunk.similarity_score > 0.3:  # Reasonable threshold
                    print(f"  Content preview: {first_chunk.content[:100]}...")
                    print("  ✓ Good retrieval quality")
                else:
                    print("  ⚠ Low similarity score - quality may be poor")
                    all_tests_passed = False
            else:
                print("  ✗ No content retrieved")
                all_tests_passed = False

        except Exception as e:
            print(f"  ✗ Error during retrieval: {str(e)}")
            all_tests_passed = False

    return all_tests_passed


async def verify_generation_quality():
    """
    Verify generation quality with sample queries.
    """
    print("\nVerifying generation quality...")

    # Get services
    retrieval_service = get_retrieval_service()
    generation_service = get_generation_service()

    # Test query
    test_query = "What is the difference between sensing and perception in robotics?"

    print(f"Test query: '{test_query}'")

    try:
        # Retrieve relevant content
        retrieved_chunks = await retrieval_service.retrieve_relevant_content(
            query=test_query,
            top_k=3
        )

        if len(retrieved_chunks) > 0:
            # Generate response
            response_data = await generation_service.generate_response(
                query=test_query,
                retrieved_context=retrieved_chunks
            )

            answer = response_data["answer"]
            citations = response_data["citations"]
            confidence = response_data["confidence"]

            print(f"Generated answer (first 150 chars): {answer[:150]}...")
            print(f"Citations provided: {len(citations)}")
            print(f"Confidence score: {confidence:.3f}")

            # Basic quality checks
            if len(answer) > 50:  # Reasonable length
                print("✓ Reasonable response length")
            else:
                print("⚠ Short response - may indicate quality issue")

            if len(citations) > 0:
                print("✓ Citations provided")
            else:
                print("⚠ No citations provided")

            if confidence > 0.5:
                print("✓ Adequate confidence")
            else:
                print("⚠ Low confidence")

            print("✓ Generation quality test completed")
            return True
        else:
            print("✗ No content retrieved for generation test")
            return False

    except Exception as e:
        print(f"✗ Error during generation quality test: {str(e)}")
        return False


async def run_verification():
    """
    Run all verification tests.
    """
    print("Starting content indexing and retrieval verification...\n")

    # Verify content indexing
    indexing_ok = await verify_content_indexing()

    # Verify retrieval quality
    retrieval_ok = await verify_retrieval_quality()

    # Verify generation quality
    generation_ok = await verify_generation_quality()

    print(f"\nVerification Summary:")
    print(f"- Content indexing: {'✓ PASS' if indexing_ok else '✗ FAIL'}")
    print(f"- Retrieval quality: {'✓ PASS' if retrieval_ok else '✗ FAIL'}")
    print(f"- Generation quality: {'✓ PASS' if generation_ok else '✗ FAIL'}")

    overall_success = indexing_ok and retrieval_ok and generation_ok
    print(f"- Overall: {'✓ PASS' if overall_success else '✗ FAIL'}")

    if overall_success:
        print("\nAll verification tests passed! The RAG system is functioning properly.")
    else:
        print("\nSome verification tests failed. Please review the issues above.")

    return overall_success


async def main():
    """
    Main function to run verification.
    """
    try:
        await run_verification()
    except Exception as e:
        print(f"Error running verification: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())