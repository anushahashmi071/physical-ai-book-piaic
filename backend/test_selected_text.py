"""
Basic tests for selected text query functionality.
This is a preliminary test file to validate the core functionality.
"""

import asyncio
from services.agent import get_agent_service
from services.vector_storage import get_vector_storage_service
from models.content import ContentChunk, ContentSource, ContentMetadata
from utils.embedding import get_embedding_generator


async def test_selected_text_functionality():
    """
    Test the selected text query functionality with various content types.
    """
    print("Testing selected text query functionality...")

    # Get services
    agent_service = get_agent_service()
    vector_storage = get_vector_storage_service()
    embedding_gen = get_embedding_generator()

    # Initialize vector storage
    await vector_storage.initialize_collection()

    # Create sample content chunks for testing
    sample_chunks = [
        ContentChunk(
            chunk_id="sample-chunk-1",
            content="In robotics, perception refers to the ability of a robot to sense and interpret its environment. This includes processing data from various sensors like cameras, LiDAR, and IMUs.",
            source=ContentSource(
                document_id="robotics-textbook",
                chapter="Chapter 3: Perception Systems",
                section="Introduction to Perception"
            ),
            metadata=ContentMetadata(
                tags=["perception", "sensors", "environment"],
                concepts=["robotics", "sensing"],
                difficulty_level="intermediate"
            )
        ),
        ContentChunk(
            chunk_id="sample-chunk-2",
            content="Sensor fusion combines data from multiple sensors to achieve better accuracy and reliability than individual sensors alone. Common fusion levels include signal, feature, decision, and symbol level fusion.",
            source=ContentSource(
                document_id="robotics-textbook",
                chapter="Chapter 3: Perception Systems",
                section="Sensor Fusion Techniques"
            ),
            metadata=ContentMetadata(
                tags=["sensor-fusion", "accuracy", "reliability"],
                concepts=["multi-sensor", "integration"],
                difficulty_level="intermediate"
            )
        )
    ]

    # Store sample chunks in vector database
    stored_count = await vector_storage.store_chunks(sample_chunks)
    print(f"Stored {stored_count} sample chunks in vector database")

    # Test 1: General query without selected text
    print("\nTest 1: General query without selected text")
    result1 = await agent_service.process_query(
        query="What is perception in robotics?",
        selected_text=None
    )
    print(f"Answer: {result1['answer'][:100]}...")
    print(f"Confidence: {result1['confidence']:.2f}")
    print(f"Retrieved chunks: {result1['retrieved_chunks_count']}")

    # Test 2: Query with selected text (context boosting)
    print("\nTest 2: Query with selected text (context boosting)")
    selected_text = "Sensor fusion combines data from multiple sensors to achieve better accuracy and reliability than individual sensors alone."
    result2 = await agent_service.process_query(
        query="Can you elaborate on the fusion levels mentioned?",
        selected_text=selected_text
    )
    print(f"Answer: {result2['answer'][:100]}...")
    print(f"Confidence: {result2['confidence']:.2f}")
    print(f"Retrieved chunks: {result2['retrieved_chunks_count']}")
    print(f"Grounding validation: {result2['grounding_validation']['is_properly_grounded']}")

    # Test 3: Different content type - more technical
    print("\nTest 3: Technical query with selected text")
    selected_text2 = "Common fusion levels include signal, feature, decision, and symbol level fusion."
    result3 = await agent_service.process_query(
        query="Explain these fusion levels in detail",
        selected_text=selected_text2
    )
    print(f"Answer: {result3['answer'][:100]}...")
    print(f"Confidence: {result3['confidence']:.2f}")
    print(f"Grounding validation: {result3['grounding_validation']['is_properly_grounded']}")

    # Cleanup
    await vector_storage.clear_collection()
    print("\nTest completed and collection cleared.")


if __name__ == "__main__":
    asyncio.run(test_selected_text_functionality())