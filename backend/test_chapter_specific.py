"""
Basic tests for chapter-specific question handling.
This is a preliminary test file to validate the core functionality.
"""

import asyncio
from services.agent import get_agent_service
from services.vector_storage import get_vector_storage_service
from models.content import ContentChunk, ContentSource, ContentMetadata
from utils.embedding import get_embedding_generator


async def test_chapter_specific_functionality():
    """
    Test the chapter-specific question handling with various modules.
    """
    print("Testing chapter-specific question handling...")

    # Get services
    agent_service = get_agent_service()
    vector_storage = get_vector_storage_service()
    embedding_gen = get_embedding_generator()

    # Initialize vector storage
    await vector_storage.initialize_collection()

    # Create sample content chunks for different chapters
    sample_chunks = [
        ContentChunk(
            chunk_id="chapter1-chunk-1",
            content="ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior.",
            source=ContentSource(
                document_id="robotics-textbook",
                chapter="Chapter 1: Introduction to ROS 2",
                section="Overview"
            ),
            metadata=ContentMetadata(
                tags=["ros2", "framework", "robotics"],
                concepts=["robot-software", "libraries"],
                difficulty_level="beginner"
            )
        ),
        ContentChunk(
            chunk_id="chapter3-chunk-1",
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
            chunk_id="chapter5-chunk-1",
            content="Vision-Language-Action (VLA) models integrate visual perception, natural language understanding, and action execution in robotics. These models enable robots to follow complex instructions expressed in natural language.",
            source=ContentSource(
                document_id="robotics-textbook",
                chapter="Chapter 5: Vision-Language-Action Integration",
                section="Introduction to VLA"
            ),
            metadata=ContentMetadata(
                tags=["vla", "vision", "language", "action"],
                concepts=["multimodal", "integration"],
                difficulty_level="advanced"
            )
        )
    ]

    # Store sample chunks in vector database
    stored_count = await vector_storage.store_chunks(sample_chunks)
    print(f"Stored {stored_count} sample chunks in vector database")

    # Test 1: General query (should pull from relevant chapters)
    print("\nTest 1: General query about ROS 2")
    result1 = await agent_service.process_query(
        query="What is ROS 2?",
        context_filters={"knowledge_level": "beginner"}
    )
    print(f"Answer: {result1['answer'][:100]}...")
    print(f"Confidence: {result1['confidence']:.2f}")
    print(f"Retrieved chunks: {result1['retrieved_chunks_count']}")

    # Test 2: Chapter-specific query
    print("\nTest 2: Chapter-specific query about perception")
    result2 = await agent_service.process_query(
        query="How do robots perceive their environment?",
        context_filters={
            "chapter": "Chapter 3: Perception Systems",
            "knowledge_level": "intermediate"
        }
    )
    print(f"Answer: {result2['answer'][:100]}...")
    print(f"Confidence: {result2['confidence']:.2f}")
    print(f"Retrieved chunks: {result2['retrieved_chunks_count']}")

    # Test 3: Advanced query with advanced knowledge level
    print("\nTest 3: Advanced query with advanced knowledge level")
    result3 = await agent_service.process_query(
        query="How do VLA models integrate different modalities?",
        context_filters={
            "chapter": "Chapter 5: Vision-Language-Action Integration",
            "knowledge_level": "advanced"
        }
    )
    print(f"Answer: {result3['answer'][:100]}...")
    print(f"Confidence: {result3['confidence']:.2f}")
    print(f"Retrieved chunks: {result3['retrieved_chunks_count']}")

    # Test 4: Beginner-level query on advanced topic (should simplify response)
    print("\nTest 4: Beginner-level query on advanced topic")
    result4 = await agent_service.process_query(
        query="What are VLA models?",
        context_filters={
            "chapter": "Chapter 5: Vision-Language-Action Integration",
            "knowledge_level": "beginner"
        }
    )
    print(f"Answer: {result4['answer'][:100]}...")
    print(f"Confidence: {result4['confidence']:.2f}")
    print(f"Retrieved chunks: {result4['retrieved_chunks_count']}")

    # Cleanup
    await vector_storage.clear_collection()
    print("\nTest completed and collection cleared.")


if __name__ == "__main__":
    asyncio.run(test_chapter_specific_functionality())