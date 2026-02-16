"""
Simplified embedding utilities for the RAG service.
This version uses a lightweight approach to avoid heavy dependencies like PyTorch.
"""

import numpy as np
from typing import List, Union
import logging
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import pickle
import hashlib


logger = logging.getLogger(__name__)


class EmbeddingGenerator:
    """
    Simplified embedding generator using TF-IDF instead of sentence transformers.
    This provides a lightweight alternative that avoids heavy dependencies like PyTorch.
    """

    def __init__(self, model_name: str = "tfidf"):
        """
        Initialize the embedding generator with TF-IDF approach.

        Args:
            model_name: Name of the approach to use (currently only tfidf is supported)
        """
        self.model_name = model_name
        self.vectorizer = TfidfVectorizer(
            max_features=1000,  # Reduced for performance
            stop_words='english',
            lowercase=True,
            ngram_range=(1, 2)  # Include unigrams and bigrams
        )
        self.documents = []  # Store documents for similarity calculations
        self.document_vectors = None

        logger.info(f"Initialized embedding generator with {model_name} approach")

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for the given text using a hash-based approach.
        For similarity calculations, this stores the document and calculates similarity separately.

        Args:
            text: Input text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        if not text.strip():
            raise ValueError("Input text cannot be empty")

        # For this simplified version, we'll use a hash-based approach for individual embeddings
        # but store the full text for similarity calculations
        self.documents.append(text)

        # Create a simple hash-based embedding (for API compatibility)
        text_hash = hashlib.md5(text.encode()).hexdigest()
        # Convert hex to float vector
        embedding = []
        for i in range(0, len(text_hash), 2):
            hex_pair = text_hash[i:i+2]
            val = int(hex_pair, 16) / 255.0  # Normalize to 0-1
            embedding.append(val)

        # Pad or trim to a fixed size (e.g., 128 dimensions)
        while len(embedding) < 128:
            embedding.append(0.0)
        embedding = embedding[:128]

        return embedding

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of input texts to generate embeddings for

        Returns:
            List of embedding vectors (each as a list of floats)
        """
        if not texts:
            return []

        # Filter out empty texts
        filtered_texts = [text for text in texts if text.strip()]

        if not filtered_texts:
            return []

        embeddings = []
        for text in filtered_texts:
            embeddings.append(self.generate_embedding(text))

        return embeddings

    def calculate_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings.

        Args:
            embedding1: First embedding vector
            embedding2: Second embedding vector

        Returns:
            Cosine similarity score between 0 and 1
        """
        emb1 = np.array(embedding1)
        emb2 = np.array(embedding2)

        # Normalize the vectors
        norm1 = np.linalg.norm(emb1)
        norm2 = np.linalg.norm(emb2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        # Calculate cosine similarity
        dot_product = np.dot(emb1, emb2)
        cosine_sim = dot_product / (norm1 * norm2)

        # Ensure the result is between 0 and 1
        return float(max(0, min(1, (cosine_sim + 1) / 2)))

    def calculate_similarity_tfidf(self, text1: str, text2: str) -> float:
        """
        Calculate similarity between two texts using TF-IDF approach.

        Args:
            text1: First text for comparison
            text2: Second text for comparison

        Returns:
            Similarity score between 0 and 1
        """
        # Fit the vectorizer on both texts
        self.vectorizer.fit([text1, text2])

        # Transform the texts
        tfidf_matrix = self.vectorizer.transform([text1, text2])

        # Calculate cosine similarity
        similarity_matrix = cosine_similarity(tfidf_matrix[0:1], tfidf_matrix[1:2])

        return float(similarity_matrix[0][0])


# Global instance for easy access
embedding_generator = None


def get_embedding_generator(model_name: str = "tfidf") -> EmbeddingGenerator:
    """
    Get or create a singleton instance of the embedding generator.

    Args:
        model_name: Name of the approach to use

    Returns:
        EmbeddingGenerator instance
    """
    global embedding_generator
    if embedding_generator is None:
        embedding_generator = EmbeddingGenerator(model_name)
    return embedding_generator