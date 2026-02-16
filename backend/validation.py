"""
Input validation and sanitization module for the RAG service.
"""

import re
from typing import Any, Dict, List, Optional, Union
from pydantic import BaseModel, Field, validator
from pydantic.networks import HttpUrl
import bleach
import html
from enum import Enum


class QueryType(str, Enum):
    """Enumeration of query types for validation."""
    GENERAL = "general"
    SELECTED_TEXT = "selected_text"
    CHAPTER_SPECIFIC = "chapter_specific"


class ValidatedChatRequest(BaseModel):
    """Validated chat request with proper sanitization."""

    query: str = Field(..., min_length=1, max_length=1000)
    selected_text: Optional[str] = Field(None, max_length=2000)
    context: Optional[Dict[str, Any]] = Field(default_factory=dict)
    conversation_id: Optional[str] = Field(None, max_length=100)

    @validator('query', pre=True)
    def validate_and_sanitize_query(cls, v):
        """Validate and sanitize the query input."""
        if not isinstance(v, str):
            raise ValueError('Query must be a string')

        # Sanitize HTML tags
        sanitized = bleach.clean(v, strip=True)

        # Remove potentially harmful patterns
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'on\w+\s*=', '', sanitized, flags=re.IGNORECASE)

        # Strip leading/trailing whitespace
        sanitized = sanitized.strip()

        if len(sanitized) < 1:
            raise ValueError('Query cannot be empty after sanitization')

        return sanitized

    @validator('selected_text', pre=True)
    def validate_and_sanitize_selected_text(cls, v):
        """Validate and sanitize the selected text input."""
        if v is None:
            return v

        if not isinstance(v, str):
            raise ValueError('Selected text must be a string')

        # Sanitize HTML tags
        sanitized = bleach.clean(v, strip=True)

        # Remove potentially harmful patterns
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'on\w+\s*=', '', sanitized, flags=re.IGNORECASE)

        # Strip leading/trailing whitespace
        sanitized = sanitized.strip()

        return sanitized

    @validator('context', pre=True)
    def validate_context(cls, v):
        """Validate the context dictionary."""
        if v is None:
            return {}

        if not isinstance(v, dict):
            raise ValueError('Context must be a dictionary')

        # Validate specific context keys
        allowed_keys = {'chapter', 'section', 'knowledge_level', 'conversation_id', 'filters'}
        invalid_keys = set(v.keys()) - allowed_keys

        if invalid_keys:
            raise ValueError(f'Invalid context keys: {invalid_keys}')

        return v

    @validator('conversation_id', pre=True)
    def validate_conversation_id(cls, v):
        """Validate the conversation ID."""
        if v is None:
            return v

        if not isinstance(v, str):
            raise ValueError('Conversation ID must be a string')

        # Only allow alphanumeric characters, hyphens, and underscores
        if not re.match(r'^[a-zA-Z0-9_-]+$', v):
            raise ValueError('Conversation ID can only contain alphanumeric characters, hyphens, and underscores')

        return v


class ContentChunkValidator(BaseModel):
    """Validator for content chunks."""

    chunk_id: str = Field(..., regex=r'^[a-zA-Z0-9_-]+$')
    content: str = Field(..., min_length=1, max_length=5000)
    source_document: str = Field(..., min_length=1, max_length=200)
    source_section: Optional[str] = Field(None, max_length=200)
    embedding: Optional[List[float]] = Field(None)
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)

    @validator('content', pre=True)
    def validate_and_sanitize_content(cls, v):
        """Validate and sanitize content."""
        if not isinstance(v, str):
            raise ValueError('Content must be a string')

        # Sanitize HTML tags but allow some safe formatting
        sanitized = bleach.clean(v, strip=True, tags=['p', 'br', 'strong', 'em', 'code', 'ul', 'ol', 'li'])

        # Remove potentially harmful patterns
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'on\w+\s*=', '', sanitized, flags=re.IGNORECASE)

        # Strip leading/trailing whitespace
        sanitized = sanitized.strip()

        if len(sanitized) < 1:
            raise ValueError('Content cannot be empty after sanitization')

        return sanitized

    @validator('embedding', pre=True)
    def validate_embedding(cls, v):
        """Validate embedding vector."""
        if v is None:
            return v

        if not isinstance(v, list):
            raise ValueError('Embedding must be a list of floats')

        if len(v) == 0:
            raise ValueError('Embedding cannot be empty')

        # Check that all values are floats
        for i, val in enumerate(v):
            if not isinstance(val, (int, float)):
                raise ValueError(f'Embedding[{i}] must be a number, got {type(val)}')

        return v


class InputSanitizer:
    """Class to handle input sanitization for various types of inputs."""

    @staticmethod
    def sanitize_text(text: str, allow_html: bool = False) -> str:
        """
        Sanitize text input by removing potentially harmful content.

        Args:
            text: Input text to sanitize
            allow_html: Whether to allow safe HTML tags

        Returns:
            Sanitized text
        """
        if not text:
            return text

        if allow_html:
            # Allow some safe HTML tags
            sanitized = bleach.clean(
                text,
                tags=['p', 'br', 'strong', 'em', 'code', 'ul', 'ol', 'li', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6'],
                attributes={'a': ['href', 'title'], 'img': ['src', 'alt']},
                strip=True
            )
        else:
            # Strip all HTML tags
            sanitized = bleach.clean(text, strip=True)

        # Remove potentially dangerous patterns
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
        sanitized = re.sub(r'<iframe[^>]*>.*?</iframe>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'on\w+\s*=\s*["\'][^"\']*["\']', '', sanitized, flags=re.IGNORECASE)

        # Unescape HTML entities that might be used maliciously
        sanitized = html.unescape(sanitized)

        return sanitized.strip()

    @staticmethod
    def validate_query_length(query: str, max_length: int = 1000) -> bool:
        """
        Validate query length.

        Args:
            query: Query to validate
            max_length: Maximum allowed length

        Returns:
            True if valid, False otherwise
        """
        return isinstance(query, str) and 1 <= len(query) <= max_length

    @staticmethod
    def validate_chapter_reference(chapter: str) -> bool:
        """
        Validate chapter reference format.

        Args:
            chapter: Chapter reference to validate

        Returns:
            True if valid, False otherwise
        """
        if not isinstance(chapter, str):
            return False

        # Allow alphanumeric, spaces, hyphens, colons, periods
        pattern = r'^[a-zA-Z0-9\s\-_:.,]+$'
        return bool(re.match(pattern, chapter.strip()))

    @staticmethod
    def detect_potential_injection(text: str) -> List[str]:
        """
        Detect potential injection patterns in text.

        Args:
            text: Text to scan for injection patterns

        Returns:
            List of detected injection patterns
        """
        injection_patterns = [
            (r'<script[^>]*>', 'potential_script_tag'),
            (r'javascript:', 'potential_javascript'),
            (r'vbscript:', 'potential_vbscript'),
            (r'on\w+\s*=', 'potential_event_handler'),
            (r'<iframe[^>]*>', 'potential_iframe'),
            (r'<object[^>]*>', 'potential_object'),
            (r'<embed[^>]*>', 'potential_embed'),
            (r'eval\s*\(', 'potential_eval'),
            (r'exec\s*\(', 'potential_exec'),
        ]

        detected = []
        for pattern, name in injection_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                detected.append(name)

        return detected


def validate_chat_request(raw_request: Dict[str, Any]) -> ValidatedChatRequest:
    """
    Validate and sanitize a chat request.

    Args:
        raw_request: Raw request dictionary

    Returns:
        Validated and sanitized chat request

    Raises:
        ValueError: If validation fails
    """
    try:
        validated_request = ValidatedChatRequest(**raw_request)
        return validated_request
    except Exception as e:
        raise ValueError(f"Request validation failed: {str(e)}")


def validate_content_chunk(raw_chunk: Dict[str, Any]) -> ContentChunkValidator:
    """
    Validate and sanitize a content chunk.

    Args:
        raw_chunk: Raw chunk dictionary

    Returns:
        Validated and sanitized content chunk

    Raises:
        ValueError: If validation fails
    """
    try:
        validated_chunk = ContentChunkValidator(**raw_chunk)
        return validated_chunk
    except Exception as e:
        raise ValueError(f"Content chunk validation failed: {str(e)}")


def sanitize_user_query(query: str) -> str:
    """
    Convenience function to sanitize a user query.

    Args:
        query: User query to sanitize

    Returns:
        Sanitized query
    """
    return InputSanitizer.sanitize_text(query, allow_html=False)


def sanitize_selected_text(selected_text: str) -> str:
    """
    Convenience function to sanitize selected text.

    Args:
        selected_text: Selected text to sanitize

    Returns:
        Sanitized selected text
    """
    return InputSanitizer.sanitize_text(selected_text, allow_html=True)


# Global sanitizer instance
input_sanitizer = InputSanitizer()


def get_input_sanitizer() -> InputSanitizer:
    """
    Get the global input sanitizer instance.

    Returns:
        InputSanitizer instance
    """
    return input_sanitizer


# Middleware-style validation function
def validate_and_sanitize_request(request_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate and sanitize request data in middleware style.

    Args:
        request_data: Raw request data

    Returns:
        Validated and sanitized request data
    """
    # Validate and sanitize query
    if 'query' in request_data:
        request_data['query'] = sanitize_user_query(request_data['query'])

        # Check for potential injection patterns
        injection_detections = InputSanitizer.detect_potential_injection(request_data['query'])
        if injection_detections:
            raise ValueError(f"Potential injection detected in query: {', '.join(injection_detections)}")

    # Validate and sanitize selected text
    if 'selected_text' in request_data and request_data['selected_text'] is not None:
        request_data['selected_text'] = sanitize_selected_text(request_data['selected_text'])

        # Check for potential injection patterns
        injection_detections = InputSanitizer.detect_potential_injection(request_data['selected_text'])
        if injection_detections:
            raise ValueError(f"Potential injection detected in selected text: {', '.join(injection_detections)}")

    # Validate context
    if 'context' in request_data and request_data['context']:
        context = request_data['context']
        if not isinstance(context, dict):
            raise ValueError("Context must be a dictionary")

        # Validate specific context keys
        allowed_keys = {'chapter', 'section', 'knowledge_level', 'conversation_id', 'filters'}
        invalid_keys = set(context.keys()) - allowed_keys
        if invalid_keys:
            raise ValueError(f"Invalid context keys: {invalid_keys}")

    return request_data


# Initialize validation module
print("Input validation and sanitization module initialized")