from typing import Dict, List, Optional, Any
import logging
from uuid import uuid4
from datetime import datetime, timedelta
from models.query import QueryRequest, ChatResponse


logger = logging.getLogger(__name__)


class ConversationMemory:
    """
    In-memory storage for conversation contexts.
    In a production system, this would use Redis or a database.
    """

    def __init__(self):
        self.conversations: Dict[str, List[Dict[str, Any]]] = {}
        self.last_accessed: Dict[str, datetime] = {}
        self.max_messages_per_conversation = 10  # Keep only recent messages
        self.conversation_ttl = timedelta(hours=1)  # Expire conversations after 1 hour

    def add_message(self, conversation_id: str, query: str, response: str) -> None:
        """Add a query-response pair to the conversation."""
        if conversation_id not in self.conversations:
            self.conversations[conversation_id] = []

        # Add the new message
        self.conversations[conversation_id].append({
            "query": query,
            "response": response,
            "timestamp": datetime.utcnow()
        })

        # Limit the number of messages stored
        if len(self.conversations[conversation_id]) > self.max_messages_per_conversation:
            self.conversations[conversation_id] = self.conversations[conversation_id][-self.max_messages_per_conversation:]

        # Update last accessed time
        self.last_accessed[conversation_id] = datetime.utcnow()

        # Clean up expired conversations
        self._cleanup_expired()

    def get_conversation_history(self, conversation_id: str) -> List[Dict[str, Any]]:
        """Get the conversation history for a given conversation ID."""
        # Clean up expired conversations first
        self._cleanup_expired()

        return self.conversations.get(conversation_id, [])

    def create_conversation(self) -> str:
        """Create a new conversation and return its ID."""
        conversation_id = str(uuid4())
        self.conversations[conversation_id] = []
        self.last_accessed[conversation_id] = datetime.utcnow()
        return conversation_id

    def clear_conversation(self, conversation_id: str) -> None:
        """Clear a specific conversation."""
        if conversation_id in self.conversations:
            del self.conversations[conversation_id]
            if conversation_id in self.last_accessed:
                del self.last_accessed[conversation_id]

    def _cleanup_expired(self) -> None:
        """Remove expired conversations."""
        current_time = datetime.utcnow()
        expired_ids = [
            cid for cid, last_access in self.last_accessed.items()
            if current_time - last_access > self.conversation_ttl
        ]

        for cid in expired_ids:
            self.clear_conversation(cid)
            logger.debug(f"Cleaned up expired conversation: {cid}")


class ConversationService:
    """
    Service for managing conversation context and history.
    """

    def __init__(self):
        self.memory = ConversationMemory()

    async def add_interaction(self, conversation_id: str, query: str, response: str) -> None:
        """
        Add a query-response interaction to the conversation history.

        Args:
            conversation_id: ID of the conversation
            query: User's query
            response: System's response
        """
        self.memory.add_message(conversation_id, query, response)
        logger.debug(f"Added interaction to conversation {conversation_id}")

    async def get_conversation_context(self, conversation_id: str) -> List[Dict[str, str]]:
        """
        Get the conversation history for context in follow-up questions.

        Args:
            conversation_id: ID of the conversation

        Returns:
            List of query-response pairs for context
        """
        history = self.memory.get_conversation_history(conversation_id)

        # Format for context (simple format for now)
        context = []
        for item in history:
            context.append({
                "query": item["query"],
                "answer": item["response"],
                "timestamp": item["timestamp"].isoformat()
            })

        logger.debug(f"Retrieved {len(context)} items from conversation {conversation_id}")
        return context

    async def create_new_conversation(self) -> str:
        """
        Create a new conversation and return its ID.

        Returns:
            New conversation ID
        """
        conversation_id = self.memory.create_conversation()
        logger.info(f"Created new conversation: {conversation_id}")
        return conversation_id

    async def clear_conversation(self, conversation_id: str) -> None:
        """
        Clear a conversation from memory.

        Args:
            conversation_id: ID of the conversation to clear
        """
        self.memory.clear_conversation(conversation_id)
        logger.info(f"Cleared conversation: {conversation_id}")

    async def get_conversation_summary(self, conversation_id: str) -> Dict[str, Any]:
        """
        Get a summary of the conversation.

        Args:
            conversation_id: ID of the conversation

        Returns:
            Summary information about the conversation
        """
        history = self.memory.get_conversation_history(conversation_id)

        if not history:
            return {
                "conversation_id": conversation_id,
                "message_count": 0,
                "last_activity": None
            }

        last_message = history[-1] if history else None

        return {
            "conversation_id": conversation_id,
            "message_count": len(history),
            "last_activity": last_message["timestamp"].isoformat() if last_message else None,
            "recent_queries": [item["query"] for item in history[-3:]]  # Last 3 queries
        }


# Global instance for easy access
conversation_service = ConversationService()


def get_conversation_service() -> ConversationService:
    """
    Get the conversation service instance.

    Returns:
        ConversationService instance
    """
    return conversation_service