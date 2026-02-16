import React, { useState, useRef, useEffect } from 'react';
import './RagChatbot.css';

const RagChatbot = ({ apiUrl = '/chat/' }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [conversationId, setConversationId] = useState(null);
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText.length > 0) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request body
      const requestBody = {
        query: inputValue,
        selected_text: selectedText || undefined,
        context: {
          conversation_id: conversationId || undefined
        }
      };

      // Send request to backend - using full URL to backend service
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000/chat/';
      const response = await fetch(backendUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Update conversation ID if it changed
      if (data.conversation_id && data.conversation_id !== conversationId) {
        setConversationId(data.conversation_id);
      }

      // Create bot response message
      const botMessage = {
        id: `bot-${Date.now()}`,
        text: data.answer,
        sender: 'bot',
        citations: data.citations,
        confidence: data.confidence,
        timestamp: data.timestamp
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
        id: `error-${Date.now()}`,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        isError: true,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const copyCitation = (citation) => {
    const citationText = `See: ${citation.chapter}${citation.section ? `, Section ${citation.section}` : ''}`;
    navigator.clipboard.writeText(citationText);
  };

  return (
    <div className="rag-chatbot">
      <div className="chat-header">
        <h3>Physical AI Textbook Assistant</h3>
        <p>Ask questions about the textbook content</p>
      </div>

      <div className="chat-messages">
        {messages.length === 0 && (
          <div className="welcome-message">
            <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
            <p>You can ask me questions about the textbook content, or select text on the page and ask about it.</p>
          </div>
        )}

        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.sender}-message`}
          >
            <div className="message-content">
              <p>{message.text}</p>

              {message.citations && message.citations.length > 0 && (
                <div className="citations">
                  <strong>Citations:</strong>
                  {message.citations.slice(0, 3).map((citation, idx) => (
                    <span key={idx} className="citation">
                      <a href={`#${citation.chapter.replace(/\s+/g, '-').toLowerCase()}`}>
                        {citation.chapter}
                      </a>
                      {citation.section && (
                        <span>, Section <a href={`#${citation.section.replace(/\s+/g, '-').toLowerCase()}`}>{citation.section}</a></span>
                      )}
                      <button
                        onClick={() => copyCitation(citation)}
                        className="copy-citation-btn"
                        title="Copy citation"
                      >
                        📋
                      </button>
                    </span>
                  ))}
                  {message.citations.length > 3 && (
                    <span className="more-citations">and {message.citations.length - 3} more</span>
                  )}
                </div>
              )}

              {message.confidence !== undefined && (
                <div className="confidence">
                  Confidence: {(message.confidence * 100).toFixed(0)}%
                </div>
              )}

              {message.isError && (
                <div className="error-message">
                  Error occurred while processing your request
                </div>
              )}
            </div>
          </div>
        ))}

        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {selectedText && (
        <div className="selected-text-preview">
          <small>Including selected text:</small>
          <div className="selected-text">{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}</div>
        </div>
      )}

      <form className="chat-input-form" onSubmit={handleSubmit}>
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the textbook content..."
          disabled={isLoading}
          rows={3}
        />
        <button type="submit" disabled={!inputValue.trim() || isLoading}>
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default RagChatbot;