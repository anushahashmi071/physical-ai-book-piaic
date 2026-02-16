import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';

const ChatbotInjector = () => {
  const location = useLocation();

  useEffect(() => {
    // Add chatbot styles if not already present
    const styleId = 'rag-chatbot-global-styles';
    if (!document.getElementById(styleId)) {
      const style = document.createElement('style');
      style.id = styleId;
      style.textContent = `
        .rag-chatbot-fab {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 1000;
          width: 60px;
          height: 60px;
          border-radius: 50%;
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          color: white;
          display: flex;
          align-items: center;
          justify-content: center;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
          border: none;
          font-size: 24px;
          transition: all 0.3s ease;
        }

        .rag-chatbot-fab:hover {
          transform: scale(1.1);
          box-shadow: 0 6px 16px rgba(0, 0, 0, 0.2);
        }

        .rag-chatbot-container {
          position: fixed;
          bottom: 85px;
          right: 20px;
          z-index: 1000;
          width: 400px;
          height: 500px;
          background: white;
          border-radius: 12px;
          box-shadow: 0 8px 30px rgba(0, 0, 0, 0.2);
          display: flex;
          flex-direction: column;
          overflow: hidden;
          border: 1px solid #e1e5e9;
        }

        .rag-chatbot-header {
          background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
          color: white;
          padding: 12px 16px;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .rag-chatbot-close {
          background: none;
          border: none;
          color: white;
          font-size: 18px;
          cursor: pointer;
          padding: 4px;
        }
      `;
      document.head.appendChild(style);
    }
  }, []);

  // Don't show on certain pages like admin pages or iframes
  const shouldShow = !location.pathname.includes('/admin') &&
                     !location.pathname.includes('/api') &&
                     !location.hash.includes('embedded');

  if (!shouldShow) {
    return null;
  }

  return (
    <BrowserOnly>
      {() => {
        const [isOpen, setIsOpen] = React.useState(false);

        const toggleChatbot = () => {
          setIsOpen(!isOpen);
        };

        return (
          <>
            {!isOpen && (
              <button
                className="rag-chatbot-fab"
                onClick={toggleChatbot}
                aria-label="Open AI Assistant"
                title="Physical AI Textbook Assistant"
              >
                🤖
              </button>
            )}

            {isOpen && (
              <div className="rag-chatbot-container">
                <div className="rag-chatbot-header">
                  <span>Physical AI Assistant</span>
                  <button
                    className="rag-chatbot-close"
                    onClick={toggleChatbot}
                    aria-label="Close chatbot"
                  >
                    ×
                  </button>
                </div>
                <div style={{ flex: 1, overflow: 'hidden' }}>
                  <RagChatbot />
                </div>
              </div>
            )}
          </>
        );
      }}
    </BrowserOnly>
  );
};

export default ChatbotInjector;