import React, { useState } from 'react';
import OriginalLayout from '@theme-original/Layout';
import { useLocation } from '@docusaurus/router';
import RagChatbot from '@site/src/components/RagChatbot';

export default function Layout(props) {
  const { pathname } = useLocation();

  // Don't show on certain paths
  const shouldHideChatbot = pathname.startsWith('/api/') ||
                           pathname.startsWith('/admin/') ||
                           pathname.includes('embed');

  const [isChatbotOpen, setIsChatbotOpen] = useState(false);

  // Don't render the chatbot at all if it should be hidden
  if (shouldHideChatbot) {
    return <OriginalLayout {...props} />;
  }

  return (
    <>
      <OriginalLayout {...props} />

      {/* Floating chatbot button */}
      {!isChatbotOpen && (
        <button
          onClick={() => setIsChatbotOpen(true)}
          className="rag-chatbot-fab"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            zIndex: 1000,
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#3498db',
            color: 'white',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            border: 'none',
            fontSize: '24px',
            transition: 'all 0.3s ease'
          }}
          onMouseEnter={(e) => e.target.style.transform = 'scale(1.1)'}
          onMouseLeave={(e) => e.target.style.transform = 'scale(1)'}
          aria-label="Open AI Assistant"
          title="Physical AI Textbook Assistant"
        >
          🤖
        </button>
      )}

      {/* Chatbot panel - only show when open */}
      {isChatbotOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '85px',
            right: '20px',
            zIndex: 1000,
            width: '400px',
            height: '500px',
            background: 'white',
            borderRadius: '12px',
            boxShadow: '0 8px 30px rgba(0, 0, 0, 0.2)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            border: '1px solid #e1e5e9'
          }}
        >
          <div
            style={{
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              color: 'white',
              padding: '12px 16px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <span>Physical AI Assistant</span>
            <button
              onClick={() => setIsChatbotOpen(false)}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '18px',
                cursor: 'pointer',
                padding: '4px',
                marginLeft: '10px'
              }}
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
}