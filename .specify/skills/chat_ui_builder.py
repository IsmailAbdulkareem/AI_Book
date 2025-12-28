"""Skill 8: Chat UI Builder - React chat widget, Docusaurus integration, styling, loading states"""

from pathlib import Path
from typing import Dict, Any


class ChatUiBuilder:
    """Creates and manages React chat widget for Docusaurus."""

    def __init__(self, repo_root: str = "."):
        self.repo_root = Path(repo_root)
        self.src_dir = self.repo_root / "src"
        self.components_dir = self.src_dir / "components"

    def create_chat_widget(self) -> Dict[str, Any]:
        """Create React chat widget component."""
        try:
            self.components_dir.mkdir(parents=True, exist_ok=True)
            
            widget_code = '''import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

export default function ChatWidget() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { role: 'user', content: input };
    setMessages([...messages, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: input }),
      });

      const data = await response.json();
      const assistantMessage = {
        role: 'assistant',
        content: data.response,
        sources: data.sources,
      };
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error:', error);
      setMessages((prev) => [
        ...prev,
        { role: 'assistant', content: 'Error: Unable to get response.' },
      ]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.widget}>
      <div className={styles.header}>
        <h3>AI Tutor</h3>
      </div>
      <div className={styles.messagesContainer}>
        {messages.length === 0 && (
          <div className={styles.welcome}>
            <p>Ask me about Physical AI & Humanoid Robotics!</p>
          </div>
        )}
        {messages.map((msg, idx) => (
          <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
            <div className={styles.content}>{msg.content}</div>
            {msg.sources && msg.sources.length > 0 && (
              <div className={styles.sources}>
                <small>Sources: {msg.sources.join(', ')}</small>
              </div>
            )}
          </div>
        ))}
        {loading && (
          <div className={styles.loading}>
            <div className={styles.spinner}></div>
            <p>Thinking...</p>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>
      <div className={styles.inputContainer}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
          placeholder="Ask a question..."
          disabled={loading}
        />
        <button onClick={sendMessage} disabled={loading}>
          Send
        </button>
      </div>
    </div>
  );
}
'''
            
            widget_file = self.components_dir / "ChatWidget.js"
            widget_file.write_text(widget_code)
            
            return {"status": "success", "file": str(widget_file)}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def create_chat_styles(self) -> Dict[str, Any]:
        """Create CSS module for chat widget."""
        try:
            styles_code = '''.widget {
  display: flex;
  flex-direction: column;
  height: 500px;
  border: 1px solid #ccc;
  border-radius: 8px;
  background: white;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  overflow: hidden;
}

.header {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  padding: 16px;
  font-weight: bold;
}

.messagesContainer {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
}

.message {
  margin: 12px 0;
  padding: 12px;
  border-radius: 6px;
  max-width: 80%;
}

.message.user {
  align-self: flex-end;
  background: #667eea;
  color: white;
  margin-left: auto;
}

.message.assistant {
  align-self: flex-start;
  background: #f0f0f0;
  color: black;
}

.content {
  word-wrap: break-word;
}

.sources {
  margin-top: 8px;
  font-size: 12px;
  opacity: 0.7;
}

.welcome {
  text-align: center;
  color: #999;
  padding: 40px 20px;
}

.loading {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px;
}

.spinner {
  width: 20px;
  height: 20px;
  border: 3px solid #f3f3f3;
  border-top: 3px solid #667eea;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.inputContainer {
  display: flex;
  gap: 8px;
  padding: 12px;
  border-top: 1px solid #eee;
}

.inputContainer input {
  flex: 1;
  padding: 10px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
}

.inputContainer button {
  padding: 10px 20px;
  background: #667eea;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-weight: bold;
}

.inputContainer button:hover:not(:disabled) {
  background: #764ba2;
}

.inputContainer button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}
'''
            
            styles_file = self.components_dir / "ChatWidget.module.css"
            styles_file.write_text(styles_code)
            
            return {"status": "success", "file": str(styles_file)}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def embed_in_docusaurus(self) -> Dict[str, Any]:
        """Add chat widget to Docusaurus layout."""
        try:
            # Create a page or update existing to embed widget
            instructions = """
To embed the ChatWidget in Docusaurus:

1. Import in your pages or components:
   import ChatWidget from '@site/src/components/ChatWidget';

2. Add to JSX:
   <ChatWidget />

3. Ensure FastAPI backend is running on http://localhost:8000

4. Update CORS in backend to allow Docusaurus origin if deployed
"""
            return {"status": "success", "instructions": instructions}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def add_tailwind(self) -> Dict[str, Any]:
        """Add Tailwind CSS support (optional enhancement)."""
        try:
            # Instructions for Tailwind setup
            instructions = """
To add Tailwind CSS to Docusaurus:

1. Install dependencies:
   npm install -D tailwindcss postcss autoprefixer

2. Initialize config:
   npx tailwindcss init -p

3. Configure in docusaurus.config.js postcss plugin

4. Update ChatWidget to use Tailwind classes
"""
            return {"status": "success", "instructions": instructions}
        except Exception as e:
            return {"status": "error", "message": str(e)}
