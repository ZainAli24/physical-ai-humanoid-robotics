/**
 * ChatbotWidget - Floating chat button and modal
 * Feature: 004-rag-chat (Updated for 005-text-selection)
 */

import React from 'react';
import { useChat } from '../../hooks/useChat';
import { useTextSelection } from '../../hooks/useTextSelection'; // T027
import ChatModal from './ChatModal';
import { AddToChatButton } from './AddToChatButton'; // T027
import styles from './styles.module.css';

/**
 * ChatbotWidget Component
 *
 * Renders:
 * - Floating chat button (bottom-right corner)
 * - Chat modal (when open)
 * - Add to Chat button (when text is selected) - Feature 005
 */
export default function ChatbotWidget(): JSX.Element {
  const { isOpen, openChat, closeChat, setSelectedText } = useChat();
  const { text, isValid, buttonPosition } = useTextSelection(); // T027

  // T028: Wire AddToChatButton onClick to setSelectedText and open modal
  const handleAddToChat = () => {
    if (isValid && text) {
      setSelectedText(text);
      openChat();
    }
  };

  return (
    <>
      {/* Floating Chat Button */}
      {!isOpen && (
        <button
          className={styles.floatingButton}
          onClick={openChat}
          aria-label="Open chatbot"
          title="Ask a question about the textbook"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="currentColor"
            className={styles.icon}
          >
            <path
              fillRule="evenodd"
              d="M4.848 2.771A49.144 49.144 0 0112 2.25c2.43 0 4.817.178 7.152.52 1.978.292 3.348 2.024 3.348 3.97v6.02c0 1.946-1.37 3.678-3.348 3.97a48.901 48.901 0 01-3.476.383.39.39 0 00-.297.17l-2.755 4.133a.75.75 0 01-1.248 0l-2.755-4.133a.39.39 0 00-.297-.17 48.9 48.9 0 01-3.476-.384c-1.978-.29-3.348-2.024-3.348-3.97V6.741c0-1.946 1.37-3.678 3.348-3.97z"
              clipRule="evenodd"
            />
          </svg>
        </button>
      )}

      {/* T027, T028: Add to Chat Button (appears when valid text is selected) */}
      {!isOpen && isValid && buttonPosition && (
        <AddToChatButton
          position={buttonPosition}
          onClick={handleAddToChat}
          visible={true}
        />
      )}

      {/* Chat Modal */}
      {isOpen && <ChatModal onClose={closeChat} />}
    </>
  );
}
