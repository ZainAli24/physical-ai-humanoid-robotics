/**
 * InputBar - Chat input with send button
 * Feature: 004-rag-chat
 */

import React, { useState, useRef, KeyboardEvent } from 'react';
import { useChat } from '../../hooks/useChat';
import styles from './styles.module.css';

/**
 * InputBar Component
 *
 * Features:
 * - Text input (3-1000 chars validation)
 * - Send button (disabled during streaming)
 * - Enter key to send (Shift+Enter for new line)
 * - Auto-resize textarea
 * - Character counter
 */
export default function InputBar(): JSX.Element {
  const { sendMessage, isStreaming } = useChat();
  const [input, setInput] = useState('');
  const [error, setError] = useState<string | null>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Handle input change
  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const value = e.target.value;
    setInput(value);

    // Clear error when typing
    if (error) setError(null);

    // Auto-resize textarea
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 150)}px`;
    }
  };

  // Validate and send message
  const handleSend = async () => {
    const trimmed = input.trim();

    // Validation
    if (trimmed.length < 3) {
      setError('Message must be at least 3 characters');
      return;
    }

    if (trimmed.length > 1000) {
      setError('Message must be at most 1000 characters');
      return;
    }

    // Clear input and reset textarea height
    setInput('');
    setError(null);
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
    }

    // Send message
    await sendMessage(trimmed);
  };

  // Handle Enter key (Shift+Enter for new line)
  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  // Character count
  const charCount = input.length;
  const isValid = charCount >= 3 && charCount <= 1000;
  const showWarning = charCount > 900;

  return (
    <div className={styles.inputBarContainer}>
      {/* Error Message */}
      {error && <div className={styles.inputError}>{error}</div>}

      <div className={styles.inputBar}>
        {/* Textarea */}
        <textarea
          ref={textareaRef}
          className={styles.input}
          placeholder={
            isStreaming
              ? 'Waiting for response...'
              : 'Ask about ROS 2, kinematics, sensors...'
          }
          value={input}
          onChange={handleInputChange}
          onKeyDown={handleKeyDown}
          disabled={isStreaming}
          rows={1}
          maxLength={1000}
        />

        {/* Send Button */}
        <button
          className={styles.sendButton}
          onClick={handleSend}
          disabled={isStreaming || !isValid}
          aria-label="Send message"
          title={isStreaming ? 'Waiting for response' : 'Send message'}
        >
          {isStreaming ? (
            // Loading Spinner
            <svg
              className={styles.spinner}
              xmlns="http://www.w3.org/2000/svg"
              fill="none"
              viewBox="0 0 24 24"
            >
              <circle
                className={styles.spinnerCircle}
                cx="12"
                cy="12"
                r="10"
                stroke="currentColor"
                strokeWidth="4"
              />
              <path
                className={styles.spinnerPath}
                fill="currentColor"
                d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
              />
            </svg>
          ) : (
            // Send Icon
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="currentColor"
              className={styles.icon}
            >
              <path d="M3.478 2.405a.75.75 0 00-.926.94l2.432 7.905H13.5a.75.75 0 010 1.5H4.984l-2.432 7.905a.75.75 0 00.926.94 60.519 60.519 0 0018.445-8.986.75.75 0 000-1.218A60.517 60.517 0 003.478 2.405z" />
            </svg>
          )}
        </button>
      </div>

      {/* Character Counter */}
      <div className={styles.inputFooter}>
        <div className={styles.inputHint}>
          Press Enter to send, Shift+Enter for new line
        </div>
        <div
          className={
            showWarning ? styles.charCountWarning : styles.charCount
          }
        >
          {charCount}/1000
        </div>
      </div>
    </div>
  );
}
