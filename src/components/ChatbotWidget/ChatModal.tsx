/**
 * ChatModal - Full-screen chat modal overlay
 * Feature: 004-rag-chat (Updated for 005-text-selection, 006-auth-sessions)
 */

import React, { useEffect, useRef, useState } from 'react';
import { useChat } from '../../hooks/useChat';
import { useAuth } from '../../hooks/useAuth'; // T047: Import auth hook
import MessageList from './MessageList';
import InputBar from './InputBar';
import { SelectedTextDisplay } from './SelectedTextDisplay'; // Feature 005 - T021
import ChatHistorySidebar from './ChatHistorySidebar'; // Feature 006 - User Story 3 & 6
import styles from './styles.module.css';

interface ChatModalProps {
  onClose: () => void;
}

/**
 * ChatModal Component
 *
 * Renders:
 * - Modal overlay (full-screen on mobile <768px)
 * - Header with title and close button
 * - MessageList (scrollable)
 * - InputBar (fixed at bottom)
 */
export default function ChatModal({ onClose }: ChatModalProps): JSX.Element {
  const { messages, clearChat, selectedText, setSelectedText, startNewChat, sessionId } = useChat(); // T021, T047
  const { isAuthenticated } = useAuth(); // T047: Check if user is authenticated
  const modalRef = useRef<HTMLDivElement>(null);
  const [isSidebarVisible, setIsSidebarVisible] = useState(false); // T083: Sidebar toggle state

  // Handle Escape key to close modal
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, [onClose]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    document.body.style.overflow = 'hidden';
    return () => {
      document.body.style.overflow = '';
    };
  }, []);

  // Handle click outside to close (optional, disabled for now)
  const handleOverlayClick = (e: React.MouseEvent) => {
    if (e.target === modalRef.current) {
      // onClose(); // Uncomment to enable click-outside-to-close
    }
  };

  return (
    <div
      ref={modalRef}
      className={styles.modalOverlay}
      onClick={handleOverlayClick}
    >
      <div className={styles.modalContainer}>
        {/* Header */}
        <div className={styles.modalHeader}>
          <div className={styles.headerLeft}>
            {/* T083: My Chats Button (only for authenticated users) */}
            {isAuthenticated && (
              <button
                className={styles.myChatsButton}
                onClick={() => setIsSidebarVisible(!isSidebarVisible)}
                aria-label="Toggle chat history"
                title="View my chats"
              >
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  viewBox="0 0 24 24"
                  fill="currentColor"
                  width="20"
                  height="20"
                >
                  <path d="M3.75 6.75h16.5M3.75 12h16.5m-16.5 5.25H12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" fill="none" />
                </svg>
                <span>My Chats</span>
              </button>
            )}

            <h2 className={styles.modalTitle}>
              Ask about Physical AI & Humanoid Robotics
            </h2>
          </div>

          <div className={styles.headerButtons}>
            {/* T047: New Chat Button (only for authenticated users) */}
            {isAuthenticated && messages.length > 0 && (
              <button
                className={styles.newChatButton}
                onClick={startNewChat}
                aria-label="Start new chat"
                title="Start a new chat session"
              >
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  viewBox="0 0 24 24"
                  fill="currentColor"
                  className={styles.icon}
                  width="20"
                  height="20"
                >
                  <path d="M5.25 6.375a4.125 4.125 0 118.25 0 4.125 4.125 0 01-8.25 0zM2.25 19.125a7.125 7.125 0 0114.25 0v.003l-.001.119a.75.75 0 01-.363.63 13.067 13.067 0 01-6.761 1.873c-2.472 0-4.786-.684-6.76-1.873a.75.75 0 01-.364-.63l-.001-.122zM18.75 7.5a.75.75 0 00-1.5 0v2.25H15a.75.75 0 000 1.5h2.25v2.25a.75.75 0 001.5 0v-2.25H21a.75.75 0 000-1.5h-2.25V7.5z" />
                </svg>
                <span>New Chat</span>
              </button>
            )}

            {/* Clear Chat Button (only for anonymous users) */}
            {!isAuthenticated && messages.length > 0 && (
              <button
                className={styles.clearButton}
                onClick={clearChat}
                aria-label="Clear chat"
                title="Clear chat history"
              >
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  viewBox="0 0 24 24"
                  fill="currentColor"
                  className={styles.icon}
                  width="20"
                  height="20"
                >
                  <path
                    fillRule="evenodd"
                    d="M16.5 4.478v.227a48.816 48.816 0 013.878.512.75.75 0 11-.256 1.478l-.209-.035-1.005 13.07a3 3 0 01-2.991 2.77H8.084a3 3 0 01-2.991-2.77L4.087 6.66l-.209.035a.75.75 0 01-.256-1.478A48.567 48.567 0 017.5 4.705v-.227c0-1.564 1.213-2.9 2.816-2.951a52.662 52.662 0 013.369 0c1.603.051 2.815 1.387 2.815 2.951zm-6.136-1.452a51.196 51.196 0 013.273 0C14.39 3.05 15 3.684 15 4.478v.113a49.488 49.488 0 00-6 0v-.113c0-.794.609-1.428 1.364-1.452zm-.355 5.945a.75.75 0 10-1.5.058l.347 9a.75.75 0 101.499-.058l-.346-9zm5.48.058a.75.75 0 10-1.498-.058l-.347 9a.75.75 0 001.5.058l.345-9z"
                    clipRule="evenodd"
                  />
                </svg>
              </button>
            )}

            {/* Close Button */}
            <button
              className={styles.closeButton}
              onClick={onClose}
              aria-label="Close chat"
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                viewBox="0 0 24 24"
                fill="currentColor"
                className={styles.icon}
              >
                <path
                  fillRule="evenodd"
                  d="M5.47 5.47a.75.75 0 011.06 0L12 10.94l5.47-5.47a.75.75 0 111.06 1.06L13.06 12l5.47 5.47a.75.75 0 11-1.06 1.06L12 13.06l-5.47 5.47a.75.75 0 01-1.06-1.06L10.94 12 5.47 6.53a.75.75 0 010-1.06z"
                  clipRule="evenodd"
                />
              </svg>
            </button>
          </div>
        </div>

        {/* Message List (scrollable) */}
        <MessageList />

        {/* T021: Display selected text above input bar */}
        {selectedText && (
          <SelectedTextDisplay
            selectedText={selectedText}
            onClear={() => setSelectedText(null)}
          />
        )}

        {/* Input Bar (fixed at bottom) */}
        <InputBar />
      </div>

      {/* T083-T087: Chat History Sidebar (Feature 006 - User Story 3 & 6) */}
      {isAuthenticated && (
        <ChatHistorySidebar
          isVisible={isSidebarVisible}
          onClose={() => setIsSidebarVisible(false)}
        />
      )}
    </div>
  );
}
