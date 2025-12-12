/**
 * ChatHistorySidebar - Sidebar showing user's chat sessions
 * Feature: 006-auth-sessions - User Story 3 & 6
 *
 * Displays:
 * - List of user's chat sessions
 * - "New Chat" button
 * - Empty state when no sessions exist
 * - Loading and error states
 */

import React, { useEffect, useState } from 'react';
import { useChatHistory } from '../../hooks/useChatHistory';
import { useChat } from '../../hooks/useChat';
import SessionItem from './SessionItem';
import ConfirmDialog from './ConfirmDialog';
import styles from './styles.module.css';

interface ChatHistorySidebarProps {
  isVisible: boolean;
  onClose: () => void;
}

export default function ChatHistorySidebar({
  isVisible,
  onClose,
}: ChatHistorySidebarProps): JSX.Element {
  const {
    sessions,
    isLoadingSessions,
    error,
    loadSessions,
    renameSession,
    deleteSession,
  } = useChatHistory();

  const { sessionId, startNewChat, loadMessagesFromSession } = useChat();

  // Confirmation dialog state
  const [confirmDialog, setConfirmDialog] = useState<{
    isOpen: boolean;
    sessionId: string | null;
    sessionTitle: string | null;
  }>({
    isOpen: false,
    sessionId: null,
    sessionTitle: null,
  });

  // Load sessions on mount
  useEffect(() => {
    if (isVisible) {
      loadSessions();
    }
  }, [isVisible, loadSessions]);

  // T086: Handle session selection with optimistic UI
  const handleSelectSession = async (selectedSessionId: string) => {
    if (selectedSessionId === sessionId) return; // Already active

    try {
      await loadMessagesFromSession(selectedSessionId);
    } catch (error) {
      console.error('Failed to load session:', error);
    }
  };

  // T083, T086: Handle session rename with optimistic UI
  const handleRenameSession = async (
    sessionIdToRename: string,
    newTitle: string
  ): Promise<boolean> => {
    try {
      const success = await renameSession(sessionIdToRename, newTitle);
      return success;
    } catch (error) {
      console.error('Failed to rename session:', error);
      return false;
    }
  };

  // T084, T085: Show confirmation before delete
  const handleDeleteClick = (sessionIdToDelete: string) => {
    const session = sessions.find((s) => s.id === sessionIdToDelete);
    if (!session) return;

    setConfirmDialog({
      isOpen: true,
      sessionId: sessionIdToDelete,
      sessionTitle: session.title,
    });
  };

  // T085, T087: Handle session deletion with active session check
  const handleConfirmDelete = async () => {
    const { sessionId: sessionIdToDelete } = confirmDialog;
    if (!sessionIdToDelete) return;

    try {
      const isActiveSession = sessionIdToDelete === sessionId;

      // Delete the session
      const success = await deleteSession(sessionIdToDelete);

      if (success) {
        // T087: If active session was deleted, start new chat
        if (isActiveSession) {
          startNewChat();
        }
      }
    } catch (error) {
      console.error('Failed to delete session:', error);
    } finally {
      setConfirmDialog({ isOpen: false, sessionId: null, sessionTitle: null });
    }
  };

  // Cancel delete
  const handleCancelDelete = () => {
    setConfirmDialog({ isOpen: false, sessionId: null, sessionTitle: null });
  };

  // Handle new chat
  const handleNewChat = () => {
    startNewChat();
    onClose(); // Close sidebar on mobile after selecting
  };

  return (
    <>
      <div className={`${styles.historySidebar} ${isVisible ? styles.historySidebarVisible : ''}`}>
        {/* Sidebar Header */}
        <div className={styles.sidebarHeader}>
          <h3 className={styles.sidebarTitle}>My Chats</h3>

          {/* New Chat Button */}
          <button
            className={styles.sidebarNewChatButton}
            onClick={handleNewChat}
            aria-label="Start new chat"
            title="Start a new chat"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="currentColor"
              width="20"
              height="20"
            >
              <path d="M5.25 6.375a4.125 4.125 0 118.25 0 4.125 4.125 0 01-8.25 0zM2.25 19.125a7.125 7.125 0 0114.25 0v.003l-.001.119a.75.75 0 01-.363.63 13.067 13.067 0 01-6.761 1.873c-2.472 0-4.786-.684-6.76-1.873a.75.75 0 01-.364-.63l-.001-.122zM18.75 7.5a.75.75 0 00-1.5 0v2.25H15a.75.75 0 000 1.5h2.25v2.25a.75.75 0 001.5 0v-2.25H21a.75.75 0 000-1.5h-2.25V7.5z" />
            </svg>
          </button>

          {/* Close button (mobile only) */}
          <button
            className={styles.sidebarCloseButton}
            onClick={onClose}
            aria-label="Close sidebar"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="currentColor"
              width="24"
              height="24"
            >
              <path
                fillRule="evenodd"
                d="M5.47 5.47a.75.75 0 011.06 0L12 10.94l5.47-5.47a.75.75 0 111.06 1.06L13.06 12l5.47 5.47a.75.75 0 11-1.06 1.06L12 13.06l-5.47 5.47a.75.75 0 01-1.06-1.06L10.94 12 5.47 6.53a.75.75 0 010-1.06z"
                clipRule="evenodd"
              />
            </svg>
          </button>
        </div>

        {/* Sidebar Content */}
        <div className={styles.sidebarContent}>
          {/* Loading State */}
          {isLoadingSessions && (
            <div className={styles.sidebarLoading}>
              <div className={styles.spinner} />
              <p>Loading your chats...</p>
            </div>
          )}

          {/* Error State */}
          {error && !isLoadingSessions && (
            <div className={styles.sidebarError}>
              <p>Failed to load chat history</p>
              <button onClick={loadSessions} className={styles.retryButton}>
                Retry
              </button>
            </div>
          )}

          {/* Empty State */}
          {!isLoadingSessions && !error && sessions.length === 0 && (
            <div className={styles.sidebarEmpty}>
              <p>Your conversations will appear here</p>
              <p className={styles.sidebarEmptyHint}>
                Start a new chat to begin!
              </p>
            </div>
          )}

          {/* Session List */}
          {!isLoadingSessions && !error && sessions.length > 0 && (
            <div className={styles.sessionList}>
              {sessions.map((session) => (
                <SessionItem
                  key={session.id}
                  session={session}
                  isActive={session.id === sessionId}
                  onSelect={handleSelectSession}
                  onRename={handleRenameSession}
                  onDelete={handleDeleteClick}
                />
              ))}
            </div>
          )}
        </div>
      </div>

      {/* T085: Confirmation Dialog */}
      <ConfirmDialog
        isOpen={confirmDialog.isOpen}
        title="Delete Chat Session?"
        message={`Are you sure you want to delete "${confirmDialog.sessionTitle}"? This will permanently remove all messages in this conversation.`}
        confirmLabel="Delete"
        cancelLabel="Cancel"
        isDestructive={true}
        onConfirm={handleConfirmDelete}
        onCancel={handleCancelDelete}
      />
    </>
  );
}
