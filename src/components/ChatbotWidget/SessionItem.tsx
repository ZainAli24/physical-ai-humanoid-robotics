/**
 * SessionItem - Individual chat session list item
 * Feature: 006-auth-sessions - User Story 3 & 6 (T083, T084)
 *
 * Displays:
 * - Session title (inline editable on click)
 * - Delete icon (shows on hover)
 * - Message count badge
 * - Last updated timestamp
 */

import React, { useState, useRef, useEffect } from 'react';
import { ChatSession } from '../../hooks/useChatHistory';
import styles from './styles.module.css';

interface SessionItemProps {
  session: ChatSession;
  isActive: boolean;
  onSelect: (sessionId: string) => void;
  onRename: (sessionId: string, newTitle: string) => Promise<boolean>;
  onDelete: (sessionId: string) => Promise<void>;
}

export default function SessionItem({
  session,
  isActive,
  onSelect,
  onRename,
  onDelete,
}: SessionItemProps): JSX.Element {
  const [isEditing, setIsEditing] = useState(false);
  const [editTitle, setEditTitle] = useState(session.title);
  const [isHovered, setIsHovered] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);

  // Focus input when entering edit mode
  useEffect(() => {
    if (isEditing && inputRef.current) {
      inputRef.current.focus();
      inputRef.current.select();
    }
  }, [isEditing]);

  // T083: Handle title click to enter edit mode
  const handleTitleClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    if (!isEditing) {
      setIsEditing(true);
    }
  };

  // T083: Save title on blur or Enter key
  const handleSave = async () => {
    if (!isEditing) return;

    const trimmedTitle = editTitle.trim();

    // Validate title
    if (!trimmedTitle) {
      setEditTitle(session.title); // Reset to original
      setIsEditing(false);
      return;
    }

    // Only save if title changed
    if (trimmedTitle !== session.title) {
      const success = await onRename(session.id, trimmedTitle);
      if (!success) {
        setEditTitle(session.title); // Rollback on error
      }
    }

    setIsEditing(false);
  };

  // Handle Enter and Escape keys
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      handleSave();
    } else if (e.key === 'Escape') {
      setEditTitle(session.title); // Reset
      setIsEditing(false);
    }
  };

  // T084: Handle delete button click
  const handleDeleteClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    onDelete(session.id);
  };

  // Format date (e.g., "2 hours ago", "Dec 9")
  const formatDate = (dateString: string): string => {
    const date = new Date(dateString);
    const now = new Date();
    const diffMs = now.getTime() - date.getTime();
    const diffMins = Math.floor(diffMs / 60000);

    if (diffMins < 1) return 'Just now';
    if (diffMins < 60) return `${diffMins}m ago`;

    const diffHours = Math.floor(diffMins / 60);
    if (diffHours < 24) return `${diffHours}h ago`;

    const diffDays = Math.floor(diffHours / 24);
    if (diffDays < 7) return `${diffDays}d ago`;

    return date.toLocaleDateString('en-US', { month: 'short', day: 'numeric' });
  };

  return (
    <div
      className={`${styles.sessionItem} ${isActive ? styles.sessionItemActive : ''}`}
      onClick={() => !isEditing && onSelect(session.id)}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      role="button"
      tabIndex={0}
      aria-label={`Chat session: ${session.title}`}
    >
      <div className={styles.sessionItemContent}>
        {/* T083: Inline editable title */}
        {isEditing ? (
          <input
            ref={inputRef}
            type="text"
            className={styles.sessionTitleInput}
            value={editTitle}
            onChange={(e) => setEditTitle(e.target.value)}
            onBlur={handleSave}
            onKeyDown={handleKeyDown}
            maxLength={200}
            aria-label="Edit session title"
          />
        ) : (
          <div className={styles.sessionTitle} onClick={handleTitleClick}>
            {session.title}
          </div>
        )}

        <div className={styles.sessionMeta}>
          <span className={styles.sessionDate}>{formatDate(session.updated_at)}</span>
          {session.message_count > 0 && (
            <span className={styles.sessionCount}>{session.message_count} messages</span>
          )}
        </div>
      </div>

      {/* T084: Delete icon (show on hover) */}
      {isHovered && !isEditing && (
        <button
          className={styles.sessionDeleteButton}
          onClick={handleDeleteClick}
          aria-label="Delete session"
          title="Delete this chat session"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="currentColor"
            width="18"
            height="18"
          >
            <path
              fillRule="evenodd"
              d="M16.5 4.478v.227a48.816 48.816 0 013.878.512.75.75 0 11-.256 1.478l-.209-.035-1.005 13.07a3 3 0 01-2.991 2.77H8.084a3 3 0 01-2.991-2.77L4.087 6.66l-.209.035a.75.75 0 01-.256-1.478A48.567 48.567 0 017.5 4.705v-.227c0-1.564 1.213-2.9 2.816-2.951a52.662 52.662 0 013.369 0c1.603.051 2.815 1.387 2.815 2.951zm-6.136-1.452a51.196 51.196 0 013.273 0C14.39 3.05 15 3.684 15 4.478v.113a49.488 49.488 0 00-6 0v-.113c0-.794.609-1.428 1.364-1.452zm-.355 5.945a.75.75 0 10-1.5.058l.347 9a.75.75 0 101.499-.058l-.346-9zm5.48.058a.75.75 0 10-1.498-.058l-.347 9a.75.75 0 001.5.058l.345-9z"
              clipRule="evenodd"
            />
          </svg>
        </button>
      )}
    </div>
  );
}
