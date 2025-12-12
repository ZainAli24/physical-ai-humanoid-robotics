/**
 * SelectedTextDisplay Component
 * Feature: 005-text-selection
 *
 * Badge showing selected text context in the chat modal.
 * Task: T019
 */

import React from 'react';
import styles from './styles.module.css';

export interface SelectedTextDisplayProps {
  /** Selected text from the textbook */
  selectedText: string;
  /** Callback to clear/remove selected text */
  onClear?: () => void;
}

/**
 * Badge displaying "About: [text]..." above the chat input.
 *
 * Features:
 * - Shows first 50 characters of selected text with ellipsis
 * - Light purple background matching chatbot theme
 * - Optional clear button (X) to remove selected text
 * - Rounded corners and padding for visual appeal
 *
 * Display Format:
 * - Short text (< 50 chars): "About: ROS 2 nodes are fundamental"
 * - Long text (>= 50 chars): "About: A node is a participant in the ROS 2 graph..."
 */
export function SelectedTextDisplay({ selectedText, onClear }: SelectedTextDisplayProps) {
  const MAX_DISPLAY_LENGTH = 50;

  // Truncate text if needed
  const displayText = selectedText.length > MAX_DISPLAY_LENGTH
    ? `${selectedText.substring(0, MAX_DISPLAY_LENGTH)}...`
    : selectedText;

  return (
    <div className={styles.selectedTextDisplay}>
      <span className={styles.selectedTextLabel}>About:</span>
      <span className={styles.selectedTextContent}>{displayText}</span>
      {onClear && (
        <button
          className={styles.selectedTextClear}
          onClick={onClear}
          aria-label="Clear selected text"
          type="button"
        >
          ×
        </button>
      )}
    </div>
  );
}
