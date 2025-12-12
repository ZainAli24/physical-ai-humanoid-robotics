/**
 * AddToChatButton Component
 * Feature: 005-text-selection
 *
 * Floating button that appears near text selection to add it to chat context.
 * Tasks: T016, T017, T018
 */

import React, { useEffect, useState } from 'react';
import styles from './styles.module.css';

export interface AddToChatButtonProps {
  /** Position of the button (absolute coordinates) */
  position: {
    top: number;
    left: number;
  };
  /** Callback when button is clicked */
  onClick: () => void;
  /** Whether to show the button (for fade-in animation) */
  visible: boolean;
}

/**
 * Floating "Add to Chat" button with smart positioning.
 *
 * Features:
 * - Purple gradient styling matching chatbot theme (T016)
 * - Smart positioning to avoid viewport edges (T017)
 * - Keyboard accessibility with aria-label and Enter key handler (T018)
 * - Fade-in/fade-out animations
 * - High z-index to appear above page content
 *
 * Positioning Algorithm (T017):
 * - Checks if button would go off-screen (right or bottom edge)
 * - Adjusts position to stay within viewport bounds
 * - Maintains minimum margin from viewport edges (10px)
 *
 * Accessibility (T018):
 * - tabIndex={0} for keyboard navigation
 * - aria-label for screen readers
 * - Enter key triggers onClick
 * - Focus visible styles for keyboard users
 */
export function AddToChatButton({ position, onClick, visible }: AddToChatButtonProps) {
  const [adjustedPosition, setAdjustedPosition] = useState(position);

  useEffect(() => {
    // T017: Smart positioning algorithm to avoid viewport edges
    const buttonWidth = 120; // Approximate button width
    const buttonHeight = 36; // Approximate button height
    const viewportMargin = 10; // Minimum margin from viewport edges

    const viewportWidth = window.innerWidth;
    const viewportHeight = window.innerHeight;

    let { top, left } = position;

    // Adjust horizontal position if button would go off-screen (right edge)
    if (left + buttonWidth + viewportMargin > viewportWidth) {
      left = viewportWidth - buttonWidth - viewportMargin;
    }

    // Ensure minimum left margin
    if (left < viewportMargin) {
      left = viewportMargin;
    }

    // Adjust vertical position if button would go off-screen (bottom edge)
    if (top + buttonHeight + viewportMargin > viewportHeight + window.scrollY) {
      // Position above selection instead of below
      top = position.top - buttonHeight - 20; // 20px above selection
    }

    // Ensure minimum top margin
    if (top < window.scrollY + viewportMargin) {
      top = window.scrollY + viewportMargin;
    }

    setAdjustedPosition({ top, left });
  }, [position]);

  const handleKeyDown = (event: React.KeyboardEvent) => {
    // T018: Enter key handler for keyboard accessibility
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      onClick();
    }
  };

  return (
    <button
      className={`${styles.addToChatButton} ${visible ? styles.addToChatButtonVisible : ''}`}
      style={{
        position: 'absolute',
        top: `${adjustedPosition.top}px`,
        left: `${adjustedPosition.left}px`,
      }}
      onClick={onClick}
      onKeyDown={handleKeyDown}
      tabIndex={0} // T018: Keyboard accessibility
      aria-label="Add selected text to chat" // T018: Screen reader support
      type="button"
    >
      ✨ Add to Chat
    </button>
  );
}
