/**
 * ConfirmDialog - Reusable confirmation dialog
 * Feature: 006-auth-sessions - User Story 5 & 6 (T085)
 *
 * Shows a modal dialog with:
 * - Title
 * - Message
 * - Confirm button (destructive action)
 * - Cancel button
 */

import React, { useEffect } from 'react';
import styles from './styles.module.css';

interface ConfirmDialogProps {
  isOpen: boolean;
  title: string;
  message: string;
  confirmLabel?: string;
  cancelLabel?: string;
  onConfirm: () => void;
  onCancel: () => void;
  isDestructive?: boolean;
}

export default function ConfirmDialog({
  isOpen,
  title,
  message,
  confirmLabel = 'Confirm',
  cancelLabel = 'Cancel',
  onConfirm,
  onCancel,
  isDestructive = false,
}: ConfirmDialogProps): JSX.Element | null {
  // Handle Escape key
  useEffect(() => {
    if (!isOpen) return;

    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onCancel();
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, [isOpen, onCancel]);

  if (!isOpen) return null;

  return (
    <div className={styles.confirmOverlay} onClick={onCancel}>
      <div
        className={styles.confirmDialog}
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-labelledby="confirm-dialog-title"
      >
        <div className={styles.confirmHeader}>
          <h3 id="confirm-dialog-title" className={styles.confirmTitle}>
            {title}
          </h3>
        </div>

        <div className={styles.confirmBody}>
          <p className={styles.confirmMessage}>{message}</p>
        </div>

        <div className={styles.confirmActions}>
          <button
            className={styles.confirmCancelButton}
            onClick={onCancel}
            autoFocus
          >
            {cancelLabel}
          </button>
          <button
            className={`${styles.confirmButton} ${
              isDestructive ? styles.confirmButtonDestructive : ''
            }`}
            onClick={onConfirm}
          >
            {confirmLabel}
          </button>
        </div>
      </div>
    </div>
  );
}
