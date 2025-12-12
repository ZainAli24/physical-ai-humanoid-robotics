/**
 * UserMenu - User avatar dropdown menu
 *
 * Features:
 * - Avatar with first letter of name/email
 * - Dropdown menu with Sign Out and Delete Data
 * - Confirmation dialog for data deletion
 * - Click-outside to close
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './UserMenu.module.css';

export default function UserMenu() {
  const { user, signOut, deleteAllData, isAuthenticated } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [isOpen]);

  if (!isAuthenticated || !user) return null;

  // Get initials for avatar
  const getInitials = (): string => {
    if (user.name) {
      return user.name
        .split(' ')
        .map((word) => word[0])
        .join('')
        .toUpperCase()
        .slice(0, 2);
    }
    return user.email[0].toUpperCase();
  };

  // Handle sign out
  const handleSignOut = async () => {
    setIsOpen(false);
    await signOut();
  };

  // Handle delete all data
  const handleDeleteData = () => {
    setIsOpen(false);
    setShowDeleteConfirm(true);
  };

  const confirmDeleteData = async () => {
    try {
      await deleteAllData();
      setShowDeleteConfirm(false);
    } catch (error) {
      console.error('Delete data error:', error);
      alert('Failed to delete data. Please try again.');
    }
  };

  return (
    <>
      <div className={styles.container} ref={menuRef}>
        {/* Avatar button */}
        <button
          className={styles.avatar}
          onClick={() => setIsOpen(!isOpen)}
          aria-label="User menu"
          aria-expanded={isOpen}
        >
          {getInitials()}
        </button>

        {/* Dropdown menu */}
        {isOpen && (
          <div className={styles.dropdown}>
            {/* User info */}
            <div className={styles.userInfo}>
              <div className={styles.userName}>{user.name || 'User'}</div>
              <div className={styles.userEmail}>{user.email}</div>
            </div>

            <div className={styles.divider} />

            {/* Menu items */}
            <button className={styles.menuItem} onClick={handleSignOut}>
              <span className={styles.menuIcon}>→</span>
              Sign Out
            </button>

            <button
              className={`${styles.menuItem} ${styles.menuItemDanger}`}
              onClick={handleDeleteData}
            >
              <span className={styles.menuIcon}>🗑</span>
              Delete All Data
            </button>
          </div>
        )}
      </div>

      {/* Delete confirmation modal */}
      {showDeleteConfirm && (
        <div className={styles.confirmOverlay}>
          <div className={styles.confirmModal}>
            <h3 className={styles.confirmTitle}>Delete All Data?</h3>
            <p className={styles.confirmText}>
              This will permanently delete your account, chat history, and all data.
              This action cannot be undone.
            </p>
            <div className={styles.confirmButtons}>
              <button
                className={styles.confirmCancel}
                onClick={() => setShowDeleteConfirm(false)}
              >
                Cancel
              </button>
              <button
                className={styles.confirmDelete}
                onClick={confirmDeleteData}
              >
                Delete Everything
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
