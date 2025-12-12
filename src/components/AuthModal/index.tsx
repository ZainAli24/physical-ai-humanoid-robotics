/**
 * AuthModal - Authentication modal with signup/signin forms
 *
 * Features:
 * - Toggle between signup and signin modes
 * - Email/password validation
 * - Error handling and display
 * - Loading states
 * - Responsive mobile-first design
 */

import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './AuthModal.module.css';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  initialMode?: 'signin' | 'signup';
}

export default function AuthModal({ isOpen, onClose, initialMode = 'signin' }: AuthModalProps) {
  const [mode, setMode] = useState<'signin' | 'signup'>(initialMode);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');

  const { signIn, signUp, isLoading, error, clearError } = useAuth();

  // Reset form when modal closes
  const handleClose = () => {
    setEmail('');
    setPassword('');
    setName('');
    clearError();
    onClose();
  };

  // Handle form submission
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    clearError();

    try {
      if (mode === 'signup') {
        await signUp(email, password, name || undefined);
      } else {
        await signIn(email, password);
      }

      // Success - close modal
      handleClose();
    } catch (error) {
      // Error already set in store
      console.error('Auth error:', error);
    }
  };

  // Toggle between signin and signup
  const toggleMode = () => {
    setMode(mode === 'signin' ? 'signup' : 'signin');
    clearError();
  };

  if (!isOpen) return null;

  return (
    <div className={styles.overlay} onClick={handleClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        {/* Close button */}
        <button
          className={styles.closeButton}
          onClick={handleClose}
          aria-label="Close modal"
        >
          ×
        </button>

        {/* Title */}
        <h2 className={styles.title}>
          {mode === 'signin' ? 'Sign In' : 'Create Account'}
        </h2>

        {/* Error message */}
        {error && (
          <div className={styles.error} role="alert">
            {error}
          </div>
        )}

        {/* Form */}
        <form onSubmit={handleSubmit} className={styles.form}>
          {/* Name field (signup only) */}
          {mode === 'signup' && (
            <div className={styles.field}>
              <label htmlFor="name" className={styles.label}>
                Name (optional)
              </label>
              <input
                id="name"
                type="text"
                value={name}
                onChange={(e) => setName(e.target.value)}
                className={styles.input}
                placeholder="Your name"
                maxLength={255}
                disabled={isLoading}
              />
            </div>
          )}

          {/* Email field */}
          <div className={styles.field}>
            <label htmlFor="email" className={styles.label}>
              Email
            </label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              className={styles.input}
              placeholder="student@example.com"
              required
              disabled={isLoading}
            />
          </div>

          {/* Password field */}
          <div className={styles.field}>
            <label htmlFor="password" className={styles.label}>
              Password
            </label>
            <input
              id="password"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className={styles.input}
              placeholder={mode === 'signup' ? 'Minimum 8 characters' : 'Your password'}
              required
              minLength={8}
              maxLength={128}
              disabled={isLoading}
            />
          </div>

          {/* Submit button */}
          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading
              ? 'Please wait...'
              : mode === 'signin'
              ? 'Sign In'
              : 'Create Account'}
          </button>
        </form>

        {/* Toggle mode */}
        <div className={styles.toggle}>
          {mode === 'signin' ? (
            <>
              Don't have an account?{' '}
              <button
                type="button"
                onClick={toggleMode}
                className={styles.toggleButton}
                disabled={isLoading}
              >
                Sign up
              </button>
            </>
          ) : (
            <>
              Already have an account?{' '}
              <button
                type="button"
                onClick={toggleMode}
                className={styles.toggleButton}
                disabled={isLoading}
              >
                Sign in
              </button>
            </>
          )}
        </div>
      </div>
    </div>
  );
}
