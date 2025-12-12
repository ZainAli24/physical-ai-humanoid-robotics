/**
 * useAuth - Zustand store for authentication state management
 *
 * Features:
 * - User signup, signin, signout
 * - Session persistence via localStorage
 * - Session verification on mount
 * - GDPR-compliant data deletion
 */

import { create } from 'zustand';
import { persist } from 'zustand/middleware';

// API base URL from environment
// Note: Docusaurus doesn't support process.env in browser context
const API_URL = typeof process !== 'undefined' && process.env?.REACT_APP_API_URL
  ? process.env.REACT_APP_API_URL
  : 'http://localhost:8000/api';

// User type
interface User {
  id: string;
  email: string;
  name: string | null;
  created_at: string;
}

// Auth state
interface AuthState {
  user: User | null;
  isAuthenticated: boolean;
  sessionToken: string | null;
  isLoading: boolean;
  error: string | null;

  // Actions
  signUp: (email: string, password: string, name?: string) => Promise<void>;
  signIn: (email: string, password: string) => Promise<void>;
  signOut: () => Promise<void>;
  checkSession: () => Promise<void>;
  deleteAllData: () => Promise<void>;
  clearError: () => void;
}

export const useAuth = create<AuthState>()(
  persist(
    (set, get) => ({
      // Initial state
      user: null,
      isAuthenticated: false,
      sessionToken: null,
      isLoading: false,
      error: null,

      // Clear error message
      clearError: () => set({ error: null }),

      // Sign up new user
      signUp: async (email: string, password: string, name?: string) => {
        set({ isLoading: true, error: null });

        try {
          const response = await fetch(`${API_URL}/auth/signup`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ email, password, name }),
          });

          if (!response.ok) {
            const error = await response.json();
            throw new Error(error.detail || error.error || 'Signup failed');
          }

          const { user, session_token } = await response.json();

          set({
            user,
            sessionToken: session_token,
            isAuthenticated: true,
            isLoading: false,
            error: null,
          });
        } catch (error) {
          const message = error instanceof Error ? error.message : 'Signup failed';
          set({ isLoading: false, error: message });
          throw error;
        }
      },

      // Sign in existing user
      signIn: async (email: string, password: string) => {
        set({ isLoading: true, error: null });

        try {
          const response = await fetch(`${API_URL}/auth/signin`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ email, password }),
          });

          if (!response.ok) {
            const error = await response.json();
            throw new Error(error.detail || error.error || 'Signin failed');
          }

          const { user, session_token } = await response.json();

          set({
            user,
            sessionToken: session_token,
            isAuthenticated: true,
            isLoading: false,
            error: null,
          });
        } catch (error) {
          const message = error instanceof Error ? error.message : 'Signin failed';
          set({ isLoading: false, error: message });
          throw error;
        }
      },

      // Sign out current user
      signOut: async () => {
        const { sessionToken } = get();

        if (sessionToken) {
          try {
            await fetch(`${API_URL}/auth/signout`, {
              method: 'POST',
              headers: { Authorization: `Bearer ${sessionToken}` },
            });
          } catch (error) {
            // Continue with signout even if API call fails
            console.error('Signout API error:', error);
          }
        }

        set({
          user: null,
          sessionToken: null,
          isAuthenticated: false,
          error: null,
        });
      },

      // Check if session is still valid
      checkSession: async () => {
        const { sessionToken } = get();

        if (!sessionToken) {
          set({ isAuthenticated: false, user: null });
          return;
        }

        try {
          const response = await fetch(`${API_URL}/auth/session`, {
            headers: { Authorization: `Bearer ${sessionToken}` },
          });

          if (response.ok) {
            const { user } = await response.json();
            set({ user, isAuthenticated: true });
          } else {
            // Session expired or invalid
            set({
              user: null,
              sessionToken: null,
              isAuthenticated: false,
            });
          }
        } catch (error) {
          console.error('Session check error:', error);
          set({
            user: null,
            sessionToken: null,
            isAuthenticated: false,
          });
        }
      },

      // Delete all user data (GDPR compliance)
      deleteAllData: async () => {
        const { sessionToken } = get();

        if (!sessionToken) {
          throw new Error('Not authenticated');
        }

        set({ isLoading: true, error: null });

        try {
          const response = await fetch(`${API_URL}/user/data`, {
            method: 'DELETE',
            headers: { Authorization: `Bearer ${sessionToken}` },
          });

          if (!response.ok) {
            const error = await response.json();
            throw new Error(error.detail || error.error || 'Failed to delete data');
          }

          // Sign out after successful deletion
          set({
            user: null,
            sessionToken: null,
            isAuthenticated: false,
            isLoading: false,
            error: null,
          });
        } catch (error) {
          const message = error instanceof Error ? error.message : 'Failed to delete data';
          set({ isLoading: false, error: message });
          throw error;
        }
      },
    }),
    {
      // Persist configuration
      name: 'auth-storage',
      // Only persist sessionToken (user will be fetched via checkSession)
      partialize: (state) => ({
        sessionToken: state.sessionToken,
      }),
    }
  )
);
