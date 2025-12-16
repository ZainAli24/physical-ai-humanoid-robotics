/**
 * useChatHistory - Zustand store for chat history management
 * Feature: 006-auth-sessions - User Story 3
 *
 * Manages chat sessions list and loading messages from history
 */

import { create } from 'zustand';
import { useAuth } from './useAuth';

// API base URL from environment
// Note: Docusaurus doesn't support process.env in browser context
const API_URL =
  typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000/api' // Local development
    : 'https://rag-chatbot-backend-wb8a.onrender.com/api'; // Production backend

// Types
export interface ChatSession {
  id: string;
  title: string;
  created_at: string;
  updated_at: string;
  message_count: number;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources: any[] | null;
  agent_trace_id: string | null;
  created_at: string;
}

interface ChatHistoryState {
  // State
  sessions: ChatSession[];
  isLoadingSessions: boolean;
  isLoadingMessages: boolean;
  error: string | null;

  // Actions
  loadSessions: () => Promise<void>;
  loadMessages: (sessionId: string) => Promise<ChatMessage[]>;
  createSession: (title?: string) => Promise<string | null>;
  deleteSession: (sessionId: string) => Promise<boolean>;
  renameSession: (sessionId: string, newTitle: string) => Promise<boolean>;
  clearError: () => void;
}

export const useChatHistory = create<ChatHistoryState>((set, get) => ({
  // Initial state
  sessions: [],
  isLoadingSessions: false,
  isLoadingMessages: false,
  error: null,

  // Clear error
  clearError: () => set({ error: null }),

  // T055: Load user's chat sessions
  loadSessions: async () => {
    set({ isLoadingSessions: true, error: null });

    try {
      const authState = useAuth.getState();
      const sessionToken = authState.sessionToken;

      if (!sessionToken) {
        throw new Error('Not authenticated');
      }

      const response = await fetch(`${API_URL}/sessions?limit=50&offset=0`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${sessionToken}`,
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();

      set({
        sessions: data.sessions || [],
        isLoadingSessions: false,
        error: null,
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to load sessions';
      set({
        isLoadingSessions: false,
        error: message,
      });
      console.error('Load sessions error:', error);
    }
  },

  // T055: Load messages for a specific session
  loadMessages: async (sessionId: string): Promise<ChatMessage[]> => {
    set({ isLoadingMessages: true, error: null });

    try {
      const authState = useAuth.getState();
      const sessionToken = authState.sessionToken;

      if (!sessionToken) {
        throw new Error('Not authenticated');
      }

      const response = await fetch(`${API_URL}/sessions/${sessionId}/messages`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${sessionToken}`,
        },
      });

      if (!response.ok) {
        if (response.status === 404) {
          throw new Error('Session not found or access denied');
        }
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();

      set({
        isLoadingMessages: false,
        error: null,
      });

      return data.messages || [];
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to load messages';
      set({
        isLoadingMessages: false,
        error: message,
      });
      console.error('Load messages error:', error);
      return [];
    }
  },

  // T055: Create new session (placeholder - sessions are auto-created on first message)
  createSession: async (title: string = 'New Chat'): Promise<string | null> => {
    // In our architecture, sessions are created automatically on first message
    // This is a no-op that returns null to signal "use null session_id"
    return null;
  },

  // T055: Delete chat session
  deleteSession: async (sessionId: string): Promise<boolean> => {
    try {
      const authState = useAuth.getState();
      const sessionToken = authState.sessionToken;

      if (!sessionToken) {
        throw new Error('Not authenticated');
      }

      const response = await fetch(`${API_URL}/sessions/${sessionId}`, {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${sessionToken}`,
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      // Remove from local state
      set((state) => ({
        sessions: state.sessions.filter((s) => s.id !== sessionId),
      }));

      return true;
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to delete session';
      set({ error: message });
      console.error('Delete session error:', error);
      return false;
    }
  },

  // T055: Rename chat session
  renameSession: async (sessionId: string, newTitle: string): Promise<boolean> => {
    try {
      const authState = useAuth.getState();
      const sessionToken = authState.sessionToken;

      if (!sessionToken) {
        throw new Error('Not authenticated');
      }

      const response = await fetch(`${API_URL}/sessions/${sessionId}`, {
        method: 'PATCH',
        headers: {
          'Authorization': `Bearer ${sessionToken}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ title: newTitle }),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      // Update local state
      set((state) => ({
        sessions: state.sessions.map((s) =>
          s.id === sessionId ? { ...s, title: newTitle } : s
        ),
      }));

      return true;
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to rename session';
      set({ error: message });
      console.error('Rename session error:', error);
      return false;
    }
  },
}));
