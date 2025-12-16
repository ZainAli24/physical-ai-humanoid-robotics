/**
 * Zustand Store - Chat state management
 * Feature: 004-rag-chat (Updated for 005-text-selection, 006-auth-sessions)
 */

import { create } from 'zustand';
import { useAuth } from './useAuth'; // T045: Import auth hook

/**
 * Message - Represents a single chat message
 */
export interface Message {
  id: string; // UUID
  role: 'user' | 'assistant';
  content: string;
  sources: Source[];
  createdAt: string; // ISO 8601
}

/**
 * Source - Citation from textbook
 */
export interface Source {
  url: string;
  title: string;
  excerpt: string;
  score: number;
}

/**
 * Chat State
 */
interface ChatState {
  // UI State
  isOpen: boolean;
  isStreaming: boolean;

  // Messages
  messages: Message[];

  // Selected Text Context (Feature 005 - T020)
  selectedText: string | null;

  // Chat Session ID (Feature 006 - T046)
  sessionId: string | null;

  // Actions
  openChat: () => void;
  closeChat: () => void;
  sendMessage: (content: string) => Promise<void>;
  appendToken: (token: string) => void;
  setSources: (sources: Source[]) => void;
  clearChat: () => void;
  setSelectedText: (text: string | null) => void; // T020: Setter for selected text
  setSessionId: (id: string | null) => void; // T046: Setter for session ID
  startNewChat: () => void; // T047: Start new chat session
  loadMessagesFromSession: (sessionId: string) => Promise<void>; // T087: Load messages from session
}

/**
 * Backend API base URL
 * Note: Docusaurus doesn't support process.env in browser context
 */
const API_BASE_URL =
  typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000' // Local development
    : 'https://rag-chatbot-backend-wb8a.onrender.com'; // Production backend

/**
 * Generate UUID (simple version for client-side)
 */
function generateId(): string {
  return `${Date.now()}-${Math.random().toString(36).substring(2, 9)}`;
}

/**
 * Chat Store
 */
export const useChat = create<ChatState>((set, get) => ({
  // Initial state
  isOpen: false,
  isStreaming: false,
  messages: [],
  selectedText: null, // T020: Initial selected text state
  sessionId: null, // T046: Chat session ID for authenticated users

  // Open chat modal
  openChat: () => set({ isOpen: true }),

  // Close chat modal
  closeChat: () => set({ isOpen: false }),

  // Clear all messages
  clearChat: () => set({ messages: [] }),

  // T020: Set selected text
  setSelectedText: (text: string | null) => set({ selectedText: text }),

  // T046: Set session ID
  setSessionId: (id: string | null) => set({ sessionId: id }),

  // T047: Start new chat (clear messages and session ID)
  startNewChat: () => set({ messages: [], sessionId: null, selectedText: null }),

  // T087: Load messages from a specific session
  loadMessagesFromSession: async (sessionIdToLoad: string) => {
    try {
      const authState = useAuth.getState();
      const sessionToken = authState.sessionToken;

      if (!sessionToken) {
        throw new Error('Not authenticated');
      }

      // Fetch messages from backend
      const response = await fetch(`${API_BASE_URL}/api/sessions/${sessionIdToLoad}/messages`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${sessionToken}`,
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();

      // Transform backend messages to frontend format
      const loadedMessages: Message[] = (data.messages || []).map((msg: any) => ({
        id: msg.id,
        role: msg.role,
        content: msg.content,
        sources: msg.sources || [],
        createdAt: msg.created_at,
      }));

      // Update state
      set({
        messages: loadedMessages,
        sessionId: sessionIdToLoad,
        selectedText: null,
      });
    } catch (error) {
      console.error('Failed to load session messages:', error);
      throw error;
    }
  },

  // Send user message and stream assistant response
  sendMessage: async (content: string) => {
    const userMessage: Message = {
      id: generateId(),
      role: 'user',
      content,
      sources: [],
      createdAt: new Date().toISOString(),
    };

    // Add user message
    set((state) => ({
      messages: [...state.messages, userMessage],
      isStreaming: true,
    }));

    // Create empty assistant message for streaming
    const assistantMessageId = generateId();
    const assistantMessage: Message = {
      id: assistantMessageId,
      role: 'assistant',
      content: '',
      sources: [],
      createdAt: new Date().toISOString(),
    };

    set((state) => ({
      messages: [...state.messages, assistantMessage],
    }));

    try {
      // T045: Get session token from auth store
      const authState = useAuth.getState();
      const sessionToken = authState.sessionToken;

      // T045, T046: Include sessionId and selectedText in POST request body
      const requestBody = {
        message: content,
        session_id: get().sessionId, // T046: Use current session ID (null for new chats)
        stream: true,
        selected_text: get().selectedText || undefined, // Feature 005
      };

      // T045: Prepare headers with optional Authorization
      const headers: Record<string, string> = {
        'Content-Type': 'application/json',
      };

      if (sessionToken) {
        headers['Authorization'] = `Bearer ${sessionToken}`;
      }

      // Call chat API with Server-Sent Events
      const response = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        headers,
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      if (!response.body) {
        throw new Error('Response body is null');
      }

      // Read SSE stream
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();

        if (done) break;

        // Decode chunk
        buffer += decoder.decode(value, { stream: true });

        // Process complete SSE messages (split by double newline)
        const eventBlocks = buffer.split('\n\n');
        buffer = eventBlocks.pop() || ''; // Keep incomplete event in buffer

        for (const eventBlock of eventBlocks) {
          if (!eventBlock.trim()) continue;

          // Parse SSE event block (can have multiple lines: "event: ...\ndata: ...")
          const lines = eventBlock.split('\n');
          let eventType = '';
          let eventData = '';

          for (const line of lines) {
            if (line.startsWith('event: ')) {
              eventType = line.substring(7).trim();
            } else if (line.startsWith('data: ')) {
              eventData = line.substring(6).trim();
            }
          }

          // If we have data, parse it
          if (eventData) {
            try {
              const event = JSON.parse(eventData);

              if (event.event === 'content') {
                // Append content token to assistant message
                get().appendToken(event.data);
              } else if (event.event === 'sources') {
                // Set sources for assistant message
                get().setSources(event.data || []);
              } else if (event.event === 'done') {
                // T046: Update session ID from response
                if (event.data && event.data.session_id) {
                  set({ sessionId: event.data.session_id });
                }
                // Stop streaming
                set({ isStreaming: false });
                // T023: Clear selected text after successful message send
                set({ selectedText: null });
              } else if (event.event === 'error') {
                console.error('Chat error:', event.data);

                // Replace assistant message with error
                set((state) => ({
                  messages: state.messages.map((msg) =>
                    msg.id === assistantMessageId
                      ? { ...msg, content: `Error: ${event.data}` }
                      : msg
                  ),
                  isStreaming: false,
                }));
              }
            } catch (parseError) {
              console.error('Failed to parse SSE event:', parseError, 'Data:', eventData);
            }
          }
        }
      }
    } catch (error) {
      console.error('Send message error:', error);

      // Replace assistant message with error
      set((state) => ({
        messages: state.messages.map((msg) =>
          msg.id === assistantMessageId
            ? {
                ...msg,
                content: `Error: ${error instanceof Error ? error.message : 'Unknown error'}`,
              }
            : msg
        ),
        isStreaming: false,
      }));
    }
  },

  // Append token to the last assistant message
  appendToken: (token: string) => {
    set((state) => {
      const messages = [...state.messages];
      const lastMessage = messages[messages.length - 1];

      if (lastMessage && lastMessage.role === 'assistant') {
        lastMessage.content += token;
      }

      return { messages };
    });
  },

  // Set sources for the last assistant message
  setSources: (sources: Source[]) => {
    set((state) => {
      const messages = [...state.messages];
      const lastMessage = messages[messages.length - 1];

      if (lastMessage && lastMessage.role === 'assistant') {
        lastMessage.sources = sources;
      }

      return { messages };
    });
  },
}));
