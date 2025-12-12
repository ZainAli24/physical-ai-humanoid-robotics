/**
 * MessageList - Displays chat messages with sources
 * Feature: 004-rag-chat
 */

import React, { useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import { useChat, type Message, type Source } from '../../hooks/useChat';
import styles from './styles.module.css';

/**
 * MessageBubble Component
 */
function MessageBubble({ message }: { message: Message }): JSX.Element {
  const isUser = message.role === 'user';

  return (
    <div
      className={
        isUser ? styles.userMessageContainer : styles.assistantMessageContainer
      }
    >
      <div className={isUser ? styles.userMessage : styles.assistantMessage}>
        {/* Message Content - Render markdown for assistant, plain text for user */}
        <div className={styles.messageContent}>
          {isUser ? (
            message.content
          ) : (
            <ReactMarkdown
              components={{
                // Make links open in same tab (internal navigation)
                a: ({ node, ...props }) => <a {...props} />,
                // Remove default paragraph margins for better bubble layout
                p: ({ node, ...props }) => <p style={{ margin: 0 }} {...props} />,
              }}
            >
              {message.content}
            </ReactMarkdown>
          )}
        </div>

        {/* Timestamp */}
        <div className={styles.messageTimestamp}>
          {new Date(message.createdAt).toLocaleTimeString([], {
            hour: '2-digit',
            minute: '2-digit',
          })}
        </div>
      </div>

      {/* Sources (only for assistant messages) */}
      {!isUser && message.sources && message.sources.length > 0 && (
        <div className={styles.sourcesContainer}>
          <div className={styles.sourcesTitle}>Sources:</div>
          {message.sources.map((source, index) => (
            <SourceCard key={index} source={source} index={index} />
          ))}
        </div>
      )}
    </div>
  );
}

/**
 * SourceCard Component
 */
function SourceCard({
  source,
  index,
}: {
  source: Source;
  index: number;
}): JSX.Element {
  return (
    <a
      href={source.url}
      className={styles.sourceCard}
      target="_blank"
      rel="noopener noreferrer"
    >
      <div className={styles.sourceHeader}>
        <span className={styles.sourceNumber}>{index + 1}</span>
        <span className={styles.sourceTitle}>{source.title}</span>
        <span className={styles.sourceScore}>
          {(source.score * 100).toFixed(0)}%
        </span>
      </div>
      <div className={styles.sourceExcerpt}>{source.excerpt}</div>
    </a>
  );
}

/**
 * EmptyState Component
 */
function EmptyState(): JSX.Element {
  return (
    <div className={styles.emptyState}>
      <svg
        xmlns="http://www.w3.org/2000/svg"
        fill="none"
        viewBox="0 0 24 24"
        strokeWidth={1.5}
        stroke="currentColor"
        className={styles.emptyIcon}
      >
        <path
          strokeLinecap="round"
          strokeLinejoin="round"
          d="M8.625 12a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0H8.25m4.125 0a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0H12m4.125 0a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0h-.375M21 12c0 4.556-4.03 8.25-9 8.25a9.764 9.764 0 01-2.555-.337A5.972 5.972 0 015.41 20.97a5.969 5.969 0 01-.474-.065 4.48 4.48 0 00.978-2.025c.09-.457-.133-.901-.467-1.226C3.93 16.178 3 14.189 3 12c0-4.556 4.03-8.25 9-8.25s9 3.694 9 8.25z"
        />
      </svg>

      <h3 className={styles.emptyTitle}>Ask a question</h3>
      <p className={styles.emptyDescription}>
        I can help you learn about ROS 2, control systems, kinematics, sensors,
        and other robotics topics covered in this textbook.
      </p>

      <div className={styles.exampleQuestions}>
        <div className={styles.exampleTitle}>Try asking:</div>
        <div className={styles.exampleQuestion}>"What is ROS 2?"</div>
        <div className={styles.exampleQuestion}>
          "How does inverse kinematics work?"
        </div>
        <div className={styles.exampleQuestion}>
          "Explain PID control"
        </div>
      </div>
    </div>
  );
}

/**
 * MessageList Component
 *
 * Displays:
 * - Empty state (when no messages)
 * - User/assistant message bubbles
 * - Source citations below assistant messages
 * - Auto-scroll to bottom on new messages
 */
export default function MessageList(): JSX.Element {
  const { messages } = useChat();
  const listRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (listRef.current) {
      listRef.current.scrollTop = listRef.current.scrollHeight;
    }
  }, [messages]);

  return (
    <div ref={listRef} className={styles.messageList}>
      {messages.length === 0 ? (
        <EmptyState />
      ) : (
        messages.map((message) => (
          <MessageBubble key={message.id} message={message} />
        ))
      )}
    </div>
  );
}
