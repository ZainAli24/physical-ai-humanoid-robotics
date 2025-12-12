/**
 * Swizzled DocPage Component - Wrapper to inject ChatbotWidget
 * Feature: 004-rag-chat
 *
 * This component wraps the original Docusaurus DocPage component
 * and adds the ChatbotWidget floating button + modal
 */

import React from 'react';
import DocPage from '@theme-original/DocPage';
import type DocPageType from '@theme/DocPage';
import type { WrapperProps } from '@docusaurus/types';
import ChatbotWidget from '../../components/ChatbotWidget';

type Props = WrapperProps<typeof DocPageType>;

/**
 * DocPage Wrapper
 *
 * Renders:
 * - Original DocPage component (all Docusaurus functionality)
 * - ChatbotWidget (floating button + modal)
 */
export default function DocPageWrapper(props: Props): JSX.Element {
  return (
    <>
      {/* Original DocPage */}
      <DocPage {...props} />

      {/* ChatbotWidget (floating button + modal) */}
      <ChatbotWidget />
    </>
  );
}
