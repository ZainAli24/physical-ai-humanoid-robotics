/**
 * Root component - Wraps the entire Docusaurus application
 * This is where we add global components like AuthModal and ChatbotWidget
 */

import React from 'react';
import AuthModal from '../components/AuthModal';
import ChatbotWidget from '../components/ChatbotWidget';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <AuthModal />
      <ChatbotWidget />
    </>
  );
}
