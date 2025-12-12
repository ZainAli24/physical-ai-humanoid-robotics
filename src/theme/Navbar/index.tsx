/**
 * Swizzled Docusaurus Navbar with authentication
 *
 * This wraps the original Navbar and adds UserMenu + AuthModal
 */

import React, { useState, useEffect } from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import UserMenu from '../../components/UserMenu';
import AuthModal from '../../components/AuthModal';
import { useAuth } from '../../hooks/useAuth';
import type { WrapperProps } from '@docusaurus/types';
import type NavbarType from '@theme/Navbar';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): JSX.Element {
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const { isAuthenticated, checkSession } = useAuth();

  // Check session on mount
  useEffect(() => {
    checkSession();
  }, [checkSession]);

  return (
    <>
      <OriginalNavbar {...props} />

      {/* Auth button or User menu - injected into navbar */}
      <div
        className="navbar-auth-container"
        style={{
          position: 'fixed',
          top: 'calc(var(--ifm-navbar-height, 60px) / 2)',
          right: '9rem', // Space for theme toggle, GitHub icon, and padding
          transform: 'translateY(-50%)',
          zIndex: 1000,
          display: 'flex',
          alignItems: 'center',
          gap: '0.5rem',
        }}
      >
        {isAuthenticated ? (
          <UserMenu />
        ) : (
          <button
            onClick={() => setIsAuthModalOpen(true)}
            className="navbar-auth-button"
            style={{
              padding: '0.5rem 1rem',
              backgroundColor: '#3578e5',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              fontSize: '0.875rem',
              fontWeight: '600',
              cursor: 'pointer',
              transition: 'all 0.2s ease',
              boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
              whiteSpace: 'nowrap',
            }}
            onMouseOver={(e) => {
              e.currentTarget.style.backgroundColor = '#2b68c4';
              e.currentTarget.style.transform = 'translateY(-1px)';
              e.currentTarget.style.boxShadow = '0 4px 8px rgba(0,0,0,0.15)';
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.backgroundColor = '#3578e5';
              e.currentTarget.style.transform = 'translateY(0)';
              e.currentTarget.style.boxShadow = '0 2px 4px rgba(0,0,0,0.1)';
            }}
          >
            Sign In
          </button>
        )}
      </div>

      {/* Responsive styles */}
      <style>{`
        /* Adjust navbar items spacing to accommodate auth button */
        .navbar__items--right {
          gap: 0.5rem;
        }

        /* Tablet and small desktop */
        @media (max-width: 996px) {
          .navbar-auth-container {
            right: 6rem !important;
          }
          .navbar-auth-button {
            padding: 0.4rem 0.8rem !important;
            font-size: 0.8rem !important;
          }
        }

        /* iPad and mobile landscape */
        @media (max-width: 768px) {
          .navbar-auth-container {
            right: 4.5rem !important;
          }
          .navbar-auth-button {
            padding: 0.35rem 0.65rem !important;
            font-size: 0.75rem !important;
          }
        }

        /* Mobile portrait */
        @media (max-width: 576px) {
          .navbar-auth-container {
            right: 3.5rem !important;
          }
          .navbar-auth-button {
            padding: 0.3rem 0.5rem !important;
            font-size: 0.7rem !important;
          }
        }

        /* Very small screens */
        @media (max-width: 400px) {
          .navbar-auth-container {
            right: 3rem !important;
          }
          .navbar-auth-button {
            padding: 0.25rem 0.4rem !important;
            font-size: 0.65rem !important;
          }
        }
      `}</style>

      {/* Auth modal */}
      <AuthModal
        isOpen={isAuthModalOpen}
        onClose={() => setIsAuthModalOpen(false)}
        initialMode="signin"
      />
    </>
  );
}
