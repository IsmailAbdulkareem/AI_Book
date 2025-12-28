/**
 * UserNavbarItem Component
 *
 * Custom navbar item that shows login button or user menu.
 * - When not authenticated: Shows "Sign In" button that opens AuthModal
 * - When authenticated: Shows user avatar with dropdown menu (Sign Out)
 */
import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../components/Auth';
import AuthModal from '../../components/Auth/AuthModal';
import { signOut } from '../../lib/auth-client';
import styles from './UserNavbarItem.module.css';

// Default avatar SVG for users without profile image
function DefaultAvatar() {
  return (
    <svg
      className={styles.avatarIcon}
      viewBox="0 0 24 24"
      fill="currentColor"
      xmlns="http://www.w3.org/2000/svg"
    >
      <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
    </svg>
  );
}

export default function UserNavbarItem() {
  const { isAuthenticated, user, isPending, isProfileComplete } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);
  const [isLoggingOut, setIsLoggingOut] = useState(false);
  const dropdownRef = useRef(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setShowDropdown(false);
      }
    }
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  // Handle sign out
  const handleSignOut = async () => {
    setIsLoggingOut(true);
    try {
      await signOut();
      setShowDropdown(false);
    } catch (err) {
      console.error('Sign out failed:', err);
    } finally {
      setIsLoggingOut(false);
    }
  };

  // Show loading skeleton while auth state is loading
  if (isPending) {
    return <div className={styles.skeleton} />;
  }

  // Not authenticated - show Sign In button
  if (!isAuthenticated) {
    return (
      <>
        <button
          className={styles.signInButton}
          onClick={() => setShowAuthModal(true)}
          type="button"
        >
          Sign In
        </button>
        <AuthModal
          isOpen={showAuthModal}
          onClose={() => setShowAuthModal(false)}
        />
      </>
    );
  }

  // Authenticated - show user avatar with dropdown
  const displayName = user?.name || user?.email?.split('@')[0] || 'User';
  const avatarUrl = user?.image;

  return (
    <div className={styles.userMenu} ref={dropdownRef}>
      <button
        className={styles.avatarButton}
        onClick={() => setShowDropdown(!showDropdown)}
        type="button"
        aria-expanded={showDropdown}
        aria-haspopup="true"
      >
        {avatarUrl ? (
          <img
            src={avatarUrl}
            alt={displayName}
            className={styles.avatarImage}
          />
        ) : (
          <DefaultAvatar />
        )}
        <span className={styles.userName}>{displayName}</span>
        <svg
          className={`${styles.chevron} ${showDropdown ? styles.chevronUp : ''}`}
          viewBox="0 0 24 24"
          fill="currentColor"
        >
          <path d="M7 10l5 5 5-5z" />
        </svg>
      </button>

      {showDropdown && (
        <div className={styles.dropdown}>
          <div className={styles.dropdownHeader}>
            <span className={styles.dropdownEmail}>{user?.email}</span>
            {!isProfileComplete && (
              <span className={styles.profileIncomplete}>Profile incomplete</span>
            )}
          </div>
          <div className={styles.dropdownDivider} />
          <button
            className={styles.dropdownItem}
            onClick={handleSignOut}
            disabled={isLoggingOut}
            type="button"
          >
            {isLoggingOut ? 'Signing out...' : 'Sign Out'}
          </button>
        </div>
      )}
    </div>
  );
}
