/**
 * Custom Navbar Item Types
 *
 * Extends Docusaurus default navbar items with custom components.
 * This allows using 'custom-userNavbarItem' type in docusaurus.config.js
 */
import React from 'react';
import DefaultComponentTypes from '@theme-original/NavbarItem/ComponentTypes';

// Lazy load to avoid SSR issues
const UserNavbarItem = React.lazy(() =>
  import('../../plugins/docusaurus-plugin-chatbot/theme/NavbarItem/UserNavbarItem')
);

// Wrapper to handle SSR and Suspense
function UserNavbarItemWrapper(props) {
  const isBrowser = typeof window !== 'undefined';
  if (!isBrowser) return null;

  return (
    <React.Suspense fallback={<div style={{ width: 80 }} />}>
      <UserNavbarItem {...props} />
    </React.Suspense>
  );
}

export default {
  ...DefaultComponentTypes,
  'custom-userNavbarItem': UserNavbarItemWrapper,
};
