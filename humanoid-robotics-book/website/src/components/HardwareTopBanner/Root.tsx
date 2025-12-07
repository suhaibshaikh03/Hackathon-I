import React from 'react';
import { useLocation } from '@docusaurus/router';
import HardwareTopBanner from './index';

const Root = ({ children }: { children: React.ReactNode }) => {
  const location = useLocation();

  // Show banner on all pages except potentially the homepage if desired
  // For now, showing on all pages as required by spec
  return (
    <>
      <HardwareTopBanner />
      {children}
    </>
  );
};

export default Root;