import React from 'react';
import { useLocation } from '@docusaurus/router';
import HardwareTopBanner from '../components/HardwareTopBanner';

const Root = ({ children }: { children: React.ReactNode }) => {
  // Show banner on all pages as required by the specification
  return (
    <>
      <HardwareTopBanner />
      {children}
    </>
  );
};

export default Root;