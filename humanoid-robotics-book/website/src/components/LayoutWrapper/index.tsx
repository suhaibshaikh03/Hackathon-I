import React from 'react';
import Layout from '@theme/Layout';
import HardwareTopBanner from '../HardwareTopBanner';

interface LayoutWrapperProps {
  children: React.ReactNode;
  [key: string]: any;
}

const LayoutWrapper: React.FC<LayoutWrapperProps> = ({ children, ...layoutProps }) => {
  return (
    <Layout {...layoutProps}>
      <HardwareTopBanner />
      {children}
    </Layout>
  );
};

export default LayoutWrapper;