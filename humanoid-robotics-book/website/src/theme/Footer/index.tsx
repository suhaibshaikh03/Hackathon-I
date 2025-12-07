import React from 'react';
import OriginalFooter from '@theme-original/Footer';
import { useLocation } from '@docusaurus/router';

const FooterWrapper = (props) => {
  const location = useLocation();

  return (
    <>
      <OriginalFooter {...props} />
      <div style={{
        backgroundColor: '#f9fafb',
        borderTop: '1px solid #e5e7eb',
        padding: '0.5rem 0',
        marginTop: '1rem',
        fontSize: '0.75rem',
        textAlign: 'center'
      }}>
        <div style={{ margin: '0 auto', maxWidth: '1200px', padding: '0 1rem' }}>
          <span>CC-BY-4.0 License</span> |
          <span> Built with Spec-Kit Plus</span> |
          <a
            href={`https://github.com/humanoid-robotics-book/humanoid-robotics-book/edit/main/website/docs${location.pathname}.md`}
            target="_blank"
            rel="noopener noreferrer"
            style={{ marginLeft: '0.5rem', color: '#2563eb' }}
          >
            Edit this page
          </a>
        </div>
      </div>
    </>
  );
};

export default FooterWrapper;