import React from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

const CustomFooter = () => {
  const location = useLocation();

  // Only show footer on documentation pages
  const showFooter = location.pathname.startsWith('/docs');

  return (
    <footer className={styles.footer}>
      <div className={styles.footerContent}>
        <div className={styles.licenseInfo}>
          <p>CC-BY-4.0 License</p>
          <p>Built with Spec-Kit Plus</p>
        </div>
        <div className={styles.editLink}>
          <a
            href={`https://github.com/humanoid-robotics-book/humanoid-robotics-book/edit/main/website${location.pathname}.md`}
            target="_blank"
            rel="noopener noreferrer"
          >
            Edit this page on GitHub
          </a>
        </div>
      </div>
    </footer>
  );
};

export default CustomFooter;