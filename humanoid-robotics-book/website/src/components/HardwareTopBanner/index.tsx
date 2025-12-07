import React from 'react';
import styles from './styles.module.css';

const HardwareTopBanner = () => {
  return (
    <div className={styles.topBanner}>
      <div className={styles.bannerContent}>
        Requires Ubuntu 22.04 + NVIDIA RTX GPU
      </div>
    </div>
  );
};

export default HardwareTopBanner;