import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

type HardwareBannerProps = {
  title: string;
};

const HardwareBannerContent = [
  {
    title: 'Requires Ubuntu 22.04 + NVIDIA RTX GPU',
  },
];

function HardwareBanner({title}: HardwareBannerProps) {
  return (
    <div className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{title}</h1>
        <p className="hero__subtitle">Ensure your system meets the hardware requirements before proceeding</p>
      </div>
    </div>
  );
}

export default function HardwareBanners(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {HardwareBannerContent.map((props, idx) => (
            <HardwareBanner key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}