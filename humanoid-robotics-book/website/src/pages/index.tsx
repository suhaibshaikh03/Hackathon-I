import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="text--center padding-bottom--lg">
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <p className="hero__author">By Muhammad Suhaib Shaikh</p>
        </div>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg button--outline"
            to="/docs/intro">
            Begin Your Journey →
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageDescription() {
  return (
    <section className={styles.features}>
      <div className="container padding-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2 text--center">
            <Heading as="h2" className="padding-bottom--md">
              Embark on the Future of Robotics
            </Heading>
            <p className="padding-bottom--md">
              Dive deep into the revolutionary world of Physical AI and Humanoid Robotics.
              This comprehensive textbook guides you through cutting-edge technologies,
              from ROS 2 fundamentals to advanced NVIDIA Isaac platforms, Vision-Language-Action systems,
              and autonomous humanoid development.
            </p>
            <div className="padding-vert--lg">
              <div className="avatar avatar--vertical">
                <h3>🌟 What Awaits You:</h3>
              </div>
              <div className="row padding-vert--lg">
                <div className="col col--4">
                  <div className="avatar">
                    <div className="avatar__intro">
                      <h4 className="avatar__name">Advanced Simulation</h4>
                      <p className="avatar__subtitle">Gazebo, Isaac Sim & Unity</p>
                    </div>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="avatar">
                    <div className="avatar__intro">
                      <h4 className="avatar__name">AI Integration</h4>
                      <p className="avatar__subtitle">Isaac Lab, GR00T & VLA</p>
                    </div>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="avatar">
                    <div className="avatar__intro">
                      <h4 className="avatar__name">Autonomous Systems</h4>
                      <p className="avatar__subtitle">Complete Capstone Project</p>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CallToAction() {
  return (
    <section className={clsx('hero hero--dark', styles.heroBanner)}>
      <div className="container text--center padding-vert--xl">
        <Heading as="h2" className="padding-bottom--md">
          Ready to Transform the Future?
        </Heading>
        <p className="padding-bottom--lg">
          Start your journey into the most exciting field in technology today.
          Master the skills needed to build the next generation of intelligent,
          autonomous humanoid robots.
        </p>
        <Link
          className="button button--primary button--lg"
          to="/docs/intro">
          Start Learning Now
        </Link>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A Comprehensive Textbook on Embodied Intelligence by Muhammad Suhaib Shaikh">
      <HomepageHeader />
      <main>
        <HomepageDescription />
        <CallToAction />
      </main>
    </Layout>
  );
}
