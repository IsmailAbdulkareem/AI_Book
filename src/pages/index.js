import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import HomepageFeatures from '../components/HomepageFeatures';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/introduction">
            Start Reading - 5min ⏱️
          </Link>
         
        </div>
        <div className={styles.quickLinks} style={{marginTop: '16px'}}>
          <Link to="/docs/module1" className="button button--sm">Module 1</Link>
          <Link to="/docs/module2" className="button button--sm" style={{marginLeft:8}}>Module 2</Link>
          <Link to="/docs/module3" className="button button--sm" style={{marginLeft:8}}>Module 3</Link>
          <Link to="/docs/module4" className="button button--sm" style={{marginLeft:8}}>Module 4</Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
