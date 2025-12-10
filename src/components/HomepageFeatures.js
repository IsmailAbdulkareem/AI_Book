import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Module 1 — The Robotic Nervous System (ROS 2)',
    Svg: require('../../static/img/undraw_robotics.svg').default,
    description: 'ROS 2 fundamentals, Python integration, URDF robot descriptions and hands-on examples.',
    to: '/docs/module1',
  },
  {
    title: 'Module 2 — The Digital Twin (Gazebo & Unity)',
    Svg: require('../../static/img/undraw_machine_learning.svg').default,
    description: 'Build and simulate digital twins with Gazebo and Unity; control and visualize robot behavior.',
    to: '/docs/module2',
  },
  {
    title: 'Module 3 — The AI‑Robot Brain (NVIDIA Isaac)',
    Svg: require('../../static/img/undraw_artificial_intelligence.svg').default,
    description: 'Introduction to Isaac Sim, AI model integration, and sim-to-real considerations for robotics.',
    to: '/docs/module3',
  },
  {
    title: 'Module 4 — Vision‑Language‑Action (VLA)',
    Svg: require('../../static/img/undraw_robotics.svg').default,
    description: 'Integrate vision and language models to enable perception-driven actions and behaviors.',
    to: '/docs/module4',
  },
];

function Feature({Svg, title, description, to}) {
  return (
    <div className={clsx('col col--6', styles.featureCard)}>
      <div className={styles.card}>
        <div className={styles.cardMedia}>
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className={styles.cardBody}>
          <h3>{title}</h3>
          <p>{description}</p>
          <Link className="button button--outline" to={to}>
            Open module
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
