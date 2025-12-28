import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

// Module data for the card grid
const ModuleList = [
  {
    title: 'Module 1: ROS 2 Foundations',
    description: 'Learn ROS 2 — the nervous system of modern robots. Build nodes, topics, services, actions, publishers, subscribers, QoS, and real robot workflows.',
    to: '/docs/module1',
    icon: '🤖',
  },
  {
    title: 'Module 2: Simulation & Digital Twins',
    description: 'Master simulation systems: Gazebo, Unity Robotics, Isaac Sim, and digital twin workflows for training and testing robots safely.',
    to: '/docs/module2',
    icon: '🎮',
  },
  {
    title: 'Module 3: Hardware Foundations',
    description: 'Motors, actuators, torque control, IMUs, sensors, microcontrollers, embedded systems — everything real humanoids need.',
    to: '/docs/module3',
    icon: '⚙️',
  },
  {
    title: 'Module 4: VLA — Vision, Language, Action',
    description: 'Learn the most advanced robotics architecture: perception models, LLM-driven command systems, action planners, and embodied AI agents.',
    to: '/docs/module4',
    icon: '👁️',
  },
  {
    title: 'Module 5: Advanced AI & Motion Control',
    description: 'Reinforcement learning, motion planning, MPC, trajectory optimization, and how robots think and move intelligently.',
    to: '/docs/module4/module4-chapter1',
    icon: '🧠',
  },
  {
    title: 'Module 6: Designing Humanoid Robots',
    description: 'Learn end-to-end humanoid creation: mechanical design, kinematics, actuators, morphologies, energy systems, and AI-driven movement.',
    to: '/docs/hardware-lab-architecture',
    icon: '🦾',
  },
];

// Feature highlights data
const FeatureHighlights = [
  {
    title: 'AI-Driven Design',
    description: 'Built fully around modern robotics workflows, from LLM agents to VLA systems and intelligent controllers.',
    icon: '🚀',
  },
  {
    title: 'Hands-On Learning',
    description: 'Every module includes practical steps, code examples, simulations, and real robot applications.',
    icon: '💻',
  },
  {
    title: 'Industry-Inspired Curriculum',
    description: 'The content reflects what Tesla Bots, Figure AI, Apptronik, and Sanctuary AI use in real humanoid robotics pipelines.',
    icon: '🏭',
  },
];

function HeroSection() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroContent}>
        <h1 className={styles.heroTitle}>
          Physical AI & Humanoid Robotics Textbook
        </h1>
        <p className={styles.heroSubtitle}>
          A complete and practical learning system where you master the future: humanoid robotics,
          ROS 2, large action models, simulation, VLA systems, hardware, and advanced AI for
          next-generation intelligent machines.
        </p>
        <div className={styles.heroButtons}>
          <Link
            className={clsx('button button--primary button--lg', styles.primaryButton)}
            to="/docs/introduction">
            Start Reading
          </Link>
          <Link
            className={clsx('button button--outline button--lg', styles.secondaryButton)}
            to="/docs/module1">
            Explore Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

function OverviewSection() {
  return (
    <section className={styles.overview}>
      <div className="container">
        <h2 className={styles.sectionTitle}>What This Textbook Covers</h2>
        <p className={styles.sectionDescription}>
          This is a complete AI-native engineering curriculum designed for physical AI, humanoid robots,
          embodied intelligence, ROS 2 programming, digital twin simulations, and Vision-Language-Action (VLA) systems.
          Each module builds your robotics superpowers step by step.
        </p>
      </div>
    </section>
  );
}

function ModuleCard({ title, description, to, icon }) {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleIcon}>{icon}</div>
      <div className={styles.moduleContent}>
        <h3 className={styles.moduleTitle}>{title}</h3>
        <p className={styles.moduleDescription}>{description}</p>
        <Link className={styles.moduleLink} to={to}>
          Open Module →
        </Link>
      </div>
    </div>
  );
}

function ModulesSection() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Explore All Modules</h2>
        <div className={styles.moduleGrid}>
          {ModuleList.map((module, idx) => (
            <ModuleCard key={idx} {...module} />
          ))}
        </div>
        {/* Appendix Card */}
        <div className={styles.appendixCard}>
          <div className={styles.appendixContent}>
            <h3 className={styles.appendixTitle}>📚 Appendix</h3>
            <p className={styles.appendixDescription}>
              Glossary, research papers, references, external resources, and further reading for mastering robotics and AI.
            </p>
            <Link className={styles.moduleLink} to="/docs/appendix/appendix-resources">
              Open Appendix →
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

function FeatureCard({ title, description, icon }) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Why This Textbook is AI-Native & Future-Focused</h2>
        <div className={styles.featureGrid}>
          {FeatureHighlights.map((feature, idx) => (
            <FeatureCard key={idx} {...feature} />
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className="container">
        <h2 className={styles.ctaTitle}>Begin Your Robotics Journey</h2>
        <p className={styles.ctaDescription}>
          The future belongs to physical AI, embodied intelligence, and humanoid robotics. Start mastering it today.
        </p>
        <Link
          className={clsx('button button--primary button--lg', styles.ctaButton)}
          to="/docs/introduction">
          Get Started Now
        </Link>
      </div>
    </section>
  );
}

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A complete and practical learning system for humanoid robotics, ROS 2, VLA systems, simulation, and advanced AI.">
      <HeroSection />
      <main>
        <OverviewSection />
        <ModulesSection />
        <FeaturesSection />
        <CTASection />
      </main>
    </Layout>
  );
}
