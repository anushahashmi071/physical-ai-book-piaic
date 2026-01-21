import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Embodied Intelligence',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn how AI systems operate in the physical world through vision, language, and action integration.
        Understand the principles of embodied cognition that connect neural networks to robotic behavior.
      </>
    ),
  },
  {
    title: 'Simulation-First Approach',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master robotics development through simulation environments before hardware deployment.
        Test algorithms safely in Gazebo and Unity with perfect ground truth and reproducible scenarios.
      </>
    ),
  },
  {
    title: 'ROS 2 Integration',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Build on the Robot Operating System 2 framework for distributed robotics applications.
        Connect perception, planning, and control systems through standardized communication patterns.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action Pipeline',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Understand how natural language commands flow through perception and reasoning to physical action.
        Create humanoid robots that respond to human instructions with intelligent behavior.
      </>
    ),
  },
  {
    title: 'Cognitive Task Planning',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Learn how Large Language Models decompose complex goals into executable robotic tasks.
        Combine symbolic planning with neural reasoning for robust task execution.
      </>
    ),
  },
  {
    title: 'Safe & Reliable Execution',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Implement safety constraints and validation mechanisms for trustworthy robotic systems.
        Design feedback loops and error recovery for resilient autonomous operation.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
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
