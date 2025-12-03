import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  image: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Master ROS 2 Fundamentals',
    image: require('@site/static/mainimg/main_page_1.png').default,
    description: (
      <>
        Learn the building blocks of modern robotics: nodes, topics, services, and actions.
        Build a solid foundation with hands-on exercises and real-world examples using ROS 2 Humble.
      </>
    ),
  },
  {
    title: 'Simulate Before You Build',
    image: require('@site/static/mainimg/main_page_2.png').default,
    description: (
      <>
        Practice safely in Gazebo, Unity, and NVIDIA Isaac Sim. Test algorithms, train models,
        and validate behaviors in photorealistic environments before deploying to physical robots.
      </>
    ),
  },
  {
    title: 'Cutting-Edge AI for Robotics',
    image: require('@site/static/mainimg/main_page_3.png').default,
    description: (
      <>
        Explore Vision-Language-Action models like RT-1, RT-2, and PaLM-E. Train robots with
        multimodal intelligence that understand both language instructions and visual scenes.
      </>
    ),
  },
];

function Feature({title, image, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img src={image} className={styles.featureSvg} alt={title} />
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
