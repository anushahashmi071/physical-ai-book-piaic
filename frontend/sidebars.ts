import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2: The Robotic Nervous System',
      collapsed: false,
      items: [
        'ros2-nervous-system/robotic-nervous-system',
        'ros2-nervous-system/ros2-overview',
        'ros2-nervous-system/nodes-functional-units',
        'ros2-nervous-system/communication-patterns',
        'ros2-nervous-system/python-ai-agents-rclpy',
        'ros2-nervous-system/robot-structure-urdf',
        'ros2-nervous-system/system-integration',
        'ros2-nervous-system/assessments/chapter-assessments',
        'ros2-nervous-system/assessments/module-assessment',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin: Gazebo & Unity',
      collapsed: false,
      items: [
        'digital-twin-gazebo-unity/chapter-01-digital-twin-fundamentals',
        'digital-twin-gazebo-unity/chapter-02-physics-simulation-basics',
        'digital-twin-gazebo-unity/chapter-03-gazebo-robotics-simulation',
        'digital-twin-gazebo-unity/chapter-04-sensor-simulation',
        'digital-twin-gazebo-unity/chapter-05-unity-visualization',
        'digital-twin-gazebo-unity/chapter-06-ros2-integration',
        'digital-twin-gazebo-unity/chapter-07-simulation-first-workflow',
        'digital-twin-gazebo-unity/assessments',
      ],
    },
    {
      type: 'category',
      label: 'Perception & Sensor Intelligence',
      collapsed: false,
      items: [
        'perception-sensor-intelligence/chapter-01-perception-vs-sensing',
        'perception-sensor-intelligence/chapter-02-sensor-data-pipelines',
        'perception-sensor-intelligence/chapter-03-camera-perception-basics',
        'perception-sensor-intelligence/chapter-04-lidar-depth-perception',
        'perception-sensor-intelligence/chapter-05-sensor-fusion-fundamentals',
        'perception-sensor-intelligence/chapter-06-perception-ros2-integration',
        'perception-sensor-intelligence/chapter-07-simulation-testing-perception',
        'perception-sensor-intelligence/assessments',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) Integration',
      collapsed: false,
      items: [
        'vla-integration/chapter-01-from-language-to-action',
        'vla-integration/chapter-02-voice-to-command-pipeline',
        'vla-integration/chapter-03-cognitive-task-planning',
        'vla-integration/chapter-04-vision-grounding',
        'vla-integration/chapter-05-safe-action-execution',
        'vla-integration/chapter-06-capstone-autonomous-humanoid',
        'vla-integration/assessments',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
