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
  // Manual sidebar structure according to book layout specification
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction & Why Physical AI Matters',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'doc',
      id: 'learning-outcomes',
    },
    {
      type: 'doc',
      id: 'hardware-setup',
    },
    {
      type: 'category',
      label: 'The Robotic Nervous System (ROS 2)',
      items: ['module-1/intro', 'module-1/core-concepts', 'module-1/hands-on-labs', 'module-1/debugging-tips', 'module-1/quiz'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'The Digital Twin (Gazebo & Unity)',
      items: ['module-2/intro', 'module-2/core-concepts', 'module-2/hands-on-labs', 'module-2/debugging-tips', 'module-2/quiz'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'The Digital Twin (Gazebo & Unity)',
      items: ['module-3/intro', 'module-3/core-concepts', 'module-3/hands-on-labs', 'module-3/debugging-tips', 'module-3/quiz'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'The AI-Robot Brain (NVIDIA Isaac Platform)',
      items: ['module-4/intro', 'module-4/core-concepts', 'module-4/hands-on-labs', 'module-4/debugging-tips', 'module-4/quiz'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) & Conversational Robotics',
      items: ['module-5/intro', 'module-5/core-concepts', 'module-5/hands-on-labs', 'module-5/debugging-tips', 'module-5/quiz'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Autonomous Humanoid Capstone Project',
      items: ['module-6/intro', 'module-6/core-concepts', 'module-6/hands-on-labs', 'module-6/debugging-tips', 'module-6/quiz'],
      collapsed: false,
    },
    {
      type: 'doc',
      id: 'weekly-breakdown',
    },
    {
      type: 'doc',
      id: 'assessments',
    },
    {
      type: 'category',
      label: 'Appendices',
      items: ['appendices/installation-guides', 'appendices/troubleshooting-guides', 'appendices/hardware-tables'],
      collapsed: true,
    },
    {
      type: 'doc',
      id: 'references',
    },
  ],
};

export default sidebars;
