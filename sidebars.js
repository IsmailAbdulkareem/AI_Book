/**
 * Creating a sidebar enables you to:
 - Create an ordered group of docs
 - Render a sidebar for each doc of that group
 - Manually specify the order of documents in the sidebar
 - List unindexed docs as a subset in the sidebar
 - Learn more at https://docusaurus.io/docs/sidebar#sidebar-create-category
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction & Foundations',
      items: [
        'introduction/introduction',
        'introduction/intro-environment-setup',
        'glossary',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/module1-ros2',
        'module1/module1-chapter1',
        'module1/module1-chapter2',
        'module1/module1-chapter3',
        'module1/module1-chapter4',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/module2-digital-twin',
        'module2/module2-chapter1',
        'module2/module2-chapter2',
        'module2/module2-chapter3',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/module3-isaac',
        'module3/module3-chapter1',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/module4-vla',
        'module4/module4-chapter1',
      ],
    },
    'hardware-lab-architecture/hardware-lab-architecture',
    'capstone_workflow/capstone-autonomous-humanoid',
    {
      type: 'category',
      label: 'Appendices & Resources',
      items: [
        'appendix/appendix-resources',
        'appendix/appendix-assessments',
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

module.exports = sidebars;
