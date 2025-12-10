import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/AI_Book/__docusaurus/debug',
    component: ComponentCreator('/AI_Book/__docusaurus/debug', '869'),
    exact: true
  },
  {
    path: '/AI_Book/__docusaurus/debug/config',
    component: ComponentCreator('/AI_Book/__docusaurus/debug/config', '5c6'),
    exact: true
  },
  {
    path: '/AI_Book/__docusaurus/debug/content',
    component: ComponentCreator('/AI_Book/__docusaurus/debug/content', '149'),
    exact: true
  },
  {
    path: '/AI_Book/__docusaurus/debug/globalData',
    component: ComponentCreator('/AI_Book/__docusaurus/debug/globalData', '56e'),
    exact: true
  },
  {
    path: '/AI_Book/__docusaurus/debug/metadata',
    component: ComponentCreator('/AI_Book/__docusaurus/debug/metadata', '32d'),
    exact: true
  },
  {
    path: '/AI_Book/__docusaurus/debug/registry',
    component: ComponentCreator('/AI_Book/__docusaurus/debug/registry', 'c4f'),
    exact: true
  },
  {
    path: '/AI_Book/__docusaurus/debug/routes',
    component: ComponentCreator('/AI_Book/__docusaurus/debug/routes', '8af'),
    exact: true
  },
  {
    path: '/AI_Book/docs',
    component: ComponentCreator('/AI_Book/docs', 'c2a'),
    routes: [
      {
        path: '/AI_Book/docs',
        component: ComponentCreator('/AI_Book/docs', 'cd0'),
        routes: [
          {
            path: '/AI_Book/docs',
            component: ComponentCreator('/AI_Book/docs', '505'),
            routes: [
              {
                path: '/AI_Book/docs/appendix/appendix-assessments',
                component: ComponentCreator('/AI_Book/docs/appendix/appendix-assessments', '352'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/appendix/appendix-resources',
                component: ComponentCreator('/AI_Book/docs/appendix/appendix-resources', 'aaa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/capstone_workflow/',
                component: ComponentCreator('/AI_Book/docs/capstone_workflow/', '892'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/glossary',
                component: ComponentCreator('/AI_Book/docs/glossary', '3cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/hardware-lab-architecture/',
                component: ComponentCreator('/AI_Book/docs/hardware-lab-architecture/', '63a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/intro',
                component: ComponentCreator('/AI_Book/docs/intro', '2a4'),
                exact: true
              },
              {
                path: '/AI_Book/docs/introduction/',
                component: ComponentCreator('/AI_Book/docs/introduction/', '10d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/introduction/intro-environment-setup',
                component: ComponentCreator('/AI_Book/docs/introduction/intro-environment-setup', '393'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module1/',
                component: ComponentCreator('/AI_Book/docs/module1/', '645'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module1/module1-chapter1',
                component: ComponentCreator('/AI_Book/docs/module1/module1-chapter1', '377'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module1/module1-chapter2',
                component: ComponentCreator('/AI_Book/docs/module1/module1-chapter2', '2bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module1/module1-chapter3',
                component: ComponentCreator('/AI_Book/docs/module1/module1-chapter3', '02e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module1/module1-chapter4',
                component: ComponentCreator('/AI_Book/docs/module1/module1-chapter4', 'ce1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module2/',
                component: ComponentCreator('/AI_Book/docs/module2/', 'c81'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module2/module2-chapter1',
                component: ComponentCreator('/AI_Book/docs/module2/module2-chapter1', '639'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module2/module2-chapter2',
                component: ComponentCreator('/AI_Book/docs/module2/module2-chapter2', '323'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module2/module2-chapter3',
                component: ComponentCreator('/AI_Book/docs/module2/module2-chapter3', 'ecf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module3/',
                component: ComponentCreator('/AI_Book/docs/module3/', 'fe9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module3/module3-chapter1',
                component: ComponentCreator('/AI_Book/docs/module3/module3-chapter1', 'ada'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module4/',
                component: ComponentCreator('/AI_Book/docs/module4/', '6d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module4/module4-chapter1',
                component: ComponentCreator('/AI_Book/docs/module4/module4-chapter1', '614'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI_Book/docs/module4/module4-chapter2',
                component: ComponentCreator('/AI_Book/docs/module4/module4-chapter2', 'c2d'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/AI_Book/',
    component: ComponentCreator('/AI_Book/', '1bd'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
