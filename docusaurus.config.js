/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital AI with Physical Bodies',
  url: 'https://IsmailAbdulkareem.github.io',
  baseUrl: '/AI_Book/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.svg',

  organizationName: 'IsmailAbdulkareem',   // FIXED
  projectName: 'AI_Book',                  // FIXED

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs',
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    [
      './src/plugins/docusaurus-plugin-chatbot',
      {
        apiUrl: 'https://ai-book-h6kj.onrender.com',
      },
    ],
  ],

  themeConfig: ({
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'introduction/introduction',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/IsmailAbdulkareem/AI_Book',
          label: 'GitHub',
          position: 'right',
        },
        {
          type: 'custom-userNavbarItem',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Book',
              to: '/docs/introduction',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'Stack Overflow', href: 'https://stackoverflow.com/questions/tagged/docusaurus' },
            { label: 'Discord', href: 'https://discordapp.com/invite/docusaurus' },
            { label: 'Twitter', href: 'https://twitter.com/docusaurus' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'GitHub', href: 'https://github.com/IsmailAbdulkareem/AI_Book' },
          ],
        },
      ],
      copyright:
        `Copyright © ${new Date().getFullYear()} Your Organization, Inc. Built with Docusaurus.`,
    },

    prism: {
      theme: require('prism-react-renderer/themes/github'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
    },
  }),
};

module.exports = config;
