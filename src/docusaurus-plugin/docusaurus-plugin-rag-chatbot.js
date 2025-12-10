// Docusaurus plugin to inject the RAG chatbot widget into pages
// This plugin adds the chat widget to all Docusaurus pages

module.exports = function pluginRagChatbot(context, options) {
  return {
    name: 'docusaurus-plugin-rag-chatbot',

    getClientModules() {
      return [require.resolve('./chat-widget-injector')];
    },

    injectHtmlTags() {
      return {
        postBodyTags: [
          // The chat widget will be injected here
          {
            tagName: 'div',
            attributes: {
              id: 'rag-chatbot-container'
            }
          }
        ]
      };
    },

    async contentLoaded({ actions }) {
      const { createData } = actions;

      // Create the chat widget configuration
      const config = {
        apiUrl: options.apiUrl || 'http://localhost:8000',
        enabled: options.enabled !== false, // enabled by default
        position: options.position || 'bottom-right',
        theme: options.theme || 'default'
      };

      await createData(
        'rag-chatbot-config.json',
        JSON.stringify(config)
      );
    }
  };
};