/**
 * Docusaurus Plugin: RAG Chatbot
 *
 * Provides a floating chatbot UI for asking questions about book content.
 * This plugin is a pure UI layer that consumes the backend RAG API.
 *
 * @param {Object} context - Docusaurus context
 * @param {Object} options - Plugin options
 * @param {string} options.apiUrl - Backend API URL (default: http://localhost:8000)
 * @returns {Object} Plugin configuration
 */
module.exports = function pluginChatbot(context, options) {
  const { apiUrl = 'http://localhost:8000' } = options;

  return {
    name: 'docusaurus-plugin-chatbot',

    getClientModules() {
      return [require.resolve('./chatbot-client.js')];
    },

    contentLoaded({ actions }) {
      const { setGlobalData } = actions;
      setGlobalData({
        apiUrl,
      });
    },
  };
};
