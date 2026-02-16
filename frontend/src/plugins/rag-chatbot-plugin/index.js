const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'rag-chatbot-plugin',

    getClientModules() {
      return [path.resolve(__dirname, './src/ChatbotInjector')];
    },

    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@site': path.resolve(__dirname, '../../../'), // Adjusted path
          },
        },
      };
    },
  };
};