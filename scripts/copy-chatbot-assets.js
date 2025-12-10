// Build script to copy static assets for the RAG chatbot
// This ensures the CSS file is available in the static directory

const fs = require('fs-extra');
const path = require('path');

async function copyChatbotAssets() {
  try {
    // Source and destination paths
    const srcDir = path.join(__dirname, 'src/frontend');
    const destDir = path.join(__dirname, 'static/rag-chatbot');

    // Create destination directory if it doesn't exist
    await fs.ensureDir(destDir);

    // Copy the CSS file
    const cssSrc = path.join(srcDir, 'chat-widget.css');
    const cssDest = path.join(destDir, 'chat-widget.css');

    if (await fs.pathExists(cssSrc)) {
      await fs.copy(cssSrc, cssDest);
      console.log('✓ Copied chat-widget.css to static/rag-chatbot/');
    } else {
      console.warn('⚠ chat-widget.css not found at', cssSrc);
    }

    // Copy other assets if needed
    console.log('RAG chatbot assets copied successfully');
  } catch (error) {
    console.error('Error copying RAG chatbot assets:', error);
    process.exit(1);
  }
}

// Run the function if this file is executed directly
if (require.main === module) {
  copyChatbotAssets();
}

module.exports = { copyChatbotAssets };