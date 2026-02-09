# Hugging Face Spaces Deployment - Auth Server

FROM node:20-slim

WORKDIR /app

# Copy package files
COPY package*.json ./
COPY tsconfig.json ./

# Install dependencies
RUN npm ci

# Copy source code
COPY src ./src

# Build TypeScript
RUN npm run build

# Expose port (HF Spaces uses 7860)
EXPOSE 7860

# Set port for HF Spaces
ENV PORT=7860

# Start the server
CMD ["node", "dist/index.js"]
