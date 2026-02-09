# Quick Start Guide - Physical AI RAG Chatbot

## 🚀 Your Deployed Services

### Backend API (RAG Chatbot)
**URL**: https://ismail233290-backened.hf.space

**Test it**:
```bash
curl https://ismail233290-backened.hf.space/health
```

**Ask a question**:
```bash
curl -X POST https://ismail233290-backened.hf.space/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?", "session_id": "test-123"}'
```

### Auth Server (Optional Login)
**URL**: https://ismail233290-auth-server.hf.space

**Test it**:
```bash
curl https://ismail233290-auth-server.hf.space/health
```

### Frontend (Local Development)
**URL**: http://localhost:3000/AI_Book/

**To start**:
```bash
cd frontend
npm start
```

---

## 🧪 Testing Your Chatbot

### 1. Open the Website
```
http://localhost:3000/AI_Book/
```

### 2. Find the Chatbot
- Look for the chat icon in the **bottom-right corner**
- Click to open the chat panel

### 3. Ask Questions
Try these:
- "What is Physical AI?"
- "What is ROS 2?"
- "How do I set up the development environment?"

### 4. Test Text Selection
- Select any text on the page
- Click the "Ask AI" button that appears
- Ask a question about the selected text

---

## 📊 What's Working

✅ **Backend API**: Fully functional RAG chatbot
✅ **Auth Server**: Optional login system
✅ **Frontend**: Chatbot widget on all pages
✅ **No Login Required**: Anyone can use the chatbot
✅ **Optional Login**: Users can sign up for personalized features

---

## 🔗 Important URLs

- **Backend API**: https://ismail233290-backened.hf.space
- **Auth Server**: https://ismail233290-auth-server.hf.space
- **API Docs**: https://ismail233290-backened.hf.space/docs
- **Frontend (Local)**: http://localhost:3000/AI_Book/

---

## 📁 Documentation

- `FINAL-DEPLOYMENT-SUCCESS.md` - Complete deployment summary
- `DEPLOYMENT-COMPLETE.md` - Backend deployment details
- `AUTH-DEPLOYMENT-COMPLETE-GUIDE.md` - Auth server guide
- `TESTING-COMPLETE.md` - Test results

---

## 🆘 Need Help?

- Check HF Space logs: https://huggingface.co/spaces/ismail233290/backened
- Check auth logs: https://huggingface.co/spaces/ismail233290/auth-server
- GitHub repo: https://github.com/IsmailAbdulkareem/AI_Book

---

**Status**: 🟢 All systems operational
**Last Updated**: 2026-02-10
