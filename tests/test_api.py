# Test script to verify the RAG chatbot API is working
import requests
import json
import time

def test_api():
    base_url = "http://localhost:8000"

    # Test health endpoint first
    try:
        health_response = requests.get(f"{base_url}/health")
        print(f"Health check: {health_response.status_code} - {health_response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")
        return

    # Test chat endpoint
    try:
        chat_payload = {
            "query": "What is Physical AI?",
            "session_id": "test_session_123"
        }

        chat_response = requests.post(
            f"{base_url}/api/chat",
            json=chat_payload,
            headers={"Content-Type": "application/json"}
        )

        print(f"Chat endpoint: {chat_response.status_code}")
        if chat_response.status_code == 200:
            response_data = chat_response.json()
            print(f"Response: {response_data.get('response', '')[:100]}...")
            print(f"Sources: {len(response_data.get('sources', []))} sources found")
        else:
            print(f"Error: {chat_response.text}")
    except Exception as e:
        print(f"Chat endpoint test failed: {e}")

    # Test with selected text (if endpoint exists)
    try:
        selected_text_payload = {
            "query": "Explain this concept",
            "selected_text": "Physical AI is a field that bridges digital AI with physical bodies",
            "session_id": "test_session_123"
        }

        selected_response = requests.post(
            f"{base_url}/api/chat/selected",
            json=selected_text_payload,
            headers={"Content-Type": "application/json"}
        )

        print(f"Selected text endpoint: {selected_response.status_code}")
        if selected_response.status_code == 200:
            response_data = selected_response.json()
            print(f"Response: {response_data.get('response', '')[:100]}...")
        else:
            print(f"Error: {selected_response.text}")
    except Exception as e:
        print(f"Selected text endpoint test failed (this is expected if not implemented yet): {e}")

if __name__ == "__main__":
    print("Testing RAG Chatbot API...")
    test_api()