import openai
from typing import List, Dict, Any, Optional
from src.config import settings
import json

class OpenAIClient:
    def __init__(self):
        # Initialize OpenAI client with API key from settings
        openai.api_key = settings.openai_api_key
        self.model = settings.openai_model

    def generate_response(self,
                         prompt: str,
                         context: Optional[str] = None,
                         max_tokens: int = 1000,
                         temperature: float = 0.7,
                         sources: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
        """
        Generate a response using OpenAI API based on the prompt and context
        """
        try:
            # Construct the message with context if provided
            messages = []

            # System message to set the behavior
            system_message = f"You are an AI assistant helping readers understand the Physical AI & Humanoid Robotics book. Only provide answers based on the book content provided in the context. Be concise, accurate, and cite sources when possible."

            messages.append({"role": "system", "content": system_message})

            # Add context if provided
            if context:
                messages.append({"role": "system", "content": f"Book context: {context}"})

            # Add the user's question
            messages.append({"role": "user", "content": prompt})

            # Make the API call
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=temperature,
                response_format={"type": "json_object"}  # Request JSON response for structured output
            )

            # Extract the response
            content = response.choices[0].message.content
            try:
                # Try to parse as JSON for structured response
                parsed_response = json.loads(content)
                return {
                    "response": parsed_response.get("response", content),
                    "sources": sources or [],
                    "confidence_score": parsed_response.get("confidence_score", 0.8),
                    "usage": {
                        "prompt_tokens": response.usage.prompt_tokens,
                        "completion_tokens": response.usage.completion_tokens,
                        "total_tokens": response.usage.total_tokens
                    }
                }
            except json.JSONDecodeError:
                # If not JSON, return as plain text
                return {
                    "response": content,
                    "sources": sources or [],
                    "confidence_score": 0.8,
                    "usage": {
                        "prompt_tokens": response.usage.prompt_tokens,
                        "completion_tokens": response.usage.completion_tokens,
                        "total_tokens": response.usage.total_tokens
                    }
                }

        except Exception as e:
            print(f"Error generating response from OpenAI: {e}")
            raise

    def generate_response_from_messages(self,
                                      messages: List[Dict[str, str]],
                                      max_tokens: int = 1000,
                                      temperature: float = 0.7) -> str:
        """
        Generate a response from a list of messages (for conversation history)
        """
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=temperature
            )

            return response.choices[0].message.content

        except Exception as e:
            print(f"Error generating response from messages: {e}")
            raise

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the current OpenAI model
        """
        try:
            model_info = openai.Model.retrieve(self.model)
            return {
                "id": model_info.id,
                "created": model_info.created,
                "owned_by": model_info.owned_by
            }
        except Exception as e:
            print(f"Error getting model info: {e}")
            return {
                "id": self.model,
                "error": str(e)
            }

    def validate_api_key(self) -> bool:
        """
        Validate the API key by making a simple request
        """
        try:
            openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": "test"}],
                max_tokens=5
            )
            return True
        except Exception:
            return False

# Global instance of OpenAIClient
openai_client = OpenAIClient()