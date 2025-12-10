from typing import List, Dict, Any, Optional
from src.models.database import ChatMessage
from src.services.message_service import message_service
from sqlalchemy.orm import Session
from uuid import UUID

class PromptEngineeringService:
    def __init__(self):
        pass

    def create_rag_prompt(self, query: str, context: List[Dict[str, Any]],
                         conversation_history: Optional[List[ChatMessage]] = None,
                         selected_text: Optional[str] = None) -> str:
        """
        Create a RAG (Retrieval-Augmented Generation) prompt for OpenAI
        """
        prompt = "You are an AI assistant helping readers understand the Physical AI & Humanoid Robotics book.\n\n"

        # Add conversation history if available
        if conversation_history:
            prompt += "Previous conversation context:\n"
            for msg in conversation_history[-5:]:  # Use last 5 messages as context
                prompt += f"{msg.role.capitalize()}: {msg.content}\n"
            prompt += "\n"

        # Add selected text context if provided
        if selected_text:
            prompt += f"User has selected this specific text for context:\n{selected_text}\n\n"
            prompt += "Please answer the user's question based primarily on this selected text.\n\n"

        # Add retrieved context
        if context:
            prompt += "Relevant information from the book:\n"
            for i, ctx in enumerate(context):
                payload = ctx.get('payload', {})
                section_title = payload.get('title', 'Unknown Section')
                section_id = payload.get('section_id', 'Unknown')
                content_preview = payload.get('content', '')[:500]  # First 500 chars
                relevance_score = ctx.get('score', 0.0)

                prompt += f"Section {i+1} - {section_title} ({section_id}):\n"
                prompt += f"Content: {content_preview}\n"
                prompt += f"Relevance: {relevance_score:.2f}\n\n"
        else:
            prompt += "No relevant information was found in the book for this query.\n\n"

        # Add the user's query
        prompt += f"User's question: {query}\n\n"

        # Add instructions for the response
        prompt += (
            "Please provide an accurate answer based on the book content provided above. "
            "If the information is not available in the provided context, state that clearly. "
            "Cite specific sections when possible and maintain a helpful, educational tone. "
            "Keep your response concise but comprehensive."
        )

        return prompt

    def create_context_summary_prompt(self, context: List[Dict[str, Any]]) -> str:
        """
        Create a prompt to summarize the retrieved context
        """
        prompt = "Summarize the following information from the Physical AI & Humanoid Robotics book:\n\n"

        for i, ctx in enumerate(context):
            payload = ctx.get('payload', {})
            section_title = payload.get('title', 'Unknown Section')
            content_preview = payload.get('content', '')[:300]  # First 300 chars
            relevance_score = ctx.get('score', 0.0)

            prompt += f"Section {i+1} - {section_title}:\n"
            prompt += f"Content: {content_preview}\n"
            prompt += f"Relevance: {relevance_score:.2f}\n\n"

        prompt += (
            "Provide a concise summary that captures the key points relevant to the user's query. "
            "Focus on the most relevant information and maintain accuracy to the original content."
        )

        return prompt

    def create_citation_prompt(self, response: str, context: List[Dict[str, Any]]) -> str:
        """
        Create a prompt to generate proper citations from the response and context
        """
        prompt = f"Original response: {response}\n\n"
        prompt += "Book sections referenced:\n"

        for i, ctx in enumerate(context):
            payload = ctx.get('payload', {})
            section_id = payload.get('section_id', 'Unknown')
            title = payload.get('title', 'Unknown Section')
            page_url = payload.get('page_url', 'Unknown URL')
            relevance_score = ctx.get('score', 0.0)

            prompt += f"{i+1}. Section ID: {section_id}\n"
            prompt += f"   Title: {title}\n"
            prompt += f"   URL: {page_url}\n"
            prompt += f"   Relevance Score: {relevance_score:.2f}\n\n"

        prompt += (
            "Based on the original response and the referenced sections, "
            "provide specific citations that indicate which parts of the response "
            "come from which book sections. Format citations as: "
            "[Section ID: Title] or [URL] when appropriate."
        )

        return prompt

    def create_followup_prompt(self, query: str, response: str, context: List[Dict[str, Any]]) -> str:
        """
        Create a prompt for potential follow-up questions
        """
        prompt = "Based on the following interaction, suggest 2-3 relevant follow-up questions:\n\n"
        prompt += f"Original question: {query}\n"
        prompt += f"Answer provided: {response}\n\n"

        if context:
            prompt += "Relevant book sections:\n"
            for ctx in context:
                payload = ctx.get('payload', {})
                title = payload.get('title', 'Unknown Section')
                prompt += f"- {title}\n"

        prompt += (
            "Suggest follow-up questions that would help the reader better understand "
            "the topic or explore related concepts in the book."
        )

        return prompt

    def create_validation_prompt(self, response: str, query: str, context: List[Dict[str, Any]]) -> str:
        """
        Create a prompt to validate that the response is relevant and accurate
        """
        prompt = f"Question asked: {query}\n"
        prompt += f"Response given: {response}\n\n"

        if context:
            prompt += "Context provided to generate the response:\n"
            for ctx in context:
                payload = ctx.get('payload', {})
                title = payload.get('title', 'Unknown Section')
                content_preview = payload.get('content', '')[:200]
                prompt += f"- {title}: {content_preview}...\n\n"
        else:
            prompt += "No context was provided to generate the response.\n\n"

        prompt += (
            "Evaluate whether the response is: 1) Accurate according to the provided context, "
            "2) Directly addresses the question asked, 3) Appropriately mentions when information "
            "is not available in the provided context. Respond with 'VALID' if the response "
            "meets these criteria, otherwise respond with 'INVALID' followed by specific issues."
        )

        return prompt

    def create_condensed_prompt(self, query: str, context: List[Dict[str, Any]], max_length: int = 3000) -> str:
        """
        Create a condensed prompt that fits within token limits while preserving essential information
        """
        # Start with the base structure
        prompt = "You are an AI assistant helping readers understand the Physical AI & Humanoid Robotics book.\n\n"

        # Add context, but limit the total length
        if context:
            prompt += "Relevant information from the book:\n"
            total_length = len(prompt)

            for i, ctx in enumerate(context):
                if total_length >= max_length * 0.8:  # Use 80% for context to leave room for other parts
                    break

                payload = ctx.get('payload', {})
                section_title = payload.get('title', 'Unknown Section')
                content_preview = payload.get('content', '')[:500]  # First 500 chars
                relevance_score = ctx.get('score', 0.0)

                section_text = f"Section {i+1} - {section_title}:\nContent: {content_preview}\nRelevance: {relevance_score:.2f}\n\n"

                if total_length + len(section_text) > max_length * 0.8:
                    # Try to truncate the content to fit
                    available_space = max_length * 0.8 - total_length - 100  # Leave buffer
                    if available_space > 50:  # Only add if there's enough space for meaningful content
                        truncated_content = content_preview[:int(available_space)]
                        section_text = f"Section {i+1} - {section_title}:\nContent: {truncated_content}...\nRelevance: {relevance_score:.2f}\n\n"
                    else:
                        break

                prompt += section_text
                total_length += len(section_text)

        prompt += f"User's question: {query}\n\n"
        prompt += (
            "Please provide an accurate answer based on the book content provided above. "
            "If the information is not available in the provided context, state that clearly. "
            "Cite specific sections when possible and maintain a helpful, educational tone."
        )

        return prompt

# Global instance of PromptEngineeringService
prompt_engineering_service = PromptEngineeringService()