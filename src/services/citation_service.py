from typing import List, Dict, Any
from src.models.database import ChatMessage
from src.services.content_retrieval import content_retrieval_service

class CitationService:
    def __init__(self):
        self.content_retrieval = content_retrieval_service

    def format_sources(self, search_results: List[Dict[str, Any]], max_sources: int = 5) -> List[Dict[str, Any]]:
        """
        Format search results into proper citation format
        """
        formatted_sources = []

        # Sort by relevance score in descending order
        sorted_results = sorted(search_results, key=lambda x: x.get('score', 0), reverse=True)

        for result in sorted_results[:max_sources]:
            payload = result.get('payload', {})

            formatted_source = {
                "section_id": payload.get('section_id', 'N/A'),
                "title": payload.get('title', 'Untitled Section'),
                "page_url": payload.get('page_url', '#'),
                "relevance_score": result.get('score', 0.0),
                "content_preview": payload.get('content', '')[:200] + "..." if payload.get('content') else "",
                "content_type": payload.get('content_type', 'unknown')
            }

            formatted_sources.append(formatted_source)

        return formatted_sources

    def generate_citation_text(self, sources: List[Dict[str, Any]], style: str = "apa") -> str:
        """
        Generate citation text in specified style
        """
        if not sources:
            return "No sources referenced."

        if style.lower() == "apa":
            citation_text = "References:\n"
            for i, source in enumerate(sources, 1):
                citation_text += f"[{i}] {source['title']} (Section: {source['section_id']}, {source['page_url']})\n"
        elif style.lower() == "book":
            citation_text = "Sources in the book:\n"
            for source in sources:
                citation_text += f"- {source['title']} - Section {source['section_id']}, {source['page_url']}\n"
        else:
            citation_text = "Referenced sections:\n"
            for source in sources:
                citation_text += f"- {source['title']} ({source['relevance_score']:.2f})\n"

        return citation_text.strip()

    def extract_relevant_facts(self, sources: List[Dict[str, Any]], query: str) -> List[Dict[str, str]]:
        """
        Extract relevant facts from sources related to the query
        """
        facts = []

        for source in sources:
            content = source.get('content_preview', '')
            # This is a simplified approach - in a real implementation, you might use NLP to extract specific facts
            if query.lower() in content.lower():
                facts.append({
                    "fact": content,
                    "source": f"{source['title']} (Section {source['section_id']})",
                    "page_url": source['page_url']
                })

        return facts

    def create_citation_prompt(self, response: str, sources: List[Dict[str, Any]]) -> str:
        """
        Create a prompt to enhance response with proper citations
        """
        if not sources:
            return response

        # Add source citations to the response
        citation_part = "\n\nSources referenced:\n"
        for i, source in enumerate(sources, 1):
            citation_part += f"[{i}] {source['title']} - {source['page_url']}\n"

        return response + citation_part

    def validate_citations(self, response: str, sources: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate that the response properly references the provided sources
        """
        validation_result = {
            "valid": True,
            "missing_citations": [],
            "issues": []
        }

        if not sources:
            return validation_result

        response_lower = response.lower()

        for source in sources:
            # Check if the section_id or title appears in the response
            section_id = source.get('section_id', '')
            title = source.get('title', '')

            if section_id and section_id.lower() not in response_lower:
                if title and title.lower() not in response_lower:
                    validation_result["missing_citations"].append(source['section_id'])
                    validation_result["issues"].append(f"Source '{source['title']}' not referenced in response")

        validation_result["valid"] = len(validation_result["missing_citations"]) == 0

        return validation_result

    def create_source_context(self, sources: List[Dict[str, Any]], max_length: int = 1000) -> str:
        """
        Create a context string from sources that can be used in prompts
        """
        if not sources:
            return ""

        context = "Relevant book sections:\n"
        current_length = len(context)

        for source in sources:
            source_text = f"Section: {source['title']} (ID: {source['section_id']})\n"
            source_text += f"Content preview: {source['content_preview']}\n\n"

            if current_length + len(source_text) > max_length:
                # Truncate to fit the limit
                remaining_space = max_length - current_length - 20  # Leave some space for safety
                if remaining_space > 0:
                    truncated_source = source_text[:remaining_space] + "...\n\n"
                    context += truncated_source
                break
            else:
                context += source_text
                current_length += len(source_text)

        return context

# Global instance of CitationService
citation_service = CitationService()