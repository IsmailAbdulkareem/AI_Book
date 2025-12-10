from typing import List, Dict, Any, Optional
from src.services.content_retrieval import content_retrieval_service
from src.services.prompt_engineering import prompt_engineering_service
from src.ai.openai_client import openai_client
from src.logging_config import get_logger
import re

logger = get_logger(__name__)

class ResponseValidationService:
    def __init__(self):
        self.content_retrieval = content_retrieval_service
        self.prompt_engineering = prompt_engineering_service
        self.openai_client = openai_client

    def validate_response_relevance(self, query: str, response: str,
                                  context_sources: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate that the response is relevant to the book content
        """
        validation_result = {
            "is_relevant": True,
            "confidence": 0.8,
            "issues": [],
            "suggestions": []
        }

        # Check if response mentions book-specific terms
        book_related_indicators = [
            "book", "chapter", "section", "Physical AI", "Humanoid Robotics",
            "robotics", "AI", "artificial intelligence", "physical system"
        ]

        response_lower = response.lower()
        has_book_indicators = any(indicator.lower() in response_lower for indicator in book_related_indicators)

        if not has_book_indicators:
            validation_result["issues"].append("Response doesn't clearly reference book content")
            validation_result["is_relevant"] = False
            validation_result["confidence"] = 0.3

        # Check if response is consistent with provided context
        if context_sources:
            context_text = " ".join([source.get('payload', {}).get('content', '')[:200] for source in context_sources if source.get('payload')])
            if context_text and not self._check_response_consistency(response, context_text):
                validation_result["issues"].append("Response appears inconsistent with provided context")
                validation_result["is_relevant"] = False
                validation_result["confidence"] = 0.4

        # Check for generic responses that don't use specific information
        generic_indicators = [
            "I don't have access to that information",
            "I can't find that in the provided text",
            "The book doesn't contain",
            "I'm unable to determine"
        ]

        for indicator in generic_indicators:
            if indicator.lower() in response_lower:
                validation_result["issues"].append(f"Response contains generic indicator: {indicator}")
                # This might be valid if no content was found, so we don't necessarily mark as irrelevant

        # Calculate overall relevance confidence
        if validation_result["is_relevant"]:
            validation_result["confidence"] = self._calculate_relevance_score(query, response, context_sources)

        return validation_result

    def _check_response_consistency(self, response: str, context: str) -> bool:
        """
        Check if the response is consistent with the provided context
        """
        # This is a simplified check - in a real implementation, you might use more sophisticated NLP
        response_lower = response.lower()
        context_lower = context.lower()

        # Check if key terms from context appear in response (indicating it used the context)
        context_words = set(context_lower.split()[:50])  # First 50 words as key terms
        response_words = set(response_lower.split())

        # If less than 20% of context key terms appear in response, it might not be using the context
        common_words = context_words.intersection(response_words)
        if len(common_words) < 0.1 * len(context_words) and len(context_words) > 5:
            return False

        return True

    def _calculate_relevance_score(self, query: str, response: str, context_sources: List[Dict[str, Any]]) -> float:
        """
        Calculate a relevance score based on various factors
        """
        score = 0.5  # Base score

        # Factor 1: How much of the response is supported by context
        if context_sources:
            supported_ratio = min(1.0, len(context_sources) / 5)  # Up to 0.2 per source, max 1.0
            score += supported_ratio * 0.3

        # Factor 2: Response length (very short responses might be low quality)
        response_length = len(response.split())
        if 20 <= response_length <= 200:  # Good length range
            score += 0.1
        elif response_length < 10:  # Too short
            score -= 0.1
        elif response_length > 500:  # Too long
            score -= 0.05

        # Factor 3: Use of specific information from context
        if self._has_specific_info(response, context_sources):
            score += 0.1

        # Ensure score is between 0 and 1
        return max(0.0, min(1.0, score))

    def _has_specific_info(self, response: str, context_sources: List[Dict[str, Any]]) -> bool:
        """
        Check if response contains specific information from context (not just generic answers)
        """
        response_lower = response.lower()

        # Look for specific numbers, technical terms, or proper nouns that would indicate specific content
        specific_patterns = [
            r'\d+%',  # Percentages
            r'\d+\s*(cm|mm|m|kg|g|lb|ft|in)',  # Measurements
            r'\$\d+',  # Prices/dollars
            r'\b[A-Z][a-z]+\b',  # Proper nouns (potential names of concepts/methods)
        ]

        for pattern in specific_patterns:
            if re.search(pattern, response):
                return True

        # Check if response mentions specific details from context
        for source in context_sources:
            payload = source.get('payload', {})
            content = payload.get('content', '')[:300]  # First 300 chars
            if content and len(content) > 20:  # Has substantial content
                # Look for specific terms from context in response
                common_terms = set(content.lower().split()) & set(response_lower.split())
                if len(common_terms) > 3:  # At least 3 common terms
                    return True

        return False

    def validate_response_with_llm(self, query: str, response: str,
                                 context: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Use OpenAI to validate the response quality and relevance
        """
        try:
            validation_prompt = self.prompt_engineering.create_validation_prompt(response, query, context)

            # Generate validation using OpenAI
            validation_response = self.openai_client.generate_response(
                prompt=validation_prompt,
                max_tokens=150,
                temperature=0.2  # Low temperature for more consistent validation
            )

            # Parse the validation result
            validation_text = validation_response.get("response", "").upper()
            is_valid = "VALID" in validation_text

            return {
                "is_valid": is_valid,
                "llm_validation": validation_response.get("response", ""),
                "confidence": validation_response.get("confidence_score", 0.7 if is_valid else 0.3)
            }

        except Exception as e:
            logger.error(f"Error in LLM-based validation: {e}")
            # Fallback to rule-based validation
            return {
                "is_valid": True,  # Default to valid if LLM validation fails
                "llm_validation": "LLM validation unavailable",
                "confidence": 0.5
            }

    def validate_response_completeness(self, query: str, response: str) -> Dict[str, Any]:
        """
        Validate that the response adequately addresses the query
        """
        completeness_result = {
            "is_complete": True,
            "missing_elements": [],
            "confidence": 0.8
        }

        query_lower = query.lower()
        response_lower = response.lower()

        # Check for question words that might indicate incomplete response
        question_words = ["what", "how", "why", "when", "where", "who"]
        query_words = query_lower.split()

        # Check if query starts with a question word and if response addresses it
        if any(word in query_words for word in question_words[:3]):  # what, how, why
            # Check if response provides substantive information
            if len(response.split()) < 10:
                completeness_result["is_complete"] = False
                completeness_result["missing_elements"].append("Response too brief")
                completeness_result["confidence"] = 0.3

        # Check for common incomplete response indicators
        incomplete_indicators = [
            "i don't know",
            "i'm not sure",
            "i cannot determine",
            "insufficient information",
            "not mentioned in the text"
        ]

        for indicator in incomplete_indicators:
            if indicator in response_lower:
                completeness_result["missing_elements"].append(f"Response contains: {indicator}")
                if "not mentioned in the text" not in indicator:
                    completeness_result["is_complete"] = False
                    completeness_result["confidence"] = 0.2

        return completeness_result

    def validate_and_score_response(self, query: str, response: str,
                                  context_sources: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Comprehensive validation combining multiple validation methods
        """
        # Rule-based validation
        relevance_validation = self.validate_response_relevance(query, response, context_sources)

        # Completeness validation
        completeness_validation = self.validate_response_completeness(query, response)

        # LLM-based validation (optional, as it costs API calls)
        # llm_validation = self.validate_response_with_llm(query, response, context_sources)

        # Combine validation results
        overall_score = (
            relevance_validation["confidence"] * 0.5 +
            completeness_validation["confidence"] * 0.3 +
            0.2  # Base score for other factors
        ) / 1.0  # Sum of weights

        return {
            "is_valid": relevance_validation["is_relevant"] and completeness_validation["is_complete"],
            "relevance_score": relevance_validation["confidence"],
            "completeness_score": completeness_validation["confidence"],
            "overall_score": min(1.0, overall_score),
            "issues": relevance_validation["issues"] + completeness_validation["missing_elements"],
            "validation_details": {
                "relevance": relevance_validation,
                "completeness": completeness_validation
            }
        }

# Global instance of ResponseValidationService
response_validation_service = ResponseValidationService()