"""
CLI Chatbot - Terminal-based RAG Chatbot for Testing

Spec 4: 004-frontend-chatbot (CLI Track)
Purpose: Interactive terminal chatbot for validating RAG pipeline

This module provides:
- Interactive chat loop for asking questions
- Grounded answers with source citations
- Reuses RetrievalPipeline from Spec 2 and OpenAI from Spec 3

Usage:
    cd database
    uv run python chatbot.py
"""

import os
import sys
import time
from typing import List

from dotenv import load_dotenv
from openai import OpenAI

from retrieval import QueryResult, RetrievalPipeline

# Import intent detection from agent.py for consistency
from agent import (
    detect_intent,
    MessageIntent,
    get_greeting_response,
    get_meta_response,
)

# Load environment variables
load_dotenv()

# =============================================================================
# Constants (same as agent.py for consistency)
# =============================================================================

SYSTEM_PROMPT = """You are a helpful assistant for the Physical AI & Humanoid Robotics book.

YOUR ROLE:
- You answer questions about topics covered in this book
- All your factual answers must be grounded in the provided context
- You cite sources using [1], [2], etc.

ANSWERING RULES:
1. Use ONLY the provided context for factual information
2. Cite sources by number [1], [2], etc. when referencing specific content
3. Be concise, accurate, and helpful
4. Never make up or hallucinate information not in the context

WHEN TO SAY "I don't have information about that topic in the book":
- ONLY when the user asks a factual/conceptual question about the book's subject matter
- AND the provided context does not contain relevant information to answer it
- Do NOT say this for conversational messages or follow-up questions that can be answered from prior context

Context will be provided in the format:
[1] Content...
Source: URL

Answer the user's question based on this context."""

WELCOME_MESSAGE = """
╔══════════════════════════════════════════════════════════════════╗
║     Physical AI & Humanoid Robotics Book - CLI Chatbot           ║
╠══════════════════════════════════════════════════════════════════╣
║  Ask me anything about the book content!                         ║
║  Type 'quit' or 'exit' to leave.                                 ║
╚══════════════════════════════════════════════════════════════════╝
"""


# =============================================================================
# Helper Functions
# =============================================================================


def format_context(results: List[QueryResult]) -> str:
    """Format retrieved results as numbered context for the LLM.

    Args:
        results: List of QueryResult from retrieval pipeline

    Returns:
        Formatted string with numbered sources
    """
    if not results:
        return ""

    context_parts = []
    for i, result in enumerate(results, 1):
        context_parts.append(f"[{i}] {result.content}\nSource: {result.url}")

    return "\n\n".join(context_parts)


def generate_answer(question: str, context: str) -> tuple[str, int]:
    """Generate a grounded answer using OpenAI.

    Args:
        question: The user's question
        context: Formatted context from retrieval

    Returns:
        Tuple of (answer text, generation time in ms)
    """
    client = OpenAI()

    start_time = time.time()

    if not context:
        user_content = f"Question: {question}\n\nNote: No relevant context was found in the book."
    else:
        user_content = f"Context:\n{context}\n\nQuestion: {question}"

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_content},
        ],
        temperature=0.3,
        max_tokens=1000,
    )

    generation_time_ms = int((time.time() - start_time) * 1000)
    answer = response.choices[0].message.content or ""

    return answer, generation_time_ms


def print_sources(results: List[QueryResult]) -> None:
    """Print source citations in a readable format.

    Args:
        results: List of QueryResult from retrieval
    """
    if not results:
        return

    print("\n📚 Sources:")
    print("-" * 60)
    for i, result in enumerate(results, 1):
        # Truncate content to 100 chars for display
        content_preview = result.content[:100] + "..." if len(result.content) > 100 else result.content
        print(f"  [{i}] {result.url}")
        print(f"      Score: {result.score:.3f}")
        print(f"      Preview: {content_preview}")
        print()


def ask_question(question: str, top_k: int = 5) -> None:
    """Ask a question and print the grounded answer.

    Uses intent detection to handle greetings and meta questions
    without invoking the RAG pipeline.

    Args:
        question: The question to ask
        top_k: Number of sources to retrieve
    """
    # ==========================================================================
    # Step 1: Intent Detection - Route greetings and meta questions
    # ==========================================================================
    intent = detect_intent(question)

    # Handle greetings (hi, hello, thanks, etc.) - no RAG needed
    if intent == MessageIntent.GREETING:
        print("\n" + "=" * 60)
        print("📝 Answer:")
        print("=" * 60)
        print(get_greeting_response())
        print("=" * 60)
        return

    # Handle meta questions (what can you do, how do you work, etc.)
    if intent == MessageIntent.META:
        print("\n" + "=" * 60)
        print("📝 Answer:")
        print("=" * 60)
        print(get_meta_response())
        print("=" * 60)
        return

    # ==========================================================================
    # Step 2: Content questions - Use full RAG pipeline
    # ==========================================================================
    print("\n🔍 Searching for relevant content...")

    # Initialize retrieval pipeline
    try:
        pipeline = RetrievalPipeline()
    except Exception as e:
        print(f"\n❌ Error connecting to vector database: {e}")
        return

    # Search for relevant context
    try:
        start_time = time.time()
        response = pipeline.search(query=question, top_k=top_k)
        retrieval_time_ms = int((time.time() - start_time) * 1000)
        results = response.results
        print(f"   Found {len(results)} relevant chunks in {retrieval_time_ms}ms")
    except Exception as e:
        print(f"\n❌ Error during retrieval: {e}")
        return

    # Format context
    context = format_context(results)

    # Generate answer
    print("💭 Generating answer...")
    try:
        answer, generation_time_ms = generate_answer(question, context)
    except Exception as e:
        print(f"\n❌ Error generating answer: {e}")
        return

    # Print answer
    print("\n" + "=" * 60)
    print("📝 Answer:")
    print("=" * 60)
    print(answer)

    # Print sources
    print_sources(results)

    # Print timing
    print(f"⏱️  Retrieval: {retrieval_time_ms}ms | Generation: {generation_time_ms}ms")
    print("=" * 60)


def interactive_chat() -> None:
    """Run the interactive chat loop."""
    # Configure stdout for UTF-8 on Windows
    if sys.platform == "win32":
        sys.stdout.reconfigure(encoding="utf-8")

    print(WELCOME_MESSAGE)

    while True:
        try:
            # Get user input
            print()
            question = input("You: ").strip()

            # Check for exit commands
            if question.lower() in ("quit", "exit", "q"):
                print("\n👋 Goodbye! Thanks for using the chatbot.")
                break

            # Skip empty input
            if not question:
                print("Please enter a question.")
                continue

            # Ask the question
            ask_question(question)

        except KeyboardInterrupt:
            print("\n\n👋 Goodbye! Thanks for using the chatbot.")
            break
        except EOFError:
            print("\n\n👋 Goodbye! Thanks for using the chatbot.")
            break


# =============================================================================
# Main Entry Point
# =============================================================================

if __name__ == "__main__":
    # Load environment variables
    load_dotenv()

    # Verify required environment variables
    if not os.getenv("OPENAI_API_KEY"):
        print("❌ Error: OPENAI_API_KEY environment variable not set.")
        print("   Please set it in your .env file or environment.")
        sys.exit(1)

    if not os.getenv("QDRANT_URL"):
        print("❌ Error: QDRANT_URL environment variable not set.")
        print("   Please set it in your .env file or environment.")
        sys.exit(1)

    # Run interactive chat
    interactive_chat()
