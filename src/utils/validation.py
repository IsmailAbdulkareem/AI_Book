import re
from typing import Any, Optional, Union
from urllib.parse import urlparse
import bleach
from pydantic import ValidationError
from uuid import UUID

def validate_uuid(uuid_string: str) -> bool:
    """
    Validate if a string is a valid UUID
    """
    try:
        UUID(uuid_string)
        return True
    except ValueError:
        return False

def validate_url(url_string: str) -> bool:
    """
    Validate if a string is a valid URL
    """
    try:
        result = urlparse(url_string)
        return all([result.scheme, result.netloc])
    except Exception:
        return False

def sanitize_text(text: str) -> str:
    """
    Sanitize text input to prevent XSS and other injection attacks
    """
    if not text:
        return text

    # Use bleach to sanitize HTML content
    sanitized = bleach.clean(text, tags=[], attributes={}, strip=True)

    # Remove any potential script tags or other dangerous content
    sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
    sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
    sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)
    sanitized = re.sub(r'on\w+\s*=', '', sanitized, flags=re.IGNORECASE)

    return sanitized.strip()

def validate_content_length(text: str, min_length: int = 1, max_length: int = 10000) -> tuple[bool, str]:
    """
    Validate the length of content
    Returns (is_valid, error_message)
    """
    if len(text) < min_length:
        return False, f"Content must be at least {min_length} characters long"
    if len(text) > max_length:
        return False, f"Content must be no more than {max_length} characters long"
    return True, ""

def validate_selected_text_length(text: str) -> tuple[bool, str]:
    """
    Validate the length of selected text according to requirements
    """
    if len(text) < 10:
        return False, "Selected text must be at least 10 characters long"
    if len(text) > 5000:
        return False, "Selected text must be no more than 5000 characters long"
    return True, ""

def validate_message_content(content: str) -> tuple[bool, str]:
    """
    Validate message content
    """
    if not content or not content.strip():
        return False, "Message content cannot be empty"

    is_valid, error_msg = validate_content_length(content, min_length=1, max_length=10000)
    if not is_valid:
        return False, error_msg

    return True, ""

def validate_page_url(url: str) -> tuple[bool, str]:
    """
    Validate page URL
    """
    if not url:
        return True, ""  # URL is optional

    if not validate_url(url):
        return False, "Invalid URL format"

    # Additional validation for Docusaurus-style URLs
    if not url.startswith('/') and not url.startswith('http'):
        return False, "URL should be relative (starting with /) or absolute (starting with http/https)"

    return True, ""

def sanitize_user_input(data: Union[str, dict, list, Any]) -> Union[str, dict, list, Any]:
    """
    Recursively sanitize user input data
    """
    if isinstance(data, str):
        return sanitize_text(data)
    elif isinstance(data, dict):
        return {key: sanitize_user_input(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [sanitize_user_input(item) for item in data]
    else:
        return data

def normalize_whitespace(text: str) -> str:
    """
    Normalize whitespace in text (replace multiple spaces, tabs, newlines with single spaces)
    """
    if not text:
        return text
    # Replace multiple whitespace characters with a single space
    normalized = re.sub(r'\s+', ' ', text)
    return normalized.strip()

def validate_session_token(token: str) -> tuple[bool, str]:
    """
    Validate session token format
    """
    if not token:
        return False, "Session token cannot be empty"

    # Basic validation: token should be a reasonable length and not contain dangerous characters
    if len(token) < 10:
        return False, "Session token is too short"

    if len(token) > 255:
        return False, "Session token is too long"

    # Check for potentially dangerous patterns
    if re.search(r'[<>"\'&]', token):
        return False, "Session token contains invalid characters"

    return True, ""

def is_valid_email(email: str) -> bool:
    """
    Basic email validation
    """
    if not email:
        return False

    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    return re.match(pattern, email) is not None

def validate_json_payload(payload: dict, required_fields: list = None) -> tuple[bool, str]:
    """
    Validate JSON payload structure
    """
    if not isinstance(payload, dict):
        return False, "Payload must be a dictionary"

    if required_fields:
        for field in required_fields:
            if field not in payload:
                return False, f"Missing required field: {field}"

    return True, ""

def extract_urls_from_text(text: str) -> list[str]:
    """
    Extract URLs from text
    """
    if not text:
        return []

    # Pattern to match URLs
    url_pattern = r'https?://[^\s<>"{}|\\^`\[\]]+'
    urls = re.findall(url_pattern, text)

    # Validate each URL
    valid_urls = []
    for url in urls:
        if validate_url(url):
            valid_urls.append(url)

    return valid_urls

def validate_content_type(content_type: str) -> tuple[bool, str]:
    """
    Validate content type against allowed values
    """
    allowed_types = ["chapter", "section", "subsection", "figure", "table"]

    if content_type not in allowed_types:
        return False, f"Content type must be one of: {', '.join(allowed_types)}"

    return True, ""