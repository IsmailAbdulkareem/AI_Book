import os
import time
import uuid
import logging
import requests
import xml.etree.ElementTree as ET

from typing import List
from urllib.parse import urljoin, urlparse

from bs4 import BeautifulSoup
from dotenv import load_dotenv

import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from qdrant_client.http import models


# -----------------------------------------------------------------------------
# Environment & Logging
# -----------------------------------------------------------------------------

load_dotenv()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("docusaurus-rag-pipeline")


# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------

DEFAULT_COLLECTION = os.getenv("QDRANT_COLLECTION", "rag_embedding")
COHERE_EMBED_MODEL = "embed-multilingual-v3.0"
VECTOR_SIZE = 1024
REQUEST_TIMEOUT = 15


# -----------------------------------------------------------------------------
# Pipeline
# -----------------------------------------------------------------------------

class DocusaurusEmbeddingPipeline:
    def __init__(self, target_url: str):
        self.target_url = target_url.rstrip("/") + "/"

        self.cohere_client = cohere.Client(
            api_key=os.getenv("COHERE_API_KEY")
        )

        self.qdrant_client = self._init_qdrant()

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def _init_qdrant(self) -> QdrantClient:
        url = os.getenv("QDRANT_URL", "http://localhost:6333")
        api_key = os.getenv("QDRANT_API_KEY")

        return (
            QdrantClient(url=url, api_key=api_key)
            if api_key
            else QdrantClient(url=url)
        )

    # -------------------------------------------------------------------------
    # URL Discovery
    # -------------------------------------------------------------------------

    def get_all_urls(self) -> List[str]:
        """Extract all URLs using sitemap.xml with crawl fallback."""
        sitemap_url = urljoin(self.target_url, "sitemap.xml")
        urls: List[str] = []

        try:
            response = requests.get(sitemap_url, timeout=REQUEST_TIMEOUT)

            if response.status_code == 200:
                urls.extend(self._parse_sitemap(response.content))
            else:
                logger.info("Sitemap not found, falling back to crawl")
                urls.extend(self._crawl_site())

        except Exception as e:
            logger.error(f"URL discovery failed: {e}")

        return list(dict.fromkeys(urls))  # preserve order, remove duplicates

    def _parse_sitemap(self, xml_content: bytes) -> List[str]:
        urls = []
        root = ET.fromstring(xml_content)

        namespace = {"ns": "http://www.sitemaps.org/schemas/sitemap/0.9"}

        # sitemap index
        sitemap_locs = root.findall(".//ns:sitemap/ns:loc", namespace)
        if sitemap_locs:
            for loc in sitemap_locs:
                try:
                    r = requests.get(loc.text, timeout=REQUEST_TIMEOUT)
                    r.raise_for_status()
                    urls.extend(self._parse_sitemap(r.content))
                except Exception:
                    continue
            return urls

        # regular sitemap
        for loc in root.findall(".//ns:url/ns:loc", namespace):
            urls.append(loc.text)

        return urls

    def _crawl_site(self) -> List[str]:
        urls = []
        response = requests.get(self.target_url, timeout=REQUEST_TIMEOUT)
        soup = BeautifulSoup(response.content, "html.parser")

        for link in soup.find_all("a", href=True):
            full_url = urljoin(self.target_url, link["href"])
            if self._is_valid_internal_url(full_url):
                urls.append(full_url)

        return urls

    def _is_valid_internal_url(self, url: str) -> bool:
        return (
            urlparse(url).netloc == urlparse(self.target_url).netloc
            and url.startswith(self.target_url)
        )

    # -------------------------------------------------------------------------
    # Content Extraction
    # -------------------------------------------------------------------------

    def extract_text_from_url(self, url: str) -> str:
        try:
            response = requests.get(url, timeout=REQUEST_TIMEOUT)
            response.raise_for_status()
        except Exception as e:
            logger.error(f"Failed to fetch {url}: {e}")
            return ""

        soup = BeautifulSoup(response.content, "html.parser")

        for tag in soup(["script", "style"]):
            tag.decompose()

        selectors = [
            "article",
            ".markdown",
            ".theme-doc-markdown",
            "[role='main']",
            "main",
        ]

        content = ""
        for selector in selectors:
            elements = soup.select(selector)
            if elements:
                content = max(
                    (el.get_text(" ", strip=True) for el in elements),
                    key=len,
                    default=""
                )
                break

        if not content and soup.body:
            content = soup.body.get_text(" ", strip=True)

        return self._normalize_text(content)

    def _normalize_text(self, text: str) -> str:
        return " ".join(text.split())

    # -------------------------------------------------------------------------
    # Chunking
    # -------------------------------------------------------------------------

    def chunk_text(
        self,
        text: str,
        chunk_size: int = 1000,
        overlap: int = 100
    ) -> List[str]:
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size
            chunks.append(text[start:end])
            start = max(end - overlap, 0)

            if start >= len(text):
                break

        return chunks

    # -------------------------------------------------------------------------
    # Embeddings
    # -------------------------------------------------------------------------

    def embed(self, text: str) -> List[float]:
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model=COHERE_EMBED_MODEL,
                input_type="search_document",
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Embedding failed: {e}")
            return []

    # -------------------------------------------------------------------------
    # Qdrant
    # -------------------------------------------------------------------------

    def create_collection(self, name: str = DEFAULT_COLLECTION) -> None:
        existing = {
            col.name for col in self.qdrant_client.get_collections().collections
        }

        if name in existing:
            logger.info(f"Collection '{name}' already exists")
            return

        self.qdrant_client.create_collection(
            collection_name=name,
            vectors_config=models.VectorParams(
                size=VECTOR_SIZE,
                distance=models.Distance.COSINE,
            ),
        )

        logger.info(f"Created collection '{name}'")

    def save_chunk(
        self,
        *,
        content: str,
        url: str,
        embedding: List[float],
        position: int,
        collection: str = DEFAULT_COLLECTION,
    ) -> bool:
        try:
            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "content": content,
                    "url": url,
                    "position": position,
                    "created_at": time.time(),
                },
            )

            self.qdrant_client.upsert(
                collection_name=collection,
                points=[point],
            )
            return True

        except Exception as e:
            logger.error(f"Qdrant upsert failed: {e}")
            return False


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    logger.info("Starting Docusaurus Embedding Pipeline")

    pipeline = DocusaurusEmbeddingPipeline(
        target_url="https://ismailabdulkareem.github.io/AI_Book/"
    )

    pipeline.create_collection()

    urls = pipeline.get_all_urls()
    if not urls:
        logger.warning("No URLs discovered")
        return

    logger.info(f"Discovered {len(urls)} URLs")

    total_chunks = 0

    for idx, url in enumerate(urls, start=1):
        logger.info(f"[{idx}/{len(urls)}] Processing {url}")

        text = pipeline.extract_text_from_url(url)
        if not text:
            continue

        chunks = pipeline.chunk_text(text)

        for pos, chunk in enumerate(chunks):
            embedding = pipeline.embed(chunk)
            if not embedding:
                continue

            if pipeline.save_chunk(
                content=chunk,
                url=url,
                embedding=embedding,
                position=pos,
            ):
                total_chunks += 1

    logger.info(f"Pipeline completed. Total chunks stored: {total_chunks}")


if __name__ == "__main__":
    main()
