from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
from typing import Generator
import os
from src.config import settings

# Create the database engine
engine = create_engine(
    settings.neon_database_url,
    poolclass=QueuePool,
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections after 5 minutes
)

# Create a configured "SessionLocal" class
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for database models
Base = declarative_base()

def get_db() -> Generator:
    """
    Dependency function that provides database sessions
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def create_tables():
    """
    Create all database tables based on models
    """
    Base.metadata.create_all(bind=engine)

# Optional: Function to test database connection
async def test_connection():
    """
    Test the database connection
    """
    try:
        db = SessionLocal()
        # Try to execute a simple query to test the connection
        db.execute("SELECT 1")
        db.close()
        return True
    except Exception as e:
        print(f"Database connection failed: {e}")
        return False