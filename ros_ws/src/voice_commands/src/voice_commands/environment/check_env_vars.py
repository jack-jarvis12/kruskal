import os
from dotenv import load_dotenv

def check_env_vars():
    load_dotenv()
    assert os.getenv("PICOVOICE_ACCESS_KEY") is not None, "PICOVOICE_ACCESS_KEY is not set in .env file."
    assert os.getenv("OPENAI_API_KEY") is not None, "OPENAI_API_KEY is not set in .env file."

