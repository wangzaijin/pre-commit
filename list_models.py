#!/usr/bin/env python3
"""
Get the list of available OpenAI models
"""

import openai
import os
from pathlib import Path


def get_api_key():
    """Get OpenAI API key"""
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        # Try to read from config file
        config_path = Path.home() / ".openai_config"
        if config_path.exists():
            with open(config_path, "r") as f:
                for line in f:
                    if line.startswith("OPENAI_API_KEY="):
                        api_key = line.split("=", 1)[1].strip()

    if not api_key:
        print(
            "Error: OpenAI API key not found. Please set the OPENAI_API_KEY environment variable or configure it in ~/.openai_config file."
        )
        return None

    return api_key


def main():
    # Get API key and base URL
    api_key = get_api_key()
    if not api_key:
        return 1

    base_url = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1")

    # Set up OpenAI client
    client = openai.OpenAI(api_key=api_key, base_url=base_url)

    try:
        models = client.models.list()
        print("Available models:")
        for model in models.data:
            print(f"- {model.id}")
    except Exception as e:
        print(f"Failed to get model list: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
