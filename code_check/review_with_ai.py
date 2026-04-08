import argparse
import os
import sys
import subprocess
import openai
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
        sys.exit(1)

    return api_key


def get_base_url():
    """Get OpenAI API base URL"""
    base_url = os.environ.get("OPENAI_BASE_URL")
    if not base_url:
        # Default to official OpenAI API
        return "https://api.openai.com/v1"

    return base_url


def get_changed_files():
    """Get the list of changed files"""
    try:
        # Get files modified in this commit
        result = subprocess.run(
            ["git", "diff", "--name-only", "HEAD~1", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        files = result.stdout.strip().split("\n")
        return [
            f
            for f in files
            if f
            and f.endswith(
                (".py", ".js", ".ts", ".cpp", ".h", ".c", ".yaml", ".json", ".md")
            )
        ]
    except subprocess.CalledProcessError:
        # If HEAD~1 doesn't exist (e.g., initial commit), return staged files
        try:
            result = subprocess.run(
                ["git", "diff", "--name-only", "--cached"],
                capture_output=True,
                text=True,
                check=True,
            )
            files = result.stdout.strip().split("\n")
            return [
                f
                for f in files
                if f
                and f.endswith(
                    (".py", ".js", ".ts", ".cpp", ".h", ".c", ".yaml", ".json", ".md")
                )
            ]
        except subprocess.CalledProcessError:
            print("Error: Unable to get file list")
            return []


def get_file_diff(filepath):
    """Get the diff content of a single file, optimizing context length"""
    try:
        result = subprocess.run(
            [
                "git",
                "diff",
                "-U3",
                "HEAD~1",
                "HEAD",
                "--",
                filepath,
            ],  # Limit context to 3 lines
            capture_output=True,
            text=True,
            check=True,
        )
        diff = result.stdout

        # If diff is too long, truncate and keep the most important parts
        if len(diff) > 2000:  # If over 2000 characters
            lines = diff.split("\n")
            # Keep first 100 lines and last 100 lines, replace middle with ellipsis
            if len(lines) > 200:
                truncated_lines = (
                    lines[:100]
                    + ["\n... [Some code omitted to save tokens] ...\n"]
                    + lines[-100:]
                )
                diff = "\n".join(truncated_lines)

        return diff
    except subprocess.CalledProcessError:
        print(f"Error: Unable to get diff for file {filepath}")
        return None


def review_code_with_ai(file_path, diff_content):
    """Review code with AI, optimizing prompt to save tokens"""
    api_key = get_api_key()
    base_url = get_base_url()

    # Set up OpenAI client
    client = openai.OpenAI(api_key=api_key, base_url=base_url)

    # Optimized prompt, concise and clear, saving tokens
    prompt = f"""
As a code reviewer, please check the following code changes and provide feedback:
- Point out security vulnerabilities
- Point out performance issues
- Point out missing error handling
- Point out code style issues
- Provide brief improvement suggestions

File: {file_path}

Code changes:
{diff_content}

Please respond in concise language, maximum 5 points.
"""

    try:
        # Dynamically select model based on API provider
        model = os.environ.get("OPENAI_MODEL", "gpt-4o-mini")

        response = client.chat.completions.create(
            model=model,  # Use model specified in environment variable
            messages=[{"role": "user", "content": prompt}],
            max_tokens=500,  # Limit response length
            temperature=0.2,
        )

        return response.choices[0].message.content

    except Exception as e:
        print(f"Error: Failed to call OpenAI API - {str(e)}")
        return None


def main():
    parser = argparse.ArgumentParser(description="Code review with OpenAI")
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Show detailed output"
    )
    args = parser.parse_args()

    # Set API key and base URL
    api_key = get_api_key()
    base_url = get_base_url()
    openai.api_key = api_key
    openai.base_url = base_url

    # Get modified files
    changed_files = get_changed_files()

    if not changed_files:
        print("No modified files found")
        return 0

    print(f"Found {len(changed_files)} modified file(s):")
    for f in changed_files:
        print(f"  - {f}")

    print("\nStarting AI code review...")

    total_issues = 0

    for file_path in changed_files:
        print(f"\nReviewing file: {file_path}")

        diff_content = get_file_diff(file_path)
        if not diff_content or not diff_content.strip():
            print(f"  Skipping {file_path} (no diff content)")
            continue

        if args.verbose:
            print("  Analyzing...")

        review_result = review_code_with_ai(file_path, diff_content)

        if not review_result:
            print(f"  ❌ Review failed: {file_path}")
            continue

        print(f"  ✅ Review completed: {file_path}")
        print(f"\nReview Result ({file_path}):")
        print("-" * 40)
        print(review_result)
        print("-" * 40)

        # Count number of issues (simple estimation)
        total_issues += review_result.count("- ") if review_result else 0

    print(
        f"\n📊 Code review completed! Found approximately {total_issues} potential issues."
    )
    return 0 if total_issues == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
