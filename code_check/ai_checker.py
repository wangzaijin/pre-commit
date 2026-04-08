#!/usr/bin/env python3
"""
Demo AI Code Checker - Proof of concept demonstration
This version does not actually call the OpenAI API, but simulates the process
"""

import argparse
import sys
import subprocess


def get_modified_files():
    """Get the list of modified files"""
    try:
        result = subprocess.run(
            ["git", "diff", "--name-only", "--cached"],
            capture_output=True,
            text=True,
            check=True,
        )
        files = result.stdout.strip().split("\n")
        return [f for f in files if f]  # Filter out empty strings
    except subprocess.CalledProcessError:
        print("Error: Unable to get the list of staged files")
        print("Error: Unable to get the list of staged files 1 2 3")
        print("Error: Unable to get the list of staged files 1")
        sys.exit(1)


def get_file_diff(filepath):
    """Get the diff content of a single file"""
    try:
        result = subprocess.run(
            ["git", "diff", "--cached", filepath],
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout
    except subprocess.CalledProcessError:
        print(f"Error: Unable to get diff for file {filepath}")
        return None


def simulate_ai_analysis(file_path, diff_content):
    """Simulate AI analysis process"""
    print(f"🔍 Simulating analysis of {file_path} ...")

    # Check for some common code issues
    issues = []

    if ".py" in file_path:
        # Check for common issues in Python code
        if "input(" in diff_content and (
            "os.system" in diff_content or "subprocess" in diff_content
        ):
            issues.append(
                {
                    "type": "security",
                    "description": "Detected user input directly used in system command, potential security risk",
                    "suggestion": "Use parameterized methods or validate user input",
                }
            )

        if "open(" in diff_content and "try:" not in diff_content:
            issues.append(
                {
                    "type": "error_handling",
                    "description": "Detected file operations without exception handling",
                    "suggestion": "Add try-except blocks to handle possible IO errors",
                }
            )

        if "TODO" in diff_content or "FIXME" in diff_content:
            issues.append(
                {
                    "type": "development",
                    "description": "Detected to-do comment markers",
                    "suggestion": "Complete the tasks marked with TODO/FIXME",
                }
            )

    return issues


def main():
    parser = argparse.ArgumentParser(description="Demo AI Code Checker")
    parser.add_argument(
        "--show-warnings-only",
        action="store_true",
        help="Show warnings only, don't return error code",
    )
    args = parser.parse_args()

    # Get modified files
    modified_files = get_modified_files()

    if not modified_files:
        print("No modified files found")
        return 0

    print(f"Found {len(modified_files)} modified file(s):")
    for f in modified_files:
        print(f"  - {f}")

    issues_found = 0

    for file_path in modified_files:
        # Only check common code file types
        if not any(
            file_path.endswith(ext)
            for ext in [".py", ".js", ".ts", ".cpp", ".h", ".yaml", ".json", ".md"]
        ):
            continue

        print(f"\n📄 Checking file: {file_path}")

        diff_content = get_file_diff(file_path)
        if not diff_content or not diff_content.strip():
            print(f"  Skipping {file_path} (no diff content)")
            continue

        issues = simulate_ai_analysis(file_path, diff_content)

        if issues:
            issues_found += len(issues)
            print(f"  ⚠️  Found {len(issues)} issue(s) in {file_path}:")
            for i, issue in enumerate(issues, 1):
                print(f"     {i}. [{issue['type']}] {issue['description']}")
                print(f"        Suggestion: {issue['suggestion']}")
        else:
            print(f"  ✅ No issues found: {file_path}")

    if issues_found > 0:
        print(f"\n📊 Summary: Found {issues_found} issue(s).")
        print(
            "In a real environment, these would be analyzed in detail by AI and suggested fixes provided."
        )

        # If in warning mode, don't return error code
        if args.show_warnings_only:
            print("Running in warning mode, commit will continue...")
            return 0
        else:
            return 1
    else:
        print("\n✅ All files passed basic checks!")
        return 0


if __name__ == "__main__":
    sys.exit(main())
