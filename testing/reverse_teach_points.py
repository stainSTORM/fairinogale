import sys
import json
from pathlib import Path


def reverse_teach_points(input_path: str) -> None:
    input_file = Path(input_path)
    if not input_file.exists():
        print(f"Error: File '{input_path}' not found.")
        sys.exit(1)

    with open(input_file, "r") as f:
        points = json.load(f)

    reversed_points = dict(reversed(list(points.items())))

    output_file = input_file.with_stem(input_file.stem + "_reversed")
    with open(output_file, "w") as f:
        json.dump(reversed_points, f, indent=4)

    print(f"Reversed {len(reversed_points)} points -> {output_file}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python reverse_teach_points.py <teach_points.json>")
        sys.exit(1)

    reverse_teach_points(sys.argv[1])
