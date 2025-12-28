import json
from typing import Dict, Any
import ollama  # pip install ollama


SYSTEM_PROMPT = """
You are a command-to-control translator for an RC car with Ackermann steering.

You receive a short natural language command from the user.
You MUST respond with ONLY a JSON object (no text, no explanation).

The JSON schema is strictly:
{
  "speed": float,
  "steering_angle": float,
  "steering_angle_velocity": float,
  "acceleration": float,
  "jerk": float
}

Rules:
- If the user says "set velocity to X", set speed = X (float).
- If no speed is mentioned, keep speed = 0.0.
- If no steering command is mentioned, steering_angle = 0.0.
- If the user says "turn left/right", set steering_angle to a small angle:
  - "small left/right": +/-0.2
  - "left/right": +/-0.4
  - "sharp left/right": +/-0.6
- If the user mentions units like "km/h", convert to m/s (1 km/h ≈ 0.27778 m/s).
- acceleration, jerk, steering_angle_velocity can be 0.0 unless explicitly mentioned.

Output ONLY valid JSON, no markdown, no comments, no surrounding text.
"""


def parse_drive_command(command: str) -> Dict[str, Any]:
    response = ollama.chat(
        model="llama3.2:1b",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": command},
        ],
        format={
            "type": "object",
            "properties": {
                "speed": {"type": "number"},
                "steering_angle": {"type": "number"},
                "steering_angle_velocity": {"type": "number"},
                "acceleration": {"type": "number"},
                "jerk": {"type": "number"},
            },
            "required": [
                "speed",
                "steering_angle",
                "steering_angle_velocity",
                "acceleration",
                "jerk",
            ],
        },
    )

    content = response["message"]["content"]
    data = json.loads(content)

    defaults = {
        "speed": 0.0,
        "steering_angle": 0.0,
        "steering_angle_velocity": 0.0,
        "acceleration": 0.0,
        "jerk": 0.0,
    }
    defaults.update(data)
    return defaults


def main():
    print("\n🚗 Ackermann Command Translator (Ollama llama3.2:1b)")
    print("Type a driving command, or 'q' to quit.\n")

    while True:
        command = input("Command: ").strip()
        if command.lower() == "q":
            break

        if not command:
            continue

        try:
            result = parse_drive_command(command)
            print("\nParsed command →")
            print(json.dumps(result, indent=2))
            print()
        except Exception as e:
            print(f"\nError parsing command: {e}\n")


if __name__ == "__main__":
    main()
