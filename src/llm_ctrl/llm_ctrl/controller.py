import json
from typing import Dict, Any

import ollama  # pip install ollama


SYSTEM_PROMPT = """
You are a command-to-control translator for an RC car with Ackermann steering.

You receive a short natural language command from the user.
You MUST respond with ONLY a JSON object (no text, no explanation).

The JSON schema is strictly:
{
  "speed": float,                    # forward speed in m/s
  "steering_angle": float,           # radians, +left, -right
  "steering_angle_velocity": float,  # radians/s
  "acceleration": float,             # m/s^2
  "jerk": float                      # m/s^3
}

Rules:
- If the user says "set velocity to X", set "speed" = X (float).
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
    """
    Call llama3.2 via Ollama to convert a natural language command
    into Ackermann drive parameters.
    """
    response = ollama.chat(
        model="llama3.2",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": command},
        ],
    )

    content = response["message"]["content"]
    # Safety: sometimes the model may add backticks etc., so try to extract JSON
    try:
        data = json.loads(content)
    except json.JSONDecodeError:
        # Very simple fallback: strip code fences if present
        cleaned = content.strip()
        if cleaned.startswith("```
            cleaned = cleaned.strip("`")
            # Remove possible "json" language marker
            cleaned = cleaned.replace("json", "", 1).strip()
        data = json.loads(cleaned)

    # Ensure all keys exist
    default = {
        "speed": 0.0,
        "steering_angle": 0.0,
        "steering_angle_velocity": 0.0,
        "acceleration": 0.0,
        "jerk": 0.0,
    }
    default.update(data)
    return default
