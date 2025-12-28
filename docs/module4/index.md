---
id: module4-vla
title: "Module 4: Vision-Language-Action (VLA)"
sidebar_position: 7
---

# Module 4: Vision-Language-Action (VLA)

## Overview

**Vision-Language-Action (VLA)** is a paradigm that enables robots to understand natural language commands and execute them in the physical world. VLA systems combine:

- **Vision**: Understanding the visual world through cameras
- **Language**: Interpreting natural language commands
- **Action**: Executing robot behaviors via ROS 2

This module shows you how to build end-to-end VLA systems that translate "Clean the room" into a sequence of robot actions.

## What is VLA?

VLA bridges the gap between human communication and robot execution:

1. **Human speaks**: "Pick up the red cup and place it on the table"
2. **System transcribes**: Speech-to-text conversion
3. **System understands**: LLM interprets the command and plans subtasks
4. **System executes**: ROS 2 actions perform the planned behaviors

VLA makes robots accessible to non-technical users and enables complex, multi-step tasks.

## Components of a VLA System

### 1. Voice Input (ASR)

**Automatic Speech Recognition (ASR)** converts speech to text:

**OpenAI Whisper**:
- Open-source, high-accuracy ASR
- Supports multiple languages
- Can run locally or via API
- Good balance of accuracy and latency

**Alternative Options**:
- Google Speech-to-Text (cloud-based)
- Azure Speech Services
- Local models (Vosk, DeepSpeech)

### 2. LLM-Based Task Planning

**Large Language Models (LLMs)** understand natural language and can generate structured plans:

**Task Planning Process**:
1. LLM receives natural language command
2. LLM breaks down command into subtasks
3. LLM maps subtasks to robot capabilities
4. LLM outputs structured plan (JSON or similar)

**Example**:
- Input: "Clean the room"
- LLM Output:
  ```json
  {
    "tasks": [
      {"action": "navigate", "target": "living_room"},
      {"action": "detect", "object": "trash"},
      {"action": "pick", "object": "trash"},
      {"action": "place", "object": "trash", "location": "trash_bin"}
    ]
  }
  ```

**LLM Options**:
- GPT-4 / Claude (cloud APIs, high capability)
- LLaMA 2/3 (local deployment, privacy)
- Specialized robotics LLMs (PaLM-E, RT-2)

### 3. ROS 2 Action Execution

**ROS 2 Actions** execute the planned behaviors:

- **Navigation actions**: Move to locations
- **Manipulation actions**: Pick, place, grasp
- **Perception actions**: Detect objects, scan environment
- **Composite actions**: Sequences of simpler actions

## End-to-End Pipeline Example

### Conceptual Flow

```
User Voice Command
    ↓
[ASR: Whisper]
    ↓
Text: "Clean the room"
    ↓
[LLM: GPT-4]
    ↓
Structured Plan (JSON)
    ↓
[Task Executor]
    ↓
ROS 2 Actions
    ↓
Robot Behavior
```

### Implementation Sketch

**1. Speech Recognition**:
```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("user_audio.wav")
command = result["text"]  # "Clean the room"
```

**2. Task Planning**:
```python
import openai

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "You are a robot task planner..."},
        {"role": "user", "content": command}
    ]
)
plan = parse_json(response.choices[0].message.content)
```

**3. Action Execution**:
```python
import rclpy
from rclpy.action import ActionClient

# Execute navigation action
nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
goal = NavigateToPose.Goal()
goal.pose = plan['tasks'][0]['target']
nav_client.send_goal_async(goal)

# Execute manipulation actions
# ... (similar pattern for pick, place, etc.)
```

## Assessment

To demonstrate your understanding of VLA, complete the following:

### VLA Pipeline Project

Build a minimal VLA system that:

1. **Accepts voice input** (or text input for testing)
2. **Transcribes to text** using Whisper or similar
3. **Generates a plan** using an LLM (GPT-4, Claude, or local model)
4. **Executes a simple action** via ROS 2 (e.g., move forward, turn, or print a message)

**Success Criteria:**
- System processes natural language commands
- LLM generates reasonable task plans
- At least one ROS 2 action executes based on the plan
- Pipeline is documented with example inputs/outputs

**Stretch Goals:**
- Execute multiple actions in sequence
- Handle error cases (unclear commands, failed actions)
- Add feedback loop (robot reports status, system adjusts plan)

## Next Steps

- [Capstone Project: The Autonomous Humanoid](../capstone_workflow/index.md)
- [Hardware & Lab Architecture](../hardware-lab-architecture/index.md)