# blacknode-skills

`blacknode-skills` contains reusable task-level robot behaviors composed from stable Blacknode robot, perception, motion, and agent capabilities.

## Components

| Component | Purpose |
|---|---|
| `follow` | Visual following and leader/follower behavior; includes the current executable nodes and ROS 2 templates |
| `pick-place` | Pick-and-place capability contract |
| `delivery` | Item-delivery mission contract |
| `docking` | Docking and charging contract |
| `inspection` | Structured inspection mission contract |

All components are opt-in. `follow-person` remains a deprecated alias for `follow` until version 1.0.0.

## Follow workflows

The ROS 2 adapter includes templates for local-camera following, ROS-camera following, SO-ARM leader/follower setups, and independently deployable leader and follower graphs. Start the publisher or leader first, then the subscriber or follower.

The follower begins disarmed. Motion starts only after explicit authorization and stops when joint state or source detections become stale. Skill nodes request motion through `blacknode-motion`; they do not call physical drivers directly.

## Install and verify

```powershell
blacknode packages install https://github.com/temiroff/blacknode-skills.git
python -m pytest packages/blacknode-skills/tests
```

See [AGENTS.md](AGENTS.md) for capability and motion-safety rules.
