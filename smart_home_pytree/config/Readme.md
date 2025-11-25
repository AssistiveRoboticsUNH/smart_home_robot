# Protocol YAML Structure Guide

## IMPORTANT NOTES

- **All protocol names must be unique.**
- **Reserve priority `1` for emergency protocols** (e.g., `person_fell`).

---

## Protocol YAML Structure

Each protocol must be defined under the top-level key `protocols`.
Every protocol entry **must** include both a `low_level` and a `high_level` section.

This ensures consistency when parsing and integrating with both the behavior tree (low-level) and the orchestrator or scheduler (high-level).

---

## Structure Rules

### Top-Level Key

- Must start with `protocols:`  
- Each child key represents a *protocol type* (e.g., `TwoReminderProtocol`).

### Protocol Type

Groups similar protocols (for example, all two reminder-type protocols under `TwoReminderProtocol`).

### Protocol Name

- A unique identifier (e.g., `medicine_am`, `medicine_pm`).  
- Used as the key when saving to the blackboard.

### Sections (Required)

#### **low_level** → Parameters and actions executed by the robot or tree nodes.

- Can be empty (`{}`) if no low-level control is needed.

#### **high_level** → Scheduling and activation logic.

- Must include **at least one of**:
  - `requirements.event` — list of event names that trigger the protocol, or
  - `requirements.time` — dictionary with `'from'` and `'to'` fields specifying a time window.

### Uniqueness

Each protocol name must be unique across the file.

---

## Example YAML

```yaml
protocols:
  TwoReminderProtocol:
    medicine_am:
      low_level:
        first_text: "please take your morning medicine"
        second_text: "This is the second reminder to take your morning medicine"
        wait_time_between_reminders: 5
      high_level:
        priority: 2
        requirements: 
          time: {'from': '10:00', 'to': '11:00'}

    medicine_pm:
      low_level:
        first_text: "please take your night medicine"
        second_text: "This is the second reminder to take your night medicine"
        wait_time_between_reminders: 5
      high_level: 
        priority: 2
        requirements: 
          event: 
            - {'topic': '/medicine_pm_req_1', 'value': True}
            - {'topic': '/medicine_pm_req_2', 'value': True}
          time: {'from': '22:00', 'to': '23:00'}
```

---