
### JSON Payload
The JSON payload should be in the following format

```json
{
  "header": "system_mode", // mapping, navigation, emergency, config
  "command": "action_type", // init, start, pause, resume, stop, reset
  "config": {
    "operation_mode": "single|multi|loop",
    "environment": {
      "use_simulator": true,
      "enable_rviz": true,
    },
    "resource": {
      "map_file": "/path/to/map.yaml",
      "default_save_path": "/home/ubuntu/ros_maps",
      "map_name": "map_2024"
    }
  },
  "data": {
    "targets": [
      {
        "x": 1.5,
        "y": 2.3,
        "theta": 0.0
      }
    ],
    "waypoints": [
        {
            "x": 1.5,
            "y": 2.3,
        }
    ]
  },
  "metadata": {
    "session_id": "uuid4",
    "timestamp": "ISO8601",
  }
}
```

### Example Usage
1. Mapping
To start, pause, and stop mapping
```bash
// Start mapping
const robotCommand = createMSDSystemData({
            header: "mapping",
            command: "start"
        });
mqtt_client.publish(`${username}/${unit_name}/system_command`, JSON.stringify(robotCommand));

// Pause mapping
const robotCommand = createMSDSystemData({
            header: "mapping",
            command: "pause"
        });
mqtt_client.publish(`${username}/${unit_name}/system_command`, JSON.stringify(robotCommand));

// Stop mapping
const robotCommand = createMSDSystemData({
    header: "mapping",
    command: "stop",
    config: {
        resource: {
            map_name: map_name,
            default_save_path: map_path,
        },
    },
});
mqtt_client.publish(`${username}/${unit_name}/system_command`, JSON.stringify(robotCommand));
```

2. Navigation
To Initialize navigation.
```bash
const robotCommand = createMSDSystemData({
    header: "navigation",
    command: "init",
    config: {
        resource: {
            operation_mode: "single",
            map_file: `${map_path}/${yaml_file}`,
        }
    }
});
mqtt_client.publish(`${username}/${unit_name}/system_command`, JSON.stringify(robotCommand));
```

