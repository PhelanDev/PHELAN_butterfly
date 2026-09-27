# Plan: FreeRTOS Flight Core Task Separation, Lock-Free Snapshot, UDP Decoupling & WebSocket Queue Optimization

- Title: FreeRTOS Flight Task Separation, Lock-Free Snapshot, UDP Decoupling & WebSocket Queue Optimization
- Status: In Progress
- Priority: High
- Effort: Large
- Tags: FreeRTOS, Lock-Free, WebSocket, UDP, Clean-Code

## Phases
1. Phase 1: Web Client WebSocket Buffer Bloat Optimization (`bufferedAmount > 0`)
2. Phase 2: UDP Socket Decoupling from 300Hz Flight Loop to 50Hz Background Routine
3. Phase 3: Flight Core Task Isolation via FreeRTOS `vTaskDelayUntil` & Lock-Free Atomic Snapshot
4. Phase 4: Clean Code Audit (100% Zero Comments) & Complete Technical CHANGELOG Update
