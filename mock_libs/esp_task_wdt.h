
#pragma once
static void esp_task_wdt_init(int, bool) {}
static void esp_task_wdt_add(void*) {}
static void esp_task_wdt_reset() {}

typedef struct {
    int timeout_ms;
    int idle_core_mask;
    bool trigger_panic;
} esp_task_wdt_config_t;

static void esp_task_wdt_init(esp_task_wdt_config_t*) {}
