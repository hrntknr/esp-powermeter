# esp-env

> esp32c3 で bme680 の値を読み込み、mqtt に publish する。

## device

- esp32c3 (Seeed Studio XIAO ESP32C3)
- bme680 (https://akizukidenshi.com/catalog/g/gK-14469/)

## override config

- pm
  - Component config → Power Management → Support for power management
  - Component config → FreeRTOS → Kernel → configUSE_TICKLESS_IDLE
- ipv6
  - Component config → LWIP → Enable IPv6 → Enable IPV6 stateless address autoconfiguration (SLAAC)
- flash
  - Serial flasher config → Flash size (4M)
