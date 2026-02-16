# Modularyzacja projektu Light-Detector

## ✅ Zrealizowana struktura modułów

Projekt został podzielony na następujące moduły:

### 1. **circular_buffer.c/h**

- Bufory cykliczne USART (TX/RX)
- Bufory cykliczne I2C (TX/RX)
- Funkcje: `USART_kbhit()`, `USART_getchar()`, `USART_fsend()`
- Callbacki: `HAL_UART_TxCpltCallback()`, `HAL_UART_RxCpltCallback()`

### 2. **crc8.c/h**

- Obliczanie CRC-8 (polynomial 0x07)
- Konwersje hex: `hex2byte()`, `byte2hex()`, `is_hex_char()`

### 3. **protocol.c/h**

- Parsowanie ramek komunikacyjnych
- Walidacja ramek: `validate_frame()`
- Obsługa komend: `handle_command()`
- Wysyłanie odpowiedzi: `send_response_frame()`
- Przetwarzanie bufora: `process_uart_buffer()`

### 4. **bh1750.c/h**

- Sterownik czujnika BH1750
- Inicjalizacja: `BH1750_Init_Process()`, `BH1750_SetMode()`
- Operacje I2C: `I2C_Transmit_IT()`, `I2C_Receive_IT()`
- Zarządzanie pomiarami: `Measurement_*()` funkcje
- Bufor pomiarów (1000 wpisów)
- Timer aplikacyjny: `App_GetTick()`
- Callbacki: `HAL_TIM_PeriodElapsedCallback()`, callbacki I2C

### 5. **main.c** (zredukowany)

- Funkcje inicjalizacyjne HAL: `MX_*_Init()`
- Konfiguracja zegara: `SystemClock_Config()`
- Funkcja główna: `main()`
- Obsługa błędów: `Error_Handler()`

## 📋 Kroki do uruchomienia

### Jeśli używasz **STM32CubeIDE**:

1. Projekt powinien automatycznie wykryć nowe pliki `.c` w folderze `Core/Src/`
2. Jeśli nie, kliknij prawym przyciskiem na projekt → **Refresh**
3. Zbuduj projekt: **Project → Build Project** (Ctrl+B)

## ⚠️ Ważne uwagi

### Kompatybilność z STM32CubeMX:

- **Funkcje `MX_*_Init()` pozostały w main.c** - są one generowane przez CubeMX
- Możesz bezpiecznie regenerować projekt w CubeMX
- Kod użytkownika w sekcjach `USER CODE BEGIN/END` jest zachowany
- Moduły zewnętrzne (circular_buffer, crc8, protocol, bh1750) nie będą nadpisywane

### Zależności modułów:

```
main.c
  ├── circular_buffer.h
  ├── crc8.h
  ├── protocol.h
  └── bh1750.h

protocol.c
  ├── circular_buffer.h
  ├── crc8.h
  └── bh1750.h

bh1750.c
  └── circular_buffer.h

circular_buffer.c
  └── main.h

crc8.c
  (brak zależności)
```

## 🔧 Struktura plików

```
Core/
├── Inc/
│   ├── main.h
│   ├── circular_buffer.h
│   ├── crc8.h
│   ├── protocol.h
│   └── bh1750.h
└── Src/
    ├── main.c
    ├── circular_buffer.c
    ├── crc8.c
    ├── protocol.c
    └── bh1750.c
```

## ✨ Korzyści z modularyzacji

1. **Czytelność** - każdy moduł ma jasno określone zadanie
2. **Łatwość utrzymania** - zmiany w jednym module nie wpływają na inne
3. **Testowalność** - każdy moduł można testować niezależnie
4. **Reużywalność** - moduły można wykorzystać w innych projektach
5. **Kompatybilność z CubeMX** - funkcje konfiguracyjne pozostają w main.c
