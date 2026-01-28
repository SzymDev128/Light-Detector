# Przewodnik implementacji czujnika BH1750 na STM32F446RE

## 📋 Lista wymaganych elementów

### 1. Sprzęt
- ✅ **Czujnik BH1750** (moduł z czujnikiem światła)
- ✅ **Rezystory podciągające** 4.7 kΩ (2 sztuki) - jeśli moduł nie ma wbudowanych
- ✅ **Przewody połączeniowe** (jumper wires)
- ✅ **Płytka STM32F446RE** (Nucleo lub custom)

### 2. Podłączenie do STM32F446RE

#### Opcja A: I2C1 (zalecane)
```
BH1750          STM32F446RE
─────────────────────────────
VCC      →      3.3V (lub 5V)
GND      →      GND
SDA      →      PB7  (I2C1_SDA)
SCL      →      PB6  (I2C1_SCL)
ADDR     →      GND  (adres 0x23) lub VCC (adres 0x5C)
```

#### Opcja B: I2C2
```
BH1750          STM32F446RE
─────────────────────────────
VCC      →      3.3V (lub 5V)
GND      →      GND
SDA      →      PB11 (I2C2_SDA)
SCL      →      PB10 (I2C2_SCL)
ADDR     →      GND  (adres 0x23) lub VCC (adres 0x5C)
```

#### Opcja C: I2C1 alternatywne piny
```
BH1750          STM32F446RE
─────────────────────────────
VCC      →      3.3V (lub 5V)
GND      →      GND
SDA      →      PB9  (I2C1_SDA alternatywny)
SCL      →      PB8  (I2C1_SCL alternatywny)
ADDR     →      GND  (adres 0x23) lub VCC (adres 0x5C)
```

**Uwaga:** Jeśli moduł BH1750 nie ma wbudowanych rezystorów podciągających, dodaj:
- Rezystor 4.7 kΩ między SDA a VCC
- Rezystor 4.7 kΩ między SCL a VCC

---

## 🔧 Konfiguracja w STM32CubeIDE

### Krok 1: Otwórz plik .ioc
- Otwórz plik `Banaszek_Project.ioc` w STM32CubeIDE

### Krok 2: Skonfiguruj I2C1
1. W zakładce **Pinout & Configuration**:
   - Znajdź **I2C1** w liście peryferiów
   - Kliknij na **I2C1**
   - W **Mode** wybierz:
     - ✅ **I2C** (nie I2C SMBus)
   
2. W **Configuration** → **I2C1**:
   - **I2C Speed Frequency:** 100000 Hz (100 kHz - standardowa prędkość)
   - **Clock Speed:** 100000 Hz
   - **Duty Cycle:** 2 (dla 100 kHz nie ma znaczenia)
   - **General Call Address Detection:** Disable
   - **No Stretch Mode:** Disable

3. **Przypisz piny:**
   - Kliknij na pin **PB6** → wybierz **I2C1_SCL**
   - Kliknij na pin **PB7** → wybierz **I2C1_SDA**

### Krok 3: Skonfiguruj NVIC (opcjonalnie - dla przerwań)
1. W **System Core** → **NVIC**:
   - Włącz **I2C1 event interrupt** (opcjonalnie)
   - Włącz **I2C1 error interrupt** (opcjonalnie)

### Krok 4: Wygeneruj kod
1. Kliknij **Project** → **Generate Code** (lub Ctrl+Alt+G)
2. STM32CubeIDE wygeneruje:
   - `MX_I2C1_Init()` w `main.c`
   - `hi2c1` handle w `main.c`
   - Pliki HAL dla I2C

---

## 📁 Struktura plików do dodania

### Pliki do utworzenia:
```
Core/
├── Inc/
│   └── bh1750.h          ← Nowy plik nagłówkowy
└── Src/
    └── bh1750.c          ← Nowy plik źródłowy
```

---

## 💻 Kod do implementacji

### 1. Plik: `Core/Inc/bh1750.h`
```c
#ifndef BH1750_H
#define BH1750_H

#include "stm32f4xx_hal.h"

// Adres I2C czujnika BH1750
#define BH1750_ADDR_LOW    0x23  // ADDR pin do GND
#define BH1750_ADDR_HIGH   0x5C  // ADDR pin do VCC

// Komendy BH1750
#define BH1750_POWER_DOWN          0x00
#define BH1750_POWER_ON             0x01
#define BH1750_RESET                0x07
#define BH1750_CONTINUOUS_H_RES_MODE  0x10  // Rozdzielczość 1 lx, czas 120ms
#define BH1750_CONTINUOUS_H_RES_MODE2 0x11  // Rozdzielczość 0.5 lx, czas 120ms
#define BH1750_CONTINUOUS_L_RES_MODE  0x13  // Rozdzielczość 4 lx, czas 16ms
#define BH1750_ONE_TIME_H_RES_MODE    0x20  // Rozdzielczość 1 lx, czas 120ms
#define BH1750_ONE_TIME_H_RES_MODE2   0x21  // Rozdzielczość 0.5 lx, czas 120ms
#define BH1750_ONE_TIME_L_RES_MODE    0x23  // Rozdzielczość 4 lx, czas 16ms

// Funkcje
HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c, uint8_t address);
HAL_StatusTypeDef BH1750_ReadLight(I2C_HandleTypeDef *hi2c, uint8_t address, float *lux);
HAL_StatusTypeDef BH1750_SetMode(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t mode);
HAL_StatusTypeDef BH1750_Reset(I2C_HandleTypeDef *hi2c, uint8_t address);

#endif /* BH1750_H */
```

### 2. Plik: `Core/Src/bh1750.c`
```c
#include "bh1750.h"
#include <math.h>

/**
 * @brief Inicjalizacja czujnika BH1750
 * @param hi2c: Wskaźnik do struktury I2C_HandleTypeDef
 * @param address: Adres I2C czujnika (BH1750_ADDR_LOW lub BH1750_ADDR_HIGH)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_Init(I2C_HandleTypeDef *hi2c, uint8_t address) {
    HAL_StatusTypeDef status;
    
    // Włączenie czujnika
    status = BH1750_SetMode(hi2c, address, BH1750_POWER_ON);
    if (status != HAL_OK) return status;
    
    HAL_Delay(10); // Krótkie opóźnienie po włączeniu
    
    // Ustawienie trybu ciągłego pomiaru, wysoka rozdzielczość
    status = BH1750_SetMode(hi2c, address, BH1750_CONTINUOUS_H_RES_MODE);
    if (status != HAL_OK) return status;
    
    HAL_Delay(120); // Czas na pierwszy pomiar (120ms dla H_RES_MODE)
    
    return HAL_OK;
}

/**
 * @brief Ustawienie trybu pracy czujnika
 * @param hi2c: Wskaźnik do struktury I2C_HandleTypeDef
 * @param address: Adres I2C czujnika
 * @param mode: Tryb pracy (jedna z komend BH1750)
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_SetMode(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t mode) {
    return HAL_I2C_Master_Transmit(hi2c, address << 1, &mode, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reset czujnika BH1750
 * @param hi2c: Wskaźnik do struktury I2C_HandleTypeDef
 * @param address: Adres I2C czujnika
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_Reset(I2C_HandleTypeDef *hi2c, uint8_t address) {
    return BH1750_SetMode(hi2c, address, BH1750_RESET);
}

/**
 * @brief Odczyt wartości natężenia światła w luksach
 * @param hi2c: Wskaźnik do struktury I2C_HandleTypeDef
 * @param address: Adres I2C czujnika
 * @param lux: Wskaźnik do zmiennej, gdzie zostanie zapisana wartość w luksach
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef BH1750_ReadLight(I2C_HandleTypeDef *hi2c, uint8_t address, float *lux) {
    uint8_t data[2];
    HAL_StatusTypeDef status;
    uint16_t raw_value;
    
    // Odczyt 2 bajtów danych
    status = HAL_I2C_Master_Receive(hi2c, (address << 1) | 0x01, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }
    
    // Konwersja bajtów na 16-bitową wartość (big-endian)
    raw_value = (data[0] << 8) | data[1];
    
    // Przeliczenie na luksy: wartość / 1.2 (dla trybu H_RES_MODE)
    *lux = raw_value / 1.2f;
    
    return HAL_OK;
}
```

### 3. Modyfikacje w `main.c`

#### a) Dodaj include:
```c
/* USER CODE BEGIN Includes */
#include "bh1750.h"
/* USER CODE END Includes */
```

#### b) Dodaj zmienną handle I2C (jeśli nie została wygenerowana):
```c
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;  // Dodaj tę linię
```

#### c) Przykład użycia w main():
```c
int main(void) {
    // ... istniejący kod inicjalizacji ...
    
    /* USER CODE BEGIN 2 */
    // Inicjalizacja BH1750
    if (BH1750_Init(&hi2c1, BH1750_ADDR_LOW) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE END 2 */
    
    while (1) {
        /* USER CODE BEGIN 3 */
        float light_level;
        
        // Odczyt natężenia światła
        if (BH1750_ReadLight(&hi2c1, BH1750_ADDR_LOW, &light_level) == HAL_OK) {
            USART_fsend("Light: %.2f lx\r\n", light_level);
        } else {
            USART_fsend("Error reading BH1750\r\n");
        }
        
        HAL_Delay(1000); // Odczyt co sekundę
        /* USER CODE END 3 */
    }
}
```

---

## ✅ Checklist implementacji

### Konfiguracja sprzętowa:
- [ ] Podłączony czujnik BH1750 do STM32F446RE
- [ ] Rezystory podciągające 4.7kΩ (jeśli potrzebne)
- [ ] Zasilanie 3.3V lub 5V
- [ ] Połączenie masy (GND)

### Konfiguracja oprogramowania:
- [ ] Skonfigurowany I2C1 w STM32CubeIDE (.ioc)
- [ ] Przypisane piny PB6 (SCL) i PB7 (SDA)
- [ ] Wygenerowany kod z STM32CubeIDE
- [ ] Utworzony plik `bh1750.h`
- [ ] Utworzony plik `bh1750.c`
- [ ] Dodany include `bh1750.h` w `main.c`
- [ ] Dodana inicjalizacja BH1750 w `main()`
- [ ] Dodany kod odczytu w pętli głównej

### Testowanie:
- [ ] Kompilacja bez błędów
- [ ] Wgranie programu do mikrokontrolera
- [ ] Sprawdzenie komunikacji I2C (odczyt wartości)
- [ ] Weryfikacja zmiany wartości przy zmianie oświetlenia

---

## 🔍 Rozwiązywanie problemów

### Problem: Brak komunikacji z czujnikiem
- ✅ Sprawdź połączenia SDA i SCL
- ✅ Sprawdź zasilanie (3.3V lub 5V)
- ✅ Sprawdź rezystory podciągające
- ✅ Sprawdź adres I2C (0x23 lub 0x5C)
- ✅ Użyj oscyloskopu/logic analyzer do sprawdzenia sygnałów I2C

### Problem: Błędne odczyty
- ✅ Sprawdź czas opóźnienia po inicjalizacji (120ms dla H_RES_MODE)
- ✅ Sprawdź czy czujnik jest w odpowiednim trybie
- ✅ Sprawdź czy nie ma zakłóceń elektromagnetycznych

### Problem: Błąd kompilacji - brak math.h
- ✅ Dodaj `#include <math.h>` w `bh1750.c` (już jest w przykładzie)

---

## 📚 Dodatkowe informacje

### Adresy I2C:
- **0x23** - gdy ADDR pin podłączony do GND (domyślny)
- **0x5C** - gdy ADDR pin podłączony do VCC

### Tryby pomiaru:
- **H_RES_MODE** (0x10): 1 lx, 120ms - zalecany
- **H_RES_MODE2** (0x11): 0.5 lx, 120ms - wyższa rozdzielczość
- **L_RES_MODE** (0x13): 4 lx, 16ms - szybszy pomiar

### Zakres pomiarowy:
- **0 - 65535 lx** (teoretycznie)
- **Praktycznie:** 1-65535 lx dla H_RES_MODE

---

## 🎯 Następne kroki

Po zaimplementowaniu podstawowej funkcjonalności możesz:
1. Dodać obsługę różnych trybów pomiaru
2. Dodać filtrację odczytów (średnia ruchoma)
3. Zintegrować z protokołem ramkowym (komendy GET_LIGHT, SET_INTERVAL)
4. Dodać obsługę przerwań I2C
5. Dodać obsługę wielu czujników (różne adresy)

