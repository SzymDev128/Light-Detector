# Proces implementacji czujnika BH1750 - opis działania z kodem

## 1. Konfiguracja początkowa systemu

### 1.1 Struktura danych i definicje

System definiuje komendy BH1750 oraz struktury danych służące do zarządzania stanem czujnika i komunikacją I2C:

```c
// Adresy I2C czujnika BH1750 (7-bit)
#define BH1750_ADDR_LOW    0x23  // ADDR pin do GND
#define BH1750_ADDR_HIGH   0x5C  // ADDR pin do VCC

// Komendy czujnika BH1750
#define BH1750_CONTINUOUS_HIGH_RES_MODE    0x10  // Ciągły wysokiej rozdzielczości (1lx, 120ms)
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2   0x11  // Ciągły wysokiej rozdzielczości 2 (0.5lx, 120ms)
#define BH1750_CONTINUOUS_LOW_RES_MODE      0x13  // Ciągły niskiej rozdzielczości (4lx, 16ms)

// Zmienne globalne
static uint8_t bh1750_current_mode = BH1750_CONTINUOUS_HIGH_RES_MODE;
static uint8_t bh1750_initialized = 0;
static uint8_t bh1750_addr = BH1750_ADDR_LOW;

// Stany maszyny stanów inicjalizacji
typedef enum {
    BH1750_INIT_IDLE = 0,
    BH1750_INIT_PWRON,
    BH1750_INIT_RESET,
    BH1750_INIT_MODE,
    BH1750_INIT_DONE
} bh1750_init_state_t;

static bh1750_init_state_t bh1750_init_state = BH1750_INIT_IDLE;
```

System przechowuje aktualny tryb pracy czujnika w zmiennej `bh1750_current_mode`, flagę inicjalizacji w `bh1750_initialized` oraz adres I2C w `bh1750_addr`. Maszyna stanów używa typu wyliczeniowego `bh1750_init_state_t` do śledzenia postępu inicjalizacji.

### 1.2 Struktura operacji I2C

Nieblokująca komunikacja I2C wykorzystuje strukturę `i2c_operation_t`, która śledzi parametry bieżącej operacji:

```c
// Struktura dla operacji I2C
typedef struct {
    uint8_t address;     // Adres urządzenia I2C (7-bit)
    uint8_t *data;       // Wskaźnik na bufor danych
    uint16_t len;        // Długość danych
    uint8_t operation;   // 0 = TX (transmisja), 1 = RX (odbiór)
    uint8_t pending;     // Flaga: 1 = operacja w toku, 0 = wolne
} i2c_operation_t;

static i2c_operation_t i2c_op = {0};
```

Flaga `pending` działa jak mutex - zapobiega rozpoczęciu nowej operacji I2C gdy poprzednia jest jeszcze aktywna.

### 1.3 Timer systemowy dla nieblokujących opóźnień

System wykorzystuje timer TIM2 do generowania znaczników czasowych co 1ms:

```c
// Timer aplikacyjny oparty o TIM2 (1ms)
static __IO uint32_t app_tick = 0;

static uint32_t App_GetTick(void) {
    return app_tick;
}

// Callback przerwania timera (wywoływany co 1ms)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        app_tick++;
    }
}
```

Funkcja `App_GetTick()` zwraca liczbę milisekund od startu systemu, pozwalając na nieblokujące odmierzanie czasu.

### 1.4 Struktura śledzenia czasu oczekiwania BH1750

Czujnik BH1750 wymaga określonego czasu na wykonanie pomiaru (120ms lub 16ms w zależności od trybu). System śledzi ten czas bez blokowania procesora:

```c
// Struktura dla śledzenia czasu oczekiwania BH1750
typedef struct {
    uint32_t start_time;  // Czas rozpoczęcia (App_GetTick())
    uint32_t wait_time;   // Wymagany czas oczekiwania w ms
    uint8_t active;       // 1 = aktywne oczekiwanie, 0 = zakończone
} bh1750_timing_t;

static bh1750_timing_t bh1750_timing = {0};

void BH1750_StartTiming(uint32_t wait_time_ms) {
    bh1750_timing.start_time = App_GetTick();
    bh1750_timing.wait_time = wait_time_ms;
    bh1750_timing.active = 1;
}

uint8_t BH1750_IsTimingReady(void) {
    if (!bh1750_timing.active) {
        return 1; // Nie ma aktywnego oczekiwania
    }

    uint32_t current_time = App_GetTick();
    if ((current_time - bh1750_timing.start_time) >= bh1750_timing.wait_time) {
        bh1750_timing.active = 0;
        return 1; // Czas minął
    }

    return 0; // Jeszcze czeka
}
```

---

## 2. Inicjalizacja czujnika przez maszynę stanów

### 2.1 Nieblokująca transmisja I2C

Funkcja `I2C_Transmit_IT()` inicjuje transmisję przez przerwania:

```c
HAL_StatusTypeDef I2C_Transmit_IT(uint8_t address, uint8_t *data, uint16_t len) {
    if (i2c_op.pending) {
        return HAL_BUSY; // Magistrala zajęta
    }

    if (len > I2C_TXBUF_LEN) {
        return HAL_ERROR; // Zbyt duże dane
    }

    // Sprawdź stan HAL I2C
    HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hi2c1);
    if (state != HAL_I2C_STATE_READY) {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        return HAL_ERROR;
    }

    // Kopiowanie danych do bufora
    for (uint16_t i = 0; i < len; i++) {
        I2C_TxBuf[i] = data[i];
    }

    i2c_op.address = address;
    i2c_op.data = I2C_TxBuf;
    i2c_op.len = len;
    i2c_op.operation = 0; // TX
    i2c_op.pending = 1;   // Zaznacz zajętość

    // Rozpoczęcie transmisji przez przerwania (adres przesunięty o 1 bit w lewo)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&hi2c1, address << 1, I2C_TxBuf, len);

    if (status != HAL_OK) {
        i2c_op.pending = 0; // Zwolnij w przypadku błędu
    }

    return status;
}
```

Funkcja ustawia flagę `i2c_op.pending = 1`, uruchamia transmisję i natychmiast zwraca. Rzeczywista komunikacja odbywa się w tle.

### 2.2 Callback zakończenia transmisji I2C

Gdy transmisja się zakończy, biblioteka HAL wywołuje przerwanie:

```c
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        i2c_op.pending = 0; // Zwolnij magistralę
        // Operacja zakończona - można wykonać kolejną
    }
}
```

Callback wykonuje się w kontekście przerwania (kilka mikrosekund) i tylko zeruje flagę, sygnalizując że magistrala jest wolna.

### 2.3 Maszyna stanów inicjalizacji BH1750

Funkcja `BH1750_Init_Process()` jest wywoływana cyklicznie w pętli głównej i realizuje inicjalizację krok po kroku:

```c
void BH1750_Init_Process(void) {
    static uint32_t last_retry_time = 0;

    if (bh1750_initialized) {
        return; // Już zainicjalizowany
    }

    // Inicjuj tylko gdy automatyczny pomiar jest włączony
    if (!measurement_auto.enabled) {
        return;
    }

    // Jeśli był błąd I2C, zresetuj stan inicjalizacji
    if (I2C_Error) {
        I2C_Error = 0;
        bh1750_initialized = 0;
        bh1750_init_state = BH1750_INIT_IDLE;
        return;
    }

    // Nie rozpoczynaj kolejnego kroku, jeśli I2C jest zajęte
    if (i2c_op.pending) {
        return;
    }

    // Throttling - nie próbuj zbyt często (1 sekunda między próbami)
    if (bh1750_init_state == BH1750_INIT_IDLE) {
        uint32_t now = App_GetTick();
        if (now - last_retry_time < 1000) {
            return; // Czekaj
        }
        last_retry_time = now;
    }

    switch (bh1750_init_state) {
        case BH1750_INIT_IDLE: {
            // Krok 1: Wysłanie komendy POWER_ON (0x01)
            uint8_t cmd = 0x01;
            HAL_StatusTypeDef status = I2C_Transmit_IT(bh1750_addr, &cmd, 1);
            if (status == HAL_OK) {
                bh1750_init_state = BH1750_INIT_PWRON;
            }
            break;
        }
        case BH1750_INIT_PWRON: {
            // Krok 2: Wysłanie komendy RESET (0x07)
            uint8_t cmd = 0x07;
            if (I2C_Transmit_IT(bh1750_addr, &cmd, 1) == HAL_OK) {
                bh1750_init_state = BH1750_INIT_RESET;
            }
            break;
        }
        case BH1750_INIT_RESET: {
            // Krok 3: Ustawienie trybu pomiaru
            if (BH1750_SetMode(bh1750_current_mode) == HAL_OK) {
                bh1750_init_state = BH1750_INIT_MODE;
            }
            break;
        }
        case BH1750_INIT_MODE:
            // Krok 4: Czekanie na zakończenie transmisji trybu
            if (!i2c_op.pending) {
                bh1750_initialized = 1;
                bh1750_init_state = BH1750_INIT_DONE;
            }
            break;
        default:
            break;
    }
}
```

Każdy przypadek `switch` wykonuje jeden krok i kończy działanie. W kolejnym przebiegu pętli funkcja kontynuuje od następnego stanu.

### 2.4 Ustawienie trybu pomiaru

Funkcja `BH1750_SetMode()` wysyła komendę trybu i uruchamia licznik czasu oczekiwania:

```c
HAL_StatusTypeDef BH1750_SetMode(uint8_t mode) {
    HAL_StatusTypeDef status;
    uint8_t addr = bh1750_addr;

    // Wysyłanie komendy trybu do czujnika przez przerwania
    status = I2C_Transmit_IT(addr, &mode, 1);

    if (status == HAL_OK) {
        bh1750_current_mode = mode;

        // Rozpoczęcie śledzenia czasu oczekiwania
        // Opóźnienie w zależności od trybu (120ms dla high res, 16ms dla low res)
        if (mode == BH1750_CONTINUOUS_LOW_RES_MODE || mode == BH1750_ONETIME_LOW_RES_MODE) {
            BH1750_StartTiming(16);
        } else {
            BH1750_StartTiming(120);
        }
    }

    return status;
}
```

Funkcja uruchamia nieblokujące odmierzanie 120ms (lub 16ms), aby system wiedział kiedy czujnik będzie gotowy do pierwszego odczytu.

### 2.5 Obsługa błędów I2C

Gdy wystąpi błąd komunikacji, biblioteka HAL wywołuje callback:

```c
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        I2C_Error = 1;
        i2c_op.pending = 0;
        I2C_BusReset_Pending = 1; // Uruchom procedurę recovery magistrali
    }
}
```

Flaga `I2C_Error` powoduje reset maszyny stanów w `BH1750_Init_Process()`, co pozwala na automatyczną ponowną próbę inicjalizacji.

---

## 3. Automatyczne odczyty pomiarów

### 3.1 Struktura automatycznego odczytu

System wykorzystuje strukturę `measurement_auto_t` do zarządzania automatycznymi pomiarami:

```c
// Struktura dla automatycznego odczytu pomiarów
typedef struct {
    uint32_t interval_ms;      // Interwał pomiarowy w ms
    uint32_t last_measurement;  // Czas ostatniego pomiaru (App_GetTick())
    uint8_t enabled;            // Czy automatyczny odczyt jest włączony
} measurement_auto_t;

static measurement_auto_t measurement_auto = {
    .interval_ms = 1000,        // Domyślnie 1 sekunda
    .last_measurement = 0,
    .enabled = 0                // Wyłączone domyślnie
};

void Measurement_EnableAutoRead(uint8_t enable) {
    measurement_auto.enabled = enable;
    if (enable) {
        measurement_auto.last_measurement = App_GetTick();
    }
}
```

Gdy system odbierze komendę START (10), wywołuje `Measurement_EnableAutoRead(1)`, uruchamiając automatyczne pomiary.

### 3.2 Proces automatycznego odczytu

Funkcja `Measurement_AutoRead_Process()` jest wywoływana w każdym przebiegu pętli głównej:

```c
void Measurement_AutoRead_Process(void) {
    if (!measurement_auto.enabled) {
        return; // Automatyczny odczyt wyłączony
    }
    if (!bh1750_initialized) {
        return; // Czekaj na zakończenie inicjalizacji BH1750
    }

    // Sprawdzenie czy dane z odczytu są gotowe i zapisanie ich
    if (bh1750_read_ready) {
        // Zapis pomiaru do bufora
        Measurement_AddEntry(bh1750_last_lux);
        bh1750_read_ready = 0; // Wyzeruj flagę po zapisaniu
    }

    // Sprawdzenie czy minął interwał
    uint32_t current_time = App_GetTick();
    if ((current_time - measurement_auto.last_measurement) >= measurement_auto.interval_ms) {
        measurement_auto.last_measurement = current_time; // Zaktualizuj czas

        // Sprawdzenie czy I2C nie jest zajęty
        if (!i2c_op.pending) {
            // Rozpoczęcie nowego odczytu z czujnika
            bh1750_read_ready = 0; // Wyzeruj flagę przed rozpoczęciem odczytu
            HAL_StatusTypeDef status = BH1750_ReadLight(&bh1750_last_lux);

            // Jeśli I2C zawiódł, dodaj wartość 0 jako wskaźnik błędu
            if (status != HAL_OK) {
                Measurement_AddEntry(0.0f); // Błąd I2C
            }
        }
    }
}
```

Funkcja najpierw sprawdza czy poprzedni odczyt się zakończył (flaga `bh1750_read_ready`) i jeśli tak, zapisuje wynik do bufora. Następnie sprawdza czy minął interwał i jeśli magistrala I2C jest wolna, uruchamia nowy odczyt.

### 3.3 Nieblokujący odczyt z czujnika

Funkcja `BH1750_ReadLight()` inicjuje odczyt przez przerwania:

```c
// Bufory dla odczytu BH1750
static uint8_t bh1750_read_buffer[2] = {0};
static float bh1750_last_lux = 0.0f;
static uint8_t bh1750_read_ready = 0;

HAL_StatusTypeDef BH1750_ReadLight(float *lux) {
    uint8_t addr = bh1750_addr;

    // Sprawdzenie czy operacja I2C nie jest w toku
    if (i2c_op.pending) {
        return HAL_BUSY;
    }

    // Sprawdzenie czy ostatni odczyt jest gotowy
    if (bh1750_read_ready) {
        *lux = bh1750_last_lux;
        bh1750_read_ready = 0;
        return HAL_OK;
    }

    // Rozpoczęcie odczytu przez przerwania
    HAL_StatusTypeDef status = I2C_Receive_IT(addr, bh1750_read_buffer, 2);

    return status; // Wartość zostanie przetworzona w callbacku
}
```

### 3.4 Nieblokujący odbiór I2C

Funkcja `I2C_Receive_IT()` uruchamia odbiór danych:

```c
HAL_StatusTypeDef I2C_Receive_IT(uint8_t address, uint8_t *data, uint16_t len) {
    if (i2c_op.pending) {
        return HAL_BUSY; // Operacja w toku
    }

    if (len > I2C_RXBUF_LEN) {
        return HAL_ERROR; // Zbyt duże dane
    }

    i2c_op.address = address;
    i2c_op.data = data;          // Wskaźnik na bh1750_read_buffer
    i2c_op.len = len;            // 2 bajty
    i2c_op.operation = 1;        // RX (odbiór)
    i2c_op.pending = 1;          // Zajmij magistralę

    // Rozpoczęcie odbioru przez przerwania
    // (address << 1) | 0x01 - adres z bitem odczytu
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_IT(&hi2c1, (address << 1) | 0x01, I2C_RxBuf, len);

    if (status != HAL_OK) {
        i2c_op.pending = 0; // Zwolnij w przypadku błędu
    }

    return status;
}
```

Mikrokontroler wysyła na magistralę I2C adres czujnika (0x23 przesunięty o 1 bit = 0x46) z ustawionym bitem odczytu (0x01), co daje 0x47. Czujnik odpowiada ACK i zaczyna przesyłać 2 bajty danych.

### 3.5 Callback odbioru danych I2C

Gdy I2C odbierze oba bajty, wywołuje się callback:

```c
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        // Kopiowanie danych z bufora do miejsca docelowego
        if (i2c_op.operation == 1 && i2c_op.data != NULL) {
            for (uint16_t i = 0; i < i2c_op.len; i++) {
                i2c_op.data[i] = I2C_RxBuf[i];
            }

            // Jeśli to odczyt BH1750 (2 bajty)
            if (i2c_op.len == 2 && i2c_op.data == bh1750_read_buffer) {
                // Konwersja bajtów na 16-bitową wartość (big-endian)
                uint16_t raw_value = (bh1750_read_buffer[0] << 8) | bh1750_read_buffer[1];

                // Przeliczenie na luksy: wartość / 1.2 (dla trybu H_RES_MODE)
                bh1750_last_lux = raw_value / 1.2f;

                // Ustaw flagę gotowości
                bh1750_read_ready = 1;
            }
        }
        i2c_op.pending = 0; // Zwolnij magistralę
    }
}
```

Callback wykrywa że to odczyt z BH1750 (2 bajty), łączy oba bajty w wartość 16-bitową, dzieli przez 1.2 i ustawia flagę gotowości. Cała operacja trwa kilka mikrosekund.

### 3.6 Zapis pomiaru do bufora

W następnym przebiegu pętli głównej, `Measurement_AutoRead_Process()` wykrywa `bh1750_read_ready = 1` i zapisuje pomiar:

```c
#define MEASUREMENT_BUFFER_SIZE 1000
static measurement_entry_t measurement_buffer[MEASUREMENT_BUFFER_SIZE];
static uint16_t measurement_write_index = 0;  // Indeks do zapisu (head)
static uint16_t measurement_count = 0;        // Liczba zapisanych pomiarów

typedef struct measurement_entry_t {
    float lux;           // Wartość natężenia światła w luksach
    uint32_t timestamp;  // Timestamp pomiaru (App_GetTick())
} measurement_entry_t;

void Measurement_AddEntry(float lux) {
    // Zapisanie pomiaru do bufora
    measurement_buffer[measurement_write_index].lux = lux;
    measurement_buffer[measurement_write_index].timestamp = App_GetTick();

    // Aktualizacja indeksu (bufor cykliczny)
    measurement_write_index++;
    if (measurement_write_index >= MEASUREMENT_BUFFER_SIZE) {
        measurement_write_index = 0; // Zawinięcie bufora
    }

    // Aktualizacja licznika (max 1000)
    if (measurement_count < MEASUREMENT_BUFFER_SIZE) {
        measurement_count++;
    }
}
```

Bufor cykliczny może przechować do 1000 pomiarów. Po zapełnieniu najstarsze pomiary są nadpisywane.

---

## 4. Integracja z protokołem komunikacji PC-STM32

### 4.1 Parsowanie ramek protokołu

System odbiera znaki przez UART do bufora kołowego w przerwaniu `HAL_UART_RxCpltCallback()`. Funkcja `process_uart_buffer()` w pętli głównej parsuje ramki protokołu:

```c
// Maszyna stanów parsera ramek
typedef enum {
    ST_IDLE = 0,
    ST_COLLECT
} frame_state_t;

frame_state_t st = ST_IDLE;
char frame[300];
uint16_t pos = 0;

void process_uart_buffer(void) {
    while(USART_kbhit()) {
        char c = USART_getchar();

        switch(st) {
        case ST_IDLE:
            if(c == '&') {
                pos = 0;
                frame[pos++] = c;
                st = ST_COLLECT;
            }
            break;

        case ST_COLLECT:
            // Nowy start resetuje ramkę
            if(c == '&') {
                pos = 0;
                frame[pos++] = c;
                break;
            }

            // Zapisuj znak
            if(pos < sizeof(frame)-1)
                frame[pos++] = c;

            // Koniec ramki
            if(c == '*') {
                frame[pos] = 0;   // string end
                validate_frame(frame, pos);
                st = ST_IDLE;
            }
            break;
        }
    }
}
```

Parser czeka na znak `&` (początek ramki), zbiera znaki do bufora i kończy gdy wykryje `*` (koniec ramki). Następnie wywołuje `validate_frame()`, która sprawdza strukturę i CRC.

### 4.2 Obsługa komendy START (10)

Funkcja `handle_command()` rozpoznaje kod komendy i wykonuje odpowiednią akcję:

```c
void handle_command(char *cmd, const char *src_addr, const char *dst_addr, const char *id) {
    const char *device_addr = dst_addr;

    // Wyodrębnienie kodu komendy (pierwsze 2 cyfry)
    uint8_t cmd_code = (cmd[0] - '0') * 10 + (cmd[1] - '0');
    const char *params = (strlen(cmd) > 2) ? &cmd[2] : "";

    // 10 - START
    if (cmd_code == 10) {
        Measurement_EnableAutoRead(1);
        send_response_frame(device_addr, src_addr, id, "00"); // OK
    }
    // ... inne komendy
}
```

Komenda START (kod 10) wywołuje `Measurement_EnableAutoRead(1)`, która ustawia flagę `measurement_auto.enabled = 1`. To powoduje że:

1. Funkcja `BH1750_Init_Process()` rozpocznie inicjalizację czujnika
2. Po inicjalizacji `Measurement_AutoRead_Process()` rozpocznie automatyczne pomiary

Następnie system wysyła ramkę odpowiedzi z kodem "00" (sukces).

### 4.3 Wysyłanie ramki odpowiedzi

Funkcja `send_response_frame()` buduje ramkę zgodną z protokołem:

```c
void send_response_frame(const char *src_addr, const char *dst_addr, const char *id, const char *data) {
    char frame[271];
    uint16_t pos = 0;
    uint8_t crc_buf[300];
    uint16_t crc_pos = 0;

    // Budowanie ramki: & SRC DST ID LEN DATA CRC *
    frame[pos++] = '&';

    // SRC (3 znaki) - adres nadawcy odpowiedzi
    memcpy(&frame[pos], src_addr, 3);
    pos += 3;
    memcpy(&crc_buf[crc_pos], src_addr, 3);
    crc_pos += 3;

    // DST (3 znaki) - adres odbiorcy odpowiedzi
    memcpy(&frame[pos], dst_addr, 3);
    pos += 3;
    memcpy(&crc_buf[crc_pos], dst_addr, 3);
    crc_pos += 3;

    // ID (2 znaki)
    memcpy(&frame[pos], id, 2);
    pos += 2;
    memcpy(&crc_buf[crc_pos], id, 2);
    crc_pos += 2;

    // LEN (3 znaki) - długość danych
    uint16_t data_len = strlen(data);
    char len_str[4];
    len_str[0] = '0' + (data_len / 100) % 10;
    len_str[1] = '0' + (data_len / 10) % 10;
    len_str[2] = '0' + data_len % 10;
    len_str[3] = 0;
    memcpy(&frame[pos], len_str, 3);
    pos += 3;
    memcpy(&crc_buf[crc_pos], len_str, 3);
    crc_pos += 3;

    // DATA
    memcpy(&frame[pos], data, data_len);
    pos += data_len;
    memcpy(&crc_buf[crc_pos], data, data_len);
    crc_pos += data_len;

    // Obliczanie CRC
    uint8_t crc = crc8(crc_buf, crc_pos);
    char crc_hex[3];
    byte2hex(crc, crc_hex);
    crc_hex[2] = 0;

    // CRC (2 znaki hex)
    memcpy(&frame[pos], crc_hex, 2);
    pos += 2;

    // Zakończenie ramki
    frame[pos++] = '*';
    frame[pos] = 0;

    // Wysyłanie ramki przez USART
    USART_fsend("%s", frame);
}
```

### 4.4 Obsługa komendy STOP (11)

```c
// 11 - STOP
else if (cmd_code == 11) {
    Measurement_EnableAutoRead(0);
    send_response_frame(device_addr, src_addr, id, "00"); // OK
}
```

Komenda STOP wywołuje `Measurement_EnableAutoRead(0)`, która ustawia `measurement_auto.enabled = 0`. Funkcja `Measurement_AutoRead_Process()` przestaje wykonywać pomiary.

### 4.5 Obsługa komendy DOWNLOAD (12)

Komenda DOWNLOAD zwraca ostatni pomiar z bufora:

```c
// 12 - DOWNLOAD (ostatni pomiar)
else if (cmd_code == 12) {
    uint16_t count = Measurement_GetCount();
    if (count == 0) {
        send_response_frame(device_addr, src_addr, id, "03"); // ERR_NO_DATA
        return;
    }

    // Pobranie ostatniego pomiaru (najnowszego)
    measurement_entry_t *entry = Measurement_GetEntry(count - 1);
    if (!entry) {
        send_response_frame(device_addr, src_addr, id, "03"); // ERR_NO_DATA
        return;
    }

    // Konwersja float → 4-cyfrowa liczba całkowita
    uint32_t lux_val = (uint32_t)(entry->lux + 0.5f); // Zaokrąglenie
    if (lux_val > 9999) {
        lux_val = 9999; // Obcięcie do max
    }

    // Formatowanie odpowiedzi jako 4 cyfry
    char response[5];
    response[0] = '0' + (lux_val / 1000) % 10;
    response[1] = '0' + (lux_val / 100) % 10;
    response[2] = '0' + (lux_val / 10) % 10;
    response[3] = '0' + lux_val % 10;
    response[4] = 0;

    send_response_frame(device_addr, src_addr, id, response);
}
```

Funkcja pobiera najnowszy pomiar z bufora, konwertuje wartość float na 4-cyfrową liczbę całkowitą (0000-9999) i wysyła jako odpowiedź.

### 4.6 Pobranie pomiaru z bufora cyklicznego

```c
measurement_entry_t* Measurement_GetEntry(uint16_t index) {
    if (index >= measurement_count) {
        return NULL; // Nieprawidłowy indeks
    }

    // Obliczenie rzeczywistego indeksu w buforze cyklicznym
    uint16_t real_index;
    if (measurement_count < MEASUREMENT_BUFFER_SIZE) {
        // Bufor nie jest jeszcze pełny - indeksy są liniowe
        real_index = index;
    } else {
        // Bufor jest pełny - najstarszy jest po najnowszym
        real_index = (measurement_write_index + index) % MEASUREMENT_BUFFER_SIZE;
    }

    return &measurement_buffer[real_index];
}
```

Funkcja obsługuje bufor cykliczny - gdy bufor jest pełny (1000 pomiarów), najstarsze wpisy są nadpisywane, ale indeksowanie pozostaje spójne (indeks 0 = najstarszy, indeks count-1 = najnowszy).

---

## 5. Pętla główna programu

### 5.1 Struktura main()

Funkcja `main()` inicjalizuje peryferia i uruchamia pętlę główną:

```c
int main(void) {
    /* Inicjalizacja peryferiów HAL */
    HAL_Init();
    SystemClock_Config();

    /* Inicjalizacja GPIO, UART, I2C, Timer */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();

    /* Uruchomienie timera aplikacyjnego (1ms tick) */
    HAL_TIM_Base_Start_IT(&htim2);

    /* Komunikat startowy */
    USART_fsend("\r\nSYSTEM START\r\n");

    /* Uruchomienie odbioru UART przez przerwania */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    /* Pętla główna - nieblokująca */
    while (1) {
        I2C_BusRecovery_Process();      // Obsługa recovery magistrali I2C
        process_uart_buffer();           // Parsowanie ramek protokołu
        BH1750_Init_Process();           // Inicjalizacja czujnika (maszyna stanów)
        Measurement_AutoRead_Process();  // Automatyczne pomiary

        if(USART_RxBufOverflow) {
            USART_fsend("\r\nERROR: USART RX buffer overflow!\r\n");
            USART_RxBufOverflow = 0;
        }
    }
}
```

Pętla główna wywołuje cztery funkcje w stałej kolejności, które współpracują w realizacji pomiarów. Wszystkie funkcje są nieblokujące - wykonują tylko to co mogą w danym momencie i natychmiast kończą działanie.

### 5.2 Sekwencja typowego pomiaru

**Krok 1: Odbiór komendy START**

```
UART → Znak po znaku do bufora (HAL_UART_RxCpltCallback)
     → process_uart_buffer() → Wykrycie '&' i '*'
     → validate_frame() → Sprawdzenie struktury i CRC
     → handle_command() → Rozpoznanie kodu 10
     → Measurement_EnableAutoRead(1) → measurement_auto.enabled = 1
```

**Krok 2: Inicjalizacja czujnika (rozłożona na wiele przebiegów pętli)**

```
Przebieg 1:
  BH1750_Init_Process() → Stan IDLE
    → I2C_Transmit_IT(0x01) → POWER_ON przez przerwania
    → Stan = PWRON
    → return
  [HAL I2C w tle przesyła bajt]
  IRQ → HAL_I2C_MasterTxCpltCallback() → i2c_op.pending = 0

Przebieg 2:
  BH1750_Init_Process() → Stan PWRON
    → I2C_Transmit_IT(0x07) → RESET przez przerwania
    → Stan = RESET
    → return
  [HAL I2C w tle przesyła bajt]
  IRQ → HAL_I2C_MasterTxCpltCallback() → i2c_op.pending = 0

Przebieg 3:
  BH1750_Init_Process() → Stan RESET
    → BH1750_SetMode(0x10) → I2C_Transmit_IT przez przerwania
    → BH1750_StartTiming(120) → Zapis app_tick i 120ms
    → Stan = MODE
    → return
  [HAL I2C w tle przesyła bajt]
  IRQ → HAL_I2C_MasterTxCpltCallback() → i2c_op.pending = 0

Przebieg 4:
  BH1750_Init_Process() → Stan MODE
    → Sprawdzenie !i2c_op.pending → OK
    → bh1750_initialized = 1
    → Stan = DONE
    → return
```

**Krok 3: Pierwszy pomiar (po 1 sekundzie)**

```
Przebieg N (app_tick >= last_measurement + 1000):
  Measurement_AutoRead_Process()
    → Wykrycie upływu interwału
    → BH1750_ReadLight() → I2C_Receive_IT(2 bajty)
    → HAL_I2C_Master_Receive_IT() → Rozpoczęcie odbioru
    → return

  [HAL I2C w tle odbiera 2 bajty z czujnika]

  IRQ → HAL_I2C_MasterRxCpltCallback()
      → Kopiowanie I2C_RxBuf → bh1750_read_buffer
      → raw_value = (buffer[0] << 8) | buffer[1]
      → bh1750_last_lux = raw_value / 1.2f
      → bh1750_read_ready = 1
      → i2c_op.pending = 0

Przebieg N+1:
  Measurement_AutoRead_Process()
    → Wykrycie bh1750_read_ready = 1
    → Measurement_AddEntry(bh1750_last_lux)
        → measurement_buffer[index].lux = bh1750_last_lux
        → measurement_buffer[index].timestamp = app_tick
        → index++, count++
    → bh1750_read_ready = 0
    → return
```

**Krok 4: Pobranie pomiaru przez PC**

```
UART → Ramka z kodem 12 (DOWNLOAD)
     → process_uart_buffer() → validate_frame()
     → handle_command() → cmd_code = 12
     → Measurement_GetEntry(count - 1) → Odczyt ostatniego
     → Konwersja: (uint32_t)(lux + 0.5f) → Zaokrąglenie
     → Formatowanie: "1234" (4 cyfry)
     → send_response_frame() → Budowa ramki z CRC
     → USART_fsend() → Wysłanie przez UART
```

### 5.3 Hierarchia wykonania

System działa na trzech poziomach priorytetów:

**Poziom 1: Przerwania sprzętowe (najwyższy priorytet)**

```c
// Timer 1ms - aktualizacja zegara systemowego
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        app_tick++;  // ~2-3 instrukcje CPU
    }
}

// I2C TX - zakończenie transmisji
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        i2c_op.pending = 0;  // 1 instrukcja CPU
    }
}

// I2C RX - zakończenie odbioru + przetwarzanie
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Kopiowanie + konwersja: ~20-30 instrukcji CPU
    // Czas: kilka mikrosekund
}

// UART RX - odbiór pojedynczego znaku
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // Zapis do bufora kołowego: ~10 instrukcji CPU
}
```

Wszystkie przerwania wykonują się bardzo szybko (< 10 μs) i nie blokują systemu.

**Poziom 2: Pętla główna (średni priorytet)**

Funkcje w pętli mogą być przerwane przez IRQ, ale wzajemnie nie przerywają się:

```c
while (1) {
    I2C_BusRecovery_Process();      // Jeśli był błąd I2C
    process_uart_buffer();           // Zawsze (sprawdzanie bufora)
    BH1750_Init_Process();           // Jeśli !initialized && enabled
    Measurement_AutoRead_Process();  // Jeśli enabled && initialized
}
```

**Poziom 3: Opóźnione operacje (najniższy priorytet)**

Operacje nieblokujące przez porównanie czasu:

```c
// Sprawdzenie czy minął interwał
if ((App_GetTick() - last_time) >= interval) {
    // Wykonaj akcję
}
```

### 5.4 Synchronizacja przez flagi

Flagi działają jako kanały komunikacji między kontekstem przerwania a pętlą główną:

```c
/* Przykład 1: Zajętość magistrali I2C */
// Pętla główna:
i2c_op.pending = 1;  // Przed HAL_I2C_Master_Transmit_IT()
// IRQ:
i2c_op.pending = 0;  // Po zakończeniu transmisji

/* Przykład 2: Gotowość pomiaru */
// Pętla główna:
bh1750_read_ready = 0;  // Przed rozpoczęciem odczytu
// IRQ:
bh1750_read_ready = 1;  // Po przetworzeniu danych

/* Przykład 3: Błąd I2C */
// Pętla główna:
I2C_Error = 0;  // Po obsłużeniu błędu
// IRQ:
I2C_Error = 1;  // Gdy wystąpił błąd
```

Flagi są typu `uint8_t` lub `volatile uint8_t` - operacje zapisu/odczytu są atomowe na ARM Cortex-M4, więc nie potrzeba mutex'ów.

---

## 6. Podsumowanie architektury

Implementacja obsługi czujnika BH1750 na STM32F446RE wykorzystuje **nieblokującą architekturę opartą na maszynach stanów i przerwaniach**. System składa się z kilku warstw:

**Warstwa sprzętowa:**

- Timer TIM2 generuje znaczniki czasowe co 1ms (`app_tick`)
- I2C1 komunikuje się z czujnikiem przez przerwania (TX/RX callbacks)
- UART2 odbiera komendy od PC przez przerwania

**Warstwa obsługi czujnika:**

- Maszyna stanów `BH1750_Init_Process()` inicjalizuje czujnik krok po kroku
- Funkcja `BH1750_ReadLight()` inicjuje nieblokujący odczyt
- Callback `HAL_I2C_MasterRxCpltCallback()` przetwarza dane (raw → luksy)

**Warstwa pomiarowa:**

- `Measurement_AutoRead_Process()` wykonuje cykliczne pomiary (co interwał)
- Bufor cykliczny przechowuje do 1000 pomiarów
- `Measurement_AddEntry()` zapisuje każdy pomiar z timestampem

**Warstwa komunikacyjna:**

- `process_uart_buffer()` parsuje ramki protokołu
- `handle_command()` obsługuje komendy START/STOP/DOWNLOAD
- `send_response_frame()` buduje i wysyła odpowiedzi z CRC

Cały system działa w **pętli głównej**, która cyklicznie wywołuje funkcje nieblokujące. Przerwania zajmują się tylko krytycznymi zadaniami (odbiór danych, aktualizacja flag), a właściwe przetwarzanie odbywa się w pętli, gdzie może być przerwane bez utraty danych. Dzięki temu system może równocześnie obsługiwać komunikację UART, inicjalizację czujnika i pomiary bez blokowania procesora.

## 4. Obsługa błędów i sytuacji wyjątkowych

### 4.1 Błąd komunikacji I2C

**Kiedy występuje:**

- Brak odpowiedzi czujnika (brak ACK)
- Uszkodzone połączenie fizyczne
- Zakłócenia na magistrali

**Co system robi:**

- Przerywa bieżącą operację
- Raportuje błąd przez protokół komunikacyjny
- W trybie ciągłym - próbuje ponowić odczyt po czasie timeout
- Nie zmienia ostatniej poprawnie odczytanej wartości

### 4.2 Nieprawidłowe dane

**Kiedy występuje:**
Podczas pracy z czujnikiem BH1750 mogą wystąpić różne sytuacje wymagające specjalnej obsługi. Najczęstszym problemem jest błąd komunikacji I2C, który objawia się brakiem odpowiedzi czujnika w postaci sygnału ACK. Może to wynikać z uszkodzonego połączenia fizycznego, poluzowanych przewodów, zakłóceń elektromagnetycznych na magistrali lub awarii samego czujnika. Gdy system wykrywa brak potwierdzenia od czujnika, natychmiast przerywa bieżącą operację odczytu i funkcja HAL zwraca kod błędu HAL_ERROR. System rejestruje wystąpienie błędu i może raportować go przez protokół komunikacyjny do komputera. W trybie automatycznych pomiarów cyklicznych, system nie zatrzymuje całkowicie działania - zamiast tego przy kolejnym przerwaniu timera ponawia próbę odczytu, dając czujnikowi szansę na powrót do normalnej pracy. Ważne jest, że ostatnia poprawnie odczytana wartość natężenia światła pozostaje nienaruszona w buforze, dzięki czemu system zachowuje ostatni znany stan.

Inną sytuacją wymagającą uwagi są wartości skrajne odczytane z czujnika. Gdy czujnik zwraca wartość 0x0000, może to oznaczać zarówno całkowitą ciemność (brak światła), jak i potencjalny błąd pomiaru. Podobnie wartość 0xFFFF może wskazywać na nasycenie czujnika przy bardzo jasnym świetle lub nieprawidłowe działanie. System traktuje te wartości jako technicznie poprawne i akceptuje je, ponieważ mogą reprezentować rzeczywiste warunki oświetleniowe. Wartości te są normalnie przetwarzane, zapisywane do bufora i raportowane do komputera. Opcjonalnie, system może oznaczyć takie pomiary flagą jako potencjalnie podejrzane, jeśli implementacja wymaga dodatkowej walidacji danych.

Timeout komunikacji to kolejna sytuacja wyjątkowa, która występuje gdy czujnik nie odpowiada w określonym limicie czasowym. Biblioteka HAL domyślnie używa wartości HAL_MAX_DELAY, co oznacza nieskończone oczekiwanie, ale można ustawić konkretny limit czasu (np. 100ms). Gdy czas oczekiwania zostaje przekroczony, funkcja HAL_I2C_Master_Receive przerywa blokowanie procesora i zwraca status HAL_TIMEOUT. System zapisuje informację o timeout'cie, raportuje problem do nadrzędnych warstw oprogramowania i przechodzi do dalszego działania, umożliwiając obsługę innych zadań zamiast zawiesić się w nieskończonym oczekiwaniu na czujnik.

- Dokładność: ±20% (typowa)
- Powtarzalność: ±15%

### 5.3 Możliwości kalibracji

- Czujnik może być rekalibrowany przez zmianę czasu pomiaru
- Możliwość kompensacji dla różnych materiałów okien
- Możliwość ustawienia współczynnika korekcyjnego

---

## 6. Przepływ danych w systemie

W trybie pomiarów ciągłych, który aktywuje się po otrzymaniu komendy START, system wykorzystuje timer TIM2 do odmierzania interwałów czasowych. Timer generuje przerwania co 1ms, wywołując funkcję callback `HAL_TIM_PeriodElapsedCallback()`, która inkrementuje licznik `app_tick`. Ten licznik służy jako nieblokujący zegar systemowy używany przez wszystkie moduły do śledzenia upływu czasu.

W pętli głównej programu cyklicznie wykonuje się funkcja `Measurement_AutoRead_Process()`. Na każdym przebiegu sprawdza ona aktualną wartość `app_tick` i porównuje ją z czasem ostatniego pomiaru. Gdy różnica osiągnie ustawiony interwał (np. 1000ms), funkcja sprawdza czy magistrala I2C jest wolna poprzez sprawdzenie flagi `i2c_op.pending`. Jeśli magistrala jest dostępna, system wywołuje `BH1750_ReadLight()`, która inicjuje nieblokującą transmisję I2C przez `HAL_I2C_Master_Receive_IT()`. Funkcja ta natychmiast zwraca sterowanie, a transmisja przebiega w tle obsługiwana przez kontroler DMA i przerwania.

Gdy kontroler I2C zakończy odbiór 2 bajtów danych z czujnika, generuje przerwanie i wywołuje callback `HAL_I2C_MasterRxCpltCallback()`. W tym momencie procesor zostaje tymczasowo przerwany (niezależnie od tego co aktualnie wykonywał w pętli głównej) i przeskakuje do obsługi przerwania. Callback kopiuje odebrane bajty z bufora sprzętowego I2C do bufora aplikacji, łączy je w wartość 16-bitową operacją `(data[0] << 8) | data[1]`, dzieli przez 1.2 otrzymując wynik w luksach, zapisuje go w zmiennej `bh1750_last_lux` i ustawia flagę `bh1750_read_ready = 1`. Cała ta operacja trwa zaledwie kilka mikrosekund, po czym procesor powraca do przerwanego zadania w pętli głównej.

W kolejnym przebiegu pętli głównej, funkcja `Measurement_AutoRead_Process()` wykrywa ustawioną flagę `bh1750_read_ready` i wywołuje `Measurement_AddEntry()`. Ta funkcja zapisuje wartość z `bh1750_last_lux` wraz z aktualnym znacznikiem czasowym do bufora cyklicznego `measurement_buffer` pod indeksem `measurement_write_index`, następnie inkrementuje indeks z zawinięciem modulo 1000. Po zapisaniu system zeruje flagę gotowości i aktualizuje `measurement_auto.last_measurement` na aktualny `app_tick`, rozpoczynając odliczanie kolejnego interwału. Cały proces - od sprawdzenia czasu, przez inicjalizację transmisji, obsługę przerwania, aż po zapis w buforze - przebiega asynchronicznie bez blokowania procesora.

Gdy system odbiera przez UART ramkę z komendą DOWNLOAD (identyfikator 120), przepływ danych przebiega inaczej. Parser protokołu analizuje odebraną ramkę, weryfikuje jej strukturę i sumę kontrolną CRC, po czym rozpoznaje identyfikator komendy i wywołuje odpowiednią funkcję obsługi. Ta funkcja odczytuje wartość przechowywaną w buforze pomiarowym - jest to liczba float reprezentująca ostatni zmierzony poziom oświetlenia w luksach. Wartość ta musi zostać skonwertowana na format 4-cyfrowej liczby całkowitej poprzez zaokrąglenie i przycięcie do zakresu 0000-9999. System następnie konstruuje ramkę odpowiedzi według specyfikacji protokołu, wstawiając nadawcę (STM), odbiorcę (PC\_), identyfikator ramki, długość danych (004) oraz skonwertowaną wartość natężenia światła. Dla tak utworzonej ramki obliczana jest suma kontrolna CRC-8 obejmująca wszystkie pola od nadawcy do końca danych. Na koniec kompletna ramka, otoczona znakami rozpoczęcia (&) i zakończenia (*), jest wysyłana przez UART z powrotem do komputera.*ROM:\*\* Kod obsługi czujnika (~2-3 KB)

### 7.2 Priorytet zadań

1. Obsługa komunikacji UART (najwyższy - komendy użytkownika)
2. Odczyt czujnika (średni - operacja timeoutowa)
3. Przetwarzanie danych (niski - nieblokujące)

### 7.3 Synchronizacja

- Dostęp do bufora pomiaru musi być chroniony (atomic read/write)
- Timer nie może uruchomić nowego pomiaru przed zakończeniem poprzedniego
- Komunikacja I2C musi być wyłączna (jeden proces na raz)

---

## 8. Rozszerzenia i ulepszenia

### 8.1 Filtrowanie pomiarów

- Średnia krocząca z N ostatnich pomiarów
- Odrzucanie wartości odstających (outliers)
- Filtr medianowy dla stabilności

### 8.Wymagania systemowe i synchronizacja

Implementacja obsługi czujnika BH1750 wymaga wykorzystania kilku zasobów sprzętowych mikrokontrolera. Timer (np. TIM2) jest niezbędny do generowania cyklicznych pomiarów - musi być skonfigurowany tak, aby generował przerwania w odstępach odpowiadających żądanemu interwałowi próbkowania. Interfejs I2C1 służy jako magistrala komunikacyjna z czujnikiem i musi być dostępny w trakcie wykonywania odczytów. UART2 pełni rolę łącza komunikacyjnego z komputerem PC, obsługując protokół ramkowy. W pamięci RAM system rezerwuje bufor na ostatni pomiar - zwykle 4 bajty dla wartości zmiennoprzecinkowej float. Kod obsługi czujnika, włączając funkcje inicjalizacji, odczytu i przetwarzania danych, zajmuje w pamięci ROM około 2-3 KB.

System zarządza priorytetami zadań przez hierarchię przerwań sprzętowych i sekwencję wywołań w pętli głównej. Najwyższy priorytet ma przerwanie timera TIM2 (wywołuje `HAL_TIM_PeriodElapsedCallback()` co 1ms), które aktualizuje zegar systemowy `app_tick`. Następnie w hierarchii znajdują się przerwania I2C (`HAL_I2C_MasterTxCpltCallback`, `HAL_I2C_MasterRxCpltCallback`, `HAL_I2C_ErrorCallback`), które obsługują zakończenie transmisji i błędy komunikacji. Przerwanie UART (`HAL_UART_RxCpltCallback`) ma priorytet podobny do I2C i odbiera pojedyncze znaki do bufora kołowego. Wszystkie te przerwania wykonują się bardzo szybko (kilka mikrosekund), tylko aktualizując flagi i bufory, po czym natychmiast zwracają sterowanie.

Właściwe przetwarzanie danych odbywa się w pętli głównej, która wykonuje zadania w stałej kolejności: najpierw `I2C_BusRecovery_Process()` (obsługa błędów magistrali), następnie `process_uart_buffer()` (parsowanie ramek protokołu i obsługa komend), potem `BH1750_Init_Process()` (maszyna stanów inicjalizacji czujnika), i na końcu `Measurement_AutoRead_Process()` (automatyczne pomiary). Taka sekwencja zapewnia że błędy I2C są szybko naprawiane, komendy użytkownika są obsługiwane priorytetowo, a pomiary wykonują się w tle gdy system ma wolny czas.

Kluczowym aspektem implementacji jest prawidłowa synchronizacja dostępu do współdzielonych zasobów. Bufor `bh1750_last_lux` jest modyfikowany w przerwaniu I2C (callback `HAL_I2C_MasterRxCpltCallback`) i odczytywany w pętli głównej (`Measurement_AutoRead_Process`). Ponieważ zapis zmiennej typu float nie jest operacją atomową na procesorze ARM Cortex-M4, istnieje teoretyczne ryzyko przerwania zapisu w połowie. W praktyce jednak flaga `bh1750_read_ready` działa jako bariera synchronizacyjna - pętla główna czyta wartość tylko gdy flaga jest ustawiona, a po odczytaniu natychmiast ją zeruje. Przerwanie I2C nigdy nie nadpisuje wartości dopóki flaga nie zostanie wyzerowana, więc race condition nie występuje.

Magistrala I2C jest chroniona przez flagę `i2c_op.pending`, która działa jak mutex (wzajemne wykluczanie). Każda funkcja inicjująca transmisję (`I2C_Transmit_IT`, `I2C_Receive_IT`) najpierw sprawdza czy flaga jest wyzerowana - jeśli nie, zwraca HAL_BUSY. Po przejęciu magistrali funkcja ustawia `i2c_op.pending = 1` i rozpoczyna transmisję. Dopiero callback przerwania (po zakończeniu transmisji) zeruje flagę, zwalniając magistralę. Dzięki temu wiele części systemu (inicjalizacja BH1750, automatyczne pomiary, komendy użytkownika) może próbować użyć I2C, ale tylko jedna operacja może być aktywna w danym momencie. Jeśli magistrala jest zajęta, żądający po prostu czeka do kolejnego przebiegu pętli głównej.

### 9.1 Test komunikacji

**Cel:** Sprawdzenie połączenia z czujnikiem
**Metoda:** Wysłanie komendy POWER_ON i oczekiwanie ACK
**Wynik:** OK = czujnik odpowiada, ERROR = brak połączenia

### 9.2 Test poprawności pomiarów

**Cel:** Weryfikacja działania czujnika
**Metoda:**

- Pomiar w różnych warunkach oświetlenia
- PoróProcedury diagnostyczne i testowe

Podstawowym testem weryfikującym poprawność implementacji jest test komunikacji z czujnikiem. Po uruchomieniu systemu, program wysyła do czujnika komendę POWER_ON i oczekuje na sygnał ACK potwierdzający odbiór. Jeśli czujnik odpowiada prawidłowo, funkcja HAL zwraca status HAL_OK, co potwierdza że połączenie fizyczne jest poprawne, adres czujnika jest właściwy, magistrala I2C działa i czujnik jest sprawny. W przypadku braku odpowiedzi, funkcja zwraca HAL_ERROR, co wskazuje na problem z połączeniem - możliwe przyczyny to poluzowane przewody, błędny adres, niesprawny czujnik lub niedziałająca magistrala I2C.

Test poprawności pomiarów weryfikuje czy czujnik rzeczywiście mierzy natężenie światła zgodnie z oczekiwaniami. System wykonuje serie pomiarów w różnych warunkach oświetleniowych - od całkowitej ciemności przez normalne oświetlenie pomieszczenia aż do jasnego światła słonecznego. Odczyty porównywane są z wartościami z referencyjnego luksomierza lub ze znanymi wartościami typowymi (np. biuro: 300-500 lx, bezpośrednie słońce: 10000+ lx). Dodatkowo sprawdzana jest powtarzalność - przy stałym oświetleniu wykonywanych jest kilkanaście pomiarów i analizowana jest ich rozrzut. Jeśli kolejne odczyty różnią się minimalnie (w granicach ±15%), czujnik działa prawidłowo. Duże wahania mogą wskazywać na zakłócenia elektromagnetyczne lub problemy z zasilaniem.

Test trwałości i stabilności długoterminowej polega na uruchomieniu systemu w trybie ciągłych pomiarów na dłuższy okres, typowo od 1 do 24 godzin. System wykonuje pomiary co sekundę lub co kilka sekund, zapisując wyniki do pamięci lub przesyłając do komputera. Podczas testu monitorowana jest liczba błędów komunikacji - system idealny nie powinien generować żadnych błędów I2C przez cały okres testowy. Sprawdzana jest również stabilność wartości w stałych warunkach oświetlenia - jeśli czujnik znajduje się w pomieszczeniu o niezmiennym świetle, odczyty powinny pozostawać w wąskim zakresie, bez dryfu w górę lub w dół. Wykrycie systematycznego dryfu może wskazywać na problemy termiczne czujnika lub degradację elementów pomiarowych.
**Rozwiązanie:** Zawsze podzielić przez 1.2 dla trybu H_RES_MODE

---

## Podsumowanie

Pułapki i typowe błędy implementacyjne

Jednym z najczęstszych błędów popełnianych podczas implementacji jest próba odczytu czujnika natychmiast po jego włączeniu lub zmianie trybu pracy. Programista wysyła komendę ustawienia trybu i od razu próbuje odczytać dane, nie czekając na zakończenie pomiaru. Czujnik BH1750 potrzebuje 120ms na wykonanie pomiaru w trybie wysokiej rozdzielczości, więc odczyt przed upływem tego czasu zwróci nieprawidłowe dane lub spowoduje błąd komunikacji, gdyż czujnik nie jest jeszcze gotowy do odpowiedzi. Rozwiązaniem jest zawsze odczekanie minimum 120ms po wysłaniu komendy trybu pomiaru przed pierwszym odczytem, używając funkcji HAL_Delay() lub mechanizmu odroczonego odczytu przez timer.

Kolejną pułapką jest nieprawidłowe przesunięcie adresu czujnika podczas komunikacji I2C. Funkcje biblioteki HAL (HAL_I2C_Master_Transmit i HAL_I2C_Master_Receive) wymagają 7-bitowego adresu przesuniętego o jeden bit w lewo, ponieważ automatycznie dodają bit odczytu/zapisu na najmniej znaczącej pozycji. Programista, widząc w dokumentacji czujnika adres 0x23, może go użyć bezpośrednio, ale należy przekazać go jako `(0x23 << 1)` lub po prostu `0x46`. Zapomnienie tego przesunięcia powoduje że mikrokontroler próbuje komunikować się z adresem 0x11 zamiast 0x23, co kończy się brakiem odpowiedzi i błędem komunikacji.

Błędna interpretacja kolejności bajtów to kolejny częsty problem. Czujnik BH1750 przesyła dane w formacie big-endian, gdzie najpierw wysyłany jest najbardziej znaczący bajt (MSB), a następnie najmniej znaczący (LSB). Programista przyzwyczajony do architektury little-endian (używanej przez STM32) może automatycznie złożyć bajty w odwrotnej kolejności: `(data[1] << 8) | data[0]`, co daje całkowicie błędne wartości, często przewyższające zakres pomiarowy czujnika. Prawidłowa konstrukcja to zawsze `(data[0] << 8) | data[1]`, gdzie data[0] to pierwszy odebrany bajt.

Ostatnim typowym błędem jest zapomnienie o konwersji surowej wartości na luksy. Czujnik zwraca wartość liczbową, która musi zostać podzielona przez 1.2 dla trybu H_RES_MODE, aby otrzymać rzeczywisty wynik w luksach. Programista wyświetlający lub przesyłający surową wartość bez tej konwersji otrzyma wyniki zawyżone o około 20%, co prowadzi do błędnej interpretacji poziomu oświetlenia. Na przykład przy rzeczywistym oświetleniu 500 luksów, czujnik zwróci wartość około 600, która bez dzielenia przez 1.2 zostanie błędnie zinterpretowana.

---

## Podsumowanie

Implementacja obsługi czujnika BH1750 na mikrokontrolerem STM32F446RE to proces obejmujący kilka współpracujących ze sobą elementów. Rozpoczyna się od konfiguracji sprzętowej, gdzie czujnik podłączany jest do magistrali I2C na pinach PB6 i PB7. Podczas startu systemu mikrokontroler inicjalizuje czujnik wysyłając sekwencję komend - włączenie zasilania, odczekanie na stabilizację, a następnie ustawienie trybu pomiaru. Od tego momentu czujnik automatycznie wykonuje pomiary w wybranym trybie.

W trybie pracy ciągłej, timer generuje cykliczne przerwania, które uruchamiają odczyt danych z czujnika przez magistralę I2C. Odebrane dwa bajty są łączone w wartość 16-bitową, przeliczane na luksy i zapisywane do bufora. Równolegle system nasłuchuje komend przychodzących przez UART - komenda START aktywuje cykliczne pomiary, STOP je zatrzymuje, a DOWNLOAD zwraca ostatnią zmierzoną wartość do komputera PC.

Cały system musi być odporny na błędy komunikacji, prawidłowo synchronizować dostęp do współdzielonych zasobów i zarządzać priorytetami zadań. Implementacja wymaga uwagi na szczegóły czasowe, poprawną interpretację danych binarnych i odpowiednią konwersję jednostek, aby zapewnić wiarygodne pomiary natężenia światła
