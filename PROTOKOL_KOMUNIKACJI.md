# Protokół komunikacji PC ↔ STM32

## Format ramki

```
& SRC(3) DST(3) ID(2) LEN(3) DATA(0-256) CRC(2) *
```

- **&** - początek ramki
- **SRC** - nadawca (3 znaki ASCII: PC\_, STM)
- **DST** - odbiorca (3 znaki ASCII: STM, PC\_)
- **ID** - identyfikator ramki (2 cyfry dziesiętne: 00-99)
- **LEN** - długość pola DATA (3 cyfry dziesiętne: 000-256)
- **DATA** - dane (0-256 znaków ASCII)
- **CRC** - suma kontrolna CRC-8 (2 znaki hex)
- **\*** - koniec ramki

**Maksymalna długość ramki:** 271 znaków (& + 3 + 3 + 2 + 3 + 256 + 2 + \*)

### CRC-8

- **Wariant:** CRC-8
- **Polynomial:** 0x07
- **Initial value:** 0x00
- **Zakres:** SRC + DST + ID + LEN + DATA (bez & i \*)
- **MSB-first**

---

## Komendy

### 1. START (10)

Rozpoczęcie automatycznego odczytu pomiarów z BH1750.

**Wysyłka:**

```
& PC_ STM 01 003 100 XX *
```

**Odbiór:**

```
& STM PC_ 01 002 00 XX *
```

- `00` = OK

---

### 2. STOP (11)

Zatrzymanie automatycznego odczytu pomiarów.

**Wysyłka:**

```
& PC_ STM 02 003 110 XX *
```

**Odbiór:**

```
& STM PC_ 02 002 00 XX *
```

- `00` = OK

---

### 3. DOWNLOAD (12)

Pobranie ostatniego pomiaru.

**Wysyłka:**

```
& PC_ STM 03 003 120 XX *
```

**Odbiór:**

```
& STM PC_ 03 004 LLLL XX *
```

- `LLLL` = wartość natężenia światła w luksach (4 cyfry dziesiętne, 0000-9999)

**Przykład:**

```
& STM PC_ 03 004 1234 XX *
```

Odczyt: 1234 luksy

---

### 4. VIEW (13xxzz)

Podgląd wielu pomiarów z bufora. Pomiary są wysyłane w paczkach (max 63 pomiary/ramkę).

**Parametry:**

- `xx` - offset początkowy (00-99) - od końca bufora (0 = najnowszy)
- `zz` - liczba pomiarów do wyświetlenia (01-99)

**Wysyłka:**

```
& PC_ STM 04 007 130002 XX *
```

- `13` = kod komendy VIEW
- `00` = offset 0 (najnowsze pomiary)
- `02` = pobierz 2 pomiary

**Odbiór (wielokrotne ramki):**
Każda ramka zawiera:

- Offset początkowy ramki (2 cyfry)
- Ciąg wartości lux (każda po 4 cyfry)

**Format DATA:** `xxLLLLLLLL...`

- `xx` = offset początkowy tej paczki
- `LLLL` = kolejne 4-cyfrowe wartości lux

**Przykład - 2 pomiary:**

```
& STM PC_ 04 010 0012343210 XX *
```

- `00` = offset 0
- `1234` = pomiar #0 (najnowszy): 1234 lux
- `3210` = pomiar #1: 3210 lux

**Przykład - 150 pomiarów (3 ramki):**

Ramka 1:

```
& STM PC_ 04 254 00LLLLLLLL...(63 pomiary) XX *
```

- Offset: 00, pomiary 0-62 (63 pomiary × 4 cyfry = 252 znaki + 2 offset)

Ramka 2:

```
& STM PC_ 04 254 63LLLLLLLL...(63 pomiary) XX *
```

- Offset: 63, pomiary 63-125

Ramka 3:

```
& STM PC_ 04 098 26LLLLLLLL...(24 pomiary) XX *
```

- Offset: 126, pomiary 126-149 (24 pomiary × 4 = 96 + 2 offset)

**Maksymalnie pomiarów na ramkę:** 63  
Formula: `(256 - 2) / 4 = 63`

---

### 5. SET_INTERVAL (14xxxx)

Ustawienie interwału pomiarowego.

**Parametry:**

- `xxxx` = interwał w milisekundach (0001-9999)

**Wysyłka:**

```
& PC_ STM 05 007 141000 XX *
```

- `14` = kod komendy
- `1000` = 1000 ms (1 sekunda)

**Odbiór:**

```
& STM PC_ 05 002 00 XX *
```

- `00` = OK

---

### 6. GET_INTERVAL (15)

Odczyt aktualnego interwału pomiarowego.

**Wysyłka:**

```
& PC_ STM 06 003 150 XX *
```

**Odbiór:**

```
& STM PC_ 06 004 1000 XX *
```

- `1000` = aktualny interwał (1000 ms)

---

### 7. SET_MODE (16x)

Ustawienie trybu pracy czujnika BH1750.

**Parametry:**

- `x` = numer trybu (1-6):
  - `1` = Continuous High Resolution Mode (1 lx, 120 ms)
  - `2` = Continuous High Resolution Mode 2 (0.5 lx, 120 ms)
  - `3` = Continuous Low Resolution Mode (4 lx, 16 ms)
  - `4` = One Time High Resolution Mode (1 lx, 120 ms)
  - `5` = One Time High Resolution Mode 2 (0.5 lx, 120 ms)
  - `6` = One Time Low Resolution Mode (4 lx, 16 ms)

**Wysyłka:**

```
& PC_ STM 07 004 161 XX *
```

- `16` = kod komendy
- `1` = tryb 1 (Continuous High Res)

**Odbiór:**

```
& STM PC_ 07 002 00 XX *
```

- `00` = OK

---

### 8. GET_MODE (17)

Odczyt aktualnego trybu pracy czujnika.

**Wysyłka:**

```
& PC_ STM 08 003 170 XX *
```

**Odbiór:**

```
& STM PC_ 08 001 1 XX *
```

- `1` = tryb 1

---

### 9. I2C_SCAN (18)

Skanowanie magistrali I2C w poszukiwaniu czujnika BH1750.

**Wysyłka:**

```
& PC_ STM 09 003 180 XX *
```

**Odbiór (znaleziono urządzenie):**

```
& STM PC_ 09 002 23 XX *
```

- `23` = znaleziony adres (hex: 0x23)

**Odbiór (brak urządzenia):**

```
& STM PC_ 09 002 00 XX *
```

- `00` = nie znaleziono urządzenia

---

## Kody błędów

W odpowiedziach na komendy mogą wystąpić następujące kody błędów:

- `00` - OK (sukces)
- `01` - ERR_PARAM (błędne parametry)
- `02` - ERR_RANGE (wartość poza zakresem)
- `03` - ERR_NO_DATA (brak danych w buforze)
- `04` - ERR_I2C (błąd komunikacji I2C)
- `ERR` - błąd ogólny
- `ERR_CRC` - błąd sumy kontrolnej
- `ERR_LENGTH` - błędna długość danych

---

## Przykładowe sekwencje

### Typowa sesja pomiarowa

1. **Skanowanie I2C:**

```
→ & PC_ STM 01 003 180 XX *
← & STM PC_ 01 002 23 XX *
```

2. **Ustawienie interwału na 500 ms:**

```
→ & PC_ STM 02 007 140500 XX *
← & STM PC_ 02 002 00 XX *
```

3. **Start pomiarów:**

```
→ & PC_ STM 03 003 100 XX *
← & STM PC_ 03 002 00 XX *
```

4. **Odczekanie ~5 sekund...**

5. **Pobranie ostatniego pomiaru:**

```
→ & PC_ STM 04 003 120 XX *
← & STM PC_ 04 004 1234 XX *
```

6. **Pobranie 10 ostatnich pomiarów:**

```
→ & PC_ STM 05 007 130010 XX *
← & STM PC_ 05 042 001234567890120034... XX *
```

7. **Stop pomiarów:**

```
→ & PC_ STM 06 003 110 XX *
← & STM PC_ 06 002 00 XX *
```

---

## Uwagi implementacyjne

1. **Identyfikator ramki (ID):** Każda nowa komenda powinna mieć unikalny/kolejny ID dla lepszego śledzenia komunikacji.

2. **Timeout:** Zaleca się ustawienie timeout dla odpowiedzi (~500-1000 ms).

3. **Wielokrotne ramki VIEW:** Odbiornik musi być przygotowany na otrzymanie wielu ramek w odpowiedzi na jedną komendę VIEW z dużym `zz`.

4. **Bufor pomiarów:** STM32 przechowuje max 1000 pomiarów w buforze cyklicznym.

5. **Offset w VIEW:** Offset 0 = najnowszy pomiar, offset 99 = 100. pomiar od końca.

6. **Parametry dziesiętne:** Wszystkie parametry liczbowe w DATA są wysyłane jako ASCII dziesiętne (nie binarne).

7. **CRC:** CRC jest obliczany dla wszystkich pól od SRC do końca DATA (bez & i \*).
