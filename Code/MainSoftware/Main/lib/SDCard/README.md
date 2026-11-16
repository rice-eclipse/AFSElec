# SDCard: microSD card + FAT/exFAT filesystem

SPI-mode SD card driver for the RP2040 / RP2350 (Pico SDK), with ChaN's
[FatFs](http://elm-chan.org/fsw/ff/) R0.15 on top. It reads and writes normal files that a PC can
open: CSV logs, text, binary dumps and config files.

| Class       | What it's for                                                                          |
|-------------|----------------------------------------------------------------------------------------|
| `SDCard`    | The card: init, mount, format, card info, mkdir / remove / rename / list / free space   |
| `SDFile`    | One open file: `write`, `printf`, `read`, `readLine`, `seek`, `sync`                    |
| `CSVWriter` | Buffered CSV logging: `writeHeader`, `add(x).add(y)...endRow()`, auto-sync every 1 s    |
| `CSVReader` | Read a CSV back row by row (quoted fields supported), `getFloat` / `getInt` / `indexOf` |

Link with `target_link_libraries(<target> SDCard)`. The bench test is `SDCard_test`.

## This board

```cpp
#include "SDCard.h"
#include "board_pins.h"

// spi1: MISO 28, CS 29, SCK 30, MOSI 31. Card detect on GPIO 35.
SDCard sd(SD_SPI, SD_CHIP_SELECT, SPI_SCK_SD, SPI_MOSI_SD, SPI_MISO_SD, SD_CHIP_DETECT);
```

The SD socket (CUI MSD-4-A) has a card-detect switch that connects CD to ground when a card is
inserted, so the default is `cdActiveLow = true` with the internal pull-up on. If `cardPresent()` reports
the opposite of what's really in the socket, pass `false` as the 7th constructor argument. Pass
`SDCard::NO_PIN` to ignore the switch.

## Logging a flight to CSV

```cpp
SDCard sd(SD_SPI, SD_CHIP_SELECT, SPI_SCK_SD, SPI_MOSI_SD, SPI_MISO_SD, SD_CHIP_DETECT);
CSVWriter logFile;          // global: it holds a 4 kB buffer, too big for the 2 kB stack

int main() {
    stdio_init_all();
    if (!sd.begin()) printf("SD failed: %s\n", sd.lastError());

    char name[40];
    sd.nextFreeName("flights/flight_", ".csv", name, sizeof name);   // flights/flight_000.csv, _001, ...
    logFile.open(name);
    logFile.writeHeader({"t_ms", "ax_g", "ay_g", "az_g", "alt_m", "state"});

    while (true) {
        // ... read sensors ...
        logFile.add(to_ms_since_boot(get_absolute_time()))
               .add(i.ax, 3).add(i.ay, 3).add(i.az, 3)     // 3 decimals
               .add(baro.getAltitude(), 2)
               .add("BOOST")
               .endRow();
        // optional: logFile.errorCount() > 0 means some rows could not be written
    }
}
```

* `add()` takes integers, `float`/`double` (optional decimal places, default 4; `NaN` becomes an
  empty field), `bool` (1/0), `char`, and strings. A string that contains a comma, a quote or a newline
  is quoted automatically. `addf("%08lX", flags)` adds a printf-formatted field. `skip()` adds an empty field.
* Rows go into a 4 kB RAM buffer and are written to the card in 4 kB blocks. Most `endRow()`
  calls take only a few µs. The call that writes a block takes about 1 to 3 ms, depending on the card.
* **Power-loss safety:** FAT only records a file's new size when the file is synced. `endRow()` syncs
  at most every `setSyncIntervalMs()` (default 1000 ms), so a power cut loses at most about 1 s
  of data. The rest of the file is still readable. You can also call `setSyncEveryRows(n)`, or
  `sync()` yourself at important events such as apogee or landing.
* `open(path, /*append*/ true)` adds rows to an existing file. `writeHeader()` is then skipped if the file already has data.
* `SD_CSV_BUFFER_SIZE` (default 4096) sets the buffer size. Define it before including the header,
  or add it as a compile definition. Multiples of 512 are fastest.
* Maximum row length is 511 characters. A longer row is dropped and counted in `errorCount()`.

## Other file operations

```cpp
sd.writeFile("config.txt", "launch_alt=0\n");        // create / overwrite
sd.appendFile("events.txt", "apogee 1234 ms\n");
char buf[128];
int32_t n = sd.readFile("config.txt", buf, sizeof buf - 1);   // -1 on error
if (n >= 0) buf[n] = 0;

sd.mkdir("flights/2026");          // creates parents, fine if it already exists
sd.exists("flights/2026");         // files or folders
sd.isDirectory("flights");
sd.fileSize("flight_000.csv");     // -1 if missing
sd.rename("a.csv", "old/a.csv");   // rename / move
sd.remove("old/a.csv");            // file or empty folder
sd.removeRecursive("old");         // folder and everything in it
sd.printDir("/", /*recursive*/ true);
sd.listDir("flights", [](const SDCard::FileInfo& f, void*) {
    printf("%s %llu\n", f.name, (unsigned long long)f.size);
});
sd.printInfo();                    // card type, size, filesystem, label, free space
sd.freeBytes(); sd.totalBytes();
sd.format();                       // FAT32 (format(true) for exFAT). ERASES THE CARD.
```

`SDFile` gives direct access to one file:

```cpp
SDFile f;                                      // ~560 bytes; fine as a local for short use
if (f.open("events.txt", SDFile::Mode::Append)) {
    f.printf("%lu,%s\n", (unsigned long)ms, "drogue");
    f.close();                                 // or f.sync() to keep it open
}

f.open("config.txt", SDFile::Mode::Read);
char line[64];
while (f.readLine(line, sizeof line) >= 0) { /* handles \n and \r\n */ }
```

Modes: `Read` (must exist), `Write` (create or truncate), `Append` (create or open at end),
`ReadWrite` (must exist), `CreateNew` (fails if the file exists). Also available: `seek`, `position`, `size`,
`eof`, `truncate`, `read(buf, n)`, `read()` (one byte).

Reading a CSV:

```cpp
static CSVReader csv;               // ~1.1 kB
csv.open("flights/flight_000.csv");
csv.readRow();                      // header
int alt = csv.indexOf("alt_m");
while (csv.readRow()) printf("%.2f\n", csv.getFloat(alt));
csv.close();
```

## Errors

Every filesystem call returns `bool`. When one fails, `sd.lastResult()` returns the FatFs `FRESULT` and
`sd.lastError()` returns it as text (`"no such file"`, `"no filesystem"`, `"denied (full or read-only)"`, ...).
`SDFile::lastResult()` gives the same for file operations.

| Problem                         | Likely cause                                                                     |
|---------------------------------|----------------------------------------------------------------------------------|
| `begin()` fails, `cardInitialized()` false | No card, wiring, no pull-up on MISO, bad power. Try `setSpiFrequency(4'000'000)`. |
| `cardInitialized()` true, mount fails with `no filesystem` | Card not FAT/exFAT formatted. Format it on a PC, or use `sd.format()`. |
| `cardPresent()` wrong           | CD switch polarity, so flip `cdActiveLow`.                                       |
| Writes fail after hot-swapping  | Call `sd.end()` and then `sd.begin()` again after inserting a card.              |

## Timestamps

FatFs stamps files with a creation and modification time. With no clock set, the time is
2026-01-01 00:00:00 plus the time since boot. If you know the real time (for example from GPS), call
`SDCard::setDateTime(2026, 11, 6, 14, 30, 0)`. The clock then keeps running from the system timer.

## Raw sector access

`readSectors(lba, buf, count)` and `writeSectors(lba, buf, count)` bypass the filesystem and use
multi-block commands when `count > 1`. **Writing raw sectors on a mounted card will corrupt the filesystem.**

## Driver notes

* Supports MMC v3, SD v1, SD v2 (SDSC) and SDHC/SDXC (block addressed). Initializes at 400 kHz,
  then switches to `spiHz` (default 25 MHz, the SD maximum in SPI mode). `spiFrequency()` returns the
  clock the RP2350 actually set.
* Every command uses CRC7. Data blocks use CRC16 (`CMD59` on), so a corrupted read fails instead
  of returning bad data. Turn this off with `setCRC(false)` before `begin()`.
* Multi-sector writes send `ACMD23` (pre-erase) and then `CMD25`. Each write waits for the card
  to finish programming before returning, so returned data is actually on the card.
* Like the sensor drivers, the SPI baud rate and mode are set on every transaction, so the card can share a bus.
  `begin(false)` skips `spi_init` / pin setup if you configure the bus yourself.
* Only one `SDCard` can be mounted at a time. FatFs is configured for a single volume.
* Timing: a single-block write takes about 0.5 to 2 ms. Cards occasionally stall for up to about
  250 ms during internal housekeeping, and the driver allows up to 500 ms (1 s for multi-block).
  On a time-critical loop, log from the second core or accept an occasional slow `endRow()`.

## FatFs configuration (`fatfs/ffconf.h`)

`fatfs/` holds the unmodified FatFs R0.15 sources (copied from the Pico SDK's TinyUSB), except for
`ffconf.h`. The options changed from the stock values are:

| Option            | Value | Why                                                    |
|-------------------|-------|--------------------------------------------------------|
| `FF_USE_MKFS`     | 1     | `format()`                                             |
| `FF_USE_LABEL`    | 1     | `getLabel()` / `setLabel()`                            |
| `FF_USE_CHMOD`    | 1     | `f_chmod` / `f_utime` available through FatFs directly |
| `FF_USE_STRFUNC`  | 1     | `f_gets` / `f_printf` available through FatFs directly |
| `FF_FS_EXFAT`     | 1     | Cards over 32 GB come formatted as exFAT               |
| `FF_FS_RPATH`     | 0     | No current-directory support needed                    |
| `FF_VOLUMES`      | 1     | One card                                               |
| `FF_FS_NORTC`     | 0     | Timestamps come from `get_fattime()` (`SDCard::fatTime`) |
| `FF_FS_LOCK`      | 8     | Refuses to open a file twice for writing (prevents corruption) |

FatFs license (BSD-style, 1 clause): see the header of `fatfs/ff.c`.

## Bench test

```cpp
bool SDCard_test(SDCard& sd, bool verbose = false, uint32_t speedKB = 512);
```

The test is non-destructive. It works in a `_sdtest/` folder and deletes it at the end. It checks
card init, the CSD capacity, raw sector reads (0x55AA signature), mount, mkdir, file write, read,
append, `readLine` with LF and CRLF, `printf`, `seek`, open-mode errors, rename and remove,
`nextFreeName`, 500-row CSV write and read-back (including a quoted field), CSV append, listing, and
recursive delete. It also prints write and read speeds. Enable it in `Main.cpp` with `RUN_SDCARD_TEST`.
