/**
 * SDCard.h - microSD card driver + FAT/exFAT filesystem for the RP2040/RP2350 (Pico SDK), SPI mode.
 *
 * Classes:
 *   SDCard     the card itself: SPI protocol (SDv1, SDv2, SDHC/SDXC, MMC), mounting, formatting,
 *              and filesystem operations (mkdir, remove, rename, list, free space, ...).
 *   SDFile     one open file: read / write / printf / readLine / seek / sync.
 *   CSVWriter  buffered CSV logging: header, add(value) ... endRow(), automatic periodic sync.
 *   CSVReader  reads a CSV file back one row at a time and splits it into fields.
 *
 * The filesystem is ChaN's FatFs (lib/SDCard/fatfs, R0.15). Paths are plain "dir/file.csv" (no
 * drive prefix needed). Long file names are supported. FAT12/16/32 and exFAT cards all mount.
 *
 * Quick start:
 *   SDCard sd(SD_SPI, SD_CHIP_SELECT, SPI_SCK_SD, SPI_MOSI_SD, SPI_MISO_SD, SD_CHIP_DETECT);
 *   if (sd.begin()) {
 *       char name[32];
 *       sd.nextFreeName("flight_", ".csv", name, sizeof name);   // flight_000.csv, flight_001.csv ...
 *       static CSVWriter log;   // ~5 kB: make it global/static, never a local (2 kB stack)
 *       log.open(name);
 *       log.writeHeader({"t_ms", "ax", "ay", "az"});
 *       log.add(to_ms_since_boot(get_absolute_time())).add(ax, 3).add(ay, 3).add(az, 3).endRow();
 *       log.close();   // endRow() also syncs every second, so a power cut loses at most ~1 s
 *   }
 *
 * Only one SDCard can be mounted at a time (FatFs is configured for one volume).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <math.h>
#include <initializer_list>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ff.h"

class SDCard {
public:
    static constexpr uint NO_PIN = 0xFFFFFFFFu;
    static constexpr uint32_t SECTOR_SIZE = 512;

    enum class CardType : uint8_t {
        None = 0,
        MMC,   // MMC v3
        SDv1,  // SD v1.x, byte addressed, <= 2 GB
        SDv2,  // SD v2.0 standard capacity, byte addressed, <= 2 GB
        SDHC,  // SDHC / SDXC / SDUC, block addressed (> 2 GB)
    };

    enum class FsType : uint8_t { None = 0, FAT12, FAT16, FAT32, ExFAT };

    /** One directory entry, as returned by listDir(). */
    struct FileInfo {
        const char* name;   // long name (valid only during the callback)
        uint64_t size;      // bytes (0 for directories)
        bool isDir;
        bool readOnly, hidden;
        uint16_t year;      // last modified
        uint8_t month, day, hour, minute, second;
    };
    using ListCallback = void (*)(const FileInfo& info, void* user);

    /**
     * @param spi         SPI instance (spi0 / spi1)
     * @param csPin       card chip select GPIO
     * @param sckPin      SPI clock GPIO
     * @param mosiPin     SPI TX GPIO (card DI)
     * @param misoPin     SPI RX GPIO (card DO); the internal pull-up is enabled on it
     * @param cdPin       card-detect switch GPIO, or NO_PIN if unused
     * @param cdActiveLow true if the CD pin reads low with a card inserted (switch to GND, pull-up on)
     * @param spiHz       SPI clock after initialization (SD spec max 25 MHz)
     */
    SDCard(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin,
           uint cdPin = NO_PIN, bool cdActiveLow = true, uint32_t spiHz = 25'000'000);
    ~SDCard();

    SDCard(const SDCard&) = delete;
    SDCard& operator=(const SDCard&) = delete;

    // ---------------- Start / stop ----------------
    /** Sets up the pins and bus, initializes the card and mounts the filesystem. */
    bool begin(bool initBus = true);
    /** Flushes and unmounts. Open SDFile / CSVWriter objects must be closed first. */
    void end();

    /** True if the card-detect switch reports a card (always true when no CD pin is given). */
    bool cardPresent() const;
    /** Low-level card initialization only (no filesystem). Called by begin(). */
    bool initCard();
    bool cardInitialized() const { return type_ != CardType::None; }

    bool mount();
    bool unmount();
    bool isMounted() const { return mounted_; }

    /**
     * Erases the filesystem and creates a new, empty one (FAT32 by default, exFAT if requested).
     * Cards above 32 GB are normally exFAT. Takes a few seconds. The card stays mounted afterwards.
     */
    bool format(bool exfat = false, const char* label = nullptr);

    // ---------------- Card / volume information ----------------
    CardType cardType() const { return type_; }
    static const char* cardTypeName(CardType t);
    uint32_t sectorCount() const { return sectors_; }           // 512-byte sectors on the card
    uint64_t cardSizeBytes() const { return (uint64_t)sectors_ * SECTOR_SIZE; }
    FsType fsType() const;
    static const char* fsTypeName(FsType t);
    uint64_t totalBytes();                                      // filesystem capacity
    uint64_t freeBytes();                                       // scans the FAT the first time (slow on big FAT32 cards)
    bool getLabel(char* out, size_t len);
    bool setLabel(const char* label);
    /** Raw 16-byte CID / CSD registers (read during initCard()). */
    const uint8_t* cid() const { return cid_; }
    const uint8_t* csd() const { return csd_; }
    /** Prints card type, size, filesystem, label and free space to stdio. */
    void printInfo();

    // ---------------- Files and directories ----------------
    bool exists(const char* path);
    bool isDirectory(const char* path);
    /** File size in bytes, or -1 if it doesn't exist. */
    int64_t fileSize(const char* path);
    /** Deletes a file or an empty directory. */
    bool remove(const char* path);
    /** Deletes a file or a directory and everything in it. */
    bool removeRecursive(const char* path);
    /** Renames or moves (within the card). Fails if `to` exists. */
    bool rename(const char* from, const char* to);
    /** Creates a directory and any missing parents. Succeeds if it already exists. */
    bool mkdir(const char* path);
    /** Calls `cb` once per entry in `path` ("" or "/" for the root). Returns false on error. */
    bool listDir(const char* path, ListCallback cb, void* user = nullptr);
    /** Prints a directory listing to stdio (recursive if asked). */
    bool printDir(const char* path = "/", bool recursive = false);
    /**
     * Finds the first unused name of the form <prefix><NNN><suffix>, e.g. "logs/flight_007.csv".
     * @param digits zero-padded counter width (3 -> 000..999)
     */
    bool nextFreeName(const char* prefix, const char* suffix, char* out, size_t outLen, int digits = 3);

    /** Whole-file helpers. */
    bool writeFile(const char* path, const void* data, size_t len);   // create / overwrite
    bool appendFile(const char* path, const void* data, size_t len);
    bool writeFile(const char* path, const char* text);
    bool appendFile(const char* path, const char* text);
    /** Reads up to `maxLen` bytes. Returns bytes read, or -1 on error. Doesn't NUL-terminate. */
    int32_t readFile(const char* path, void* buf, size_t maxLen);

    // ---------------- Errors ----------------
    /** Result of the most recent filesystem call (FR_OK on success). */
    FRESULT lastResult() const { return lastResult_; }
    static const char* resultName(FRESULT r);
    const char* lastError() const { return resultName(lastResult_); }

    // ---------------- Timestamps ----------------
    /**
     * Sets the wall-clock time used for file create/modify stamps. The clock then runs from the
     * system timer. Without this, files are stamped 2026-01-01 00:00:00 + time since boot.
     */
    static void setDateTime(uint16_t year, uint8_t month, uint8_t day,
                            uint8_t hour, uint8_t minute, uint8_t second);
    /** FAT-packed current time (used by FatFs). */
    static uint32_t fatTime();

    // ---------------- Raw sector access (bypasses the filesystem) ----------------
    bool readSectors(uint32_t sector, uint8_t* buf, uint32_t count);
    bool writeSectors(uint32_t sector, const uint8_t* buf, uint32_t count);
    /** Waits until the card has finished any internal write. */
    bool waitReady(uint32_t timeoutMs = 500);
    /** Erase block (allocation unit) size in sectors, from the SD status / CSD (1 if unknown). */
    uint32_t eraseBlockSectors();

    // ---------------- Bus settings ----------------
    void setSpiFrequency(uint32_t hz) { spiHz_ = hz; }
    uint32_t spiFrequency() const { return actualHz_; }   // actual clock after init
    /** CRC on every command and data block (CMD59). Default on. Call before begin(). */
    void setCRC(bool on) { crc_ = on; }

    /** The card registered with FatFs (used by the diskio glue). */
    static SDCard* active() { return active_; }

private:
    friend class SDFile;

    void configureBus(uint32_t hz);
    void select();
    void deselect();
    uint8_t xfer(uint8_t b);
    bool waitNotBusy(uint32_t timeoutMs);
    uint8_t command(uint8_t cmd, uint32_t arg);
    uint8_t appCommand(uint8_t cmd, uint32_t arg);
    bool readBlock(uint8_t* buf, uint32_t len);
    bool writeBlock(const uint8_t* buf, uint8_t token);
    bool readRegister(uint8_t cmd, uint8_t* out16);
    static uint8_t crc7(const uint8_t* data, size_t len);
    static uint16_t crc16(const uint8_t* data, size_t len);
    bool fsOk(FRESULT r) { lastResult_ = r; return r == FR_OK; }
    bool removeTree(char* path, size_t cap);

    spi_inst_t* spi_;
    uint cs_, sck_, mosi_, miso_, cd_;
    bool cdActiveLow_;
    uint32_t spiHz_;
    uint32_t actualHz_ = 0;
    uint32_t busHz_ = 0;
    bool crc_ = true;

    CardType type_ = CardType::None;
    uint32_t sectors_ = 0;
    uint8_t cid_[16] = {0};
    uint8_t csd_[16] = {0};

    FATFS fs_;
    bool mounted_ = false;
    FRESULT lastResult_ = FR_OK;

    static SDCard* active_;
};

/**
 * One open file. Closing (or destroying) the object flushes it. Data only reaches the card's
 * directory entry on sync()/close(), so call sync() periodically while logging.
 */
class SDFile {
public:
    enum class Mode : uint8_t {
        Read,       // must exist
        Write,      // create or truncate
        Append,     // create or open at end
        ReadWrite,  // must exist, position 0
        CreateNew,  // fails if it exists
    };

    SDFile() = default;
    SDFile(const char* path, Mode mode) { open(path, mode); }
    ~SDFile() { close(); }
    SDFile(const SDFile&) = delete;
    SDFile& operator=(const SDFile&) = delete;

    bool open(const char* path, Mode mode = Mode::Read);
    bool close();
    bool isOpen() const { return open_; }
    explicit operator bool() const { return open_; }

    // ---------------- Writing ----------------
    /** Returns the number of bytes written (less than len on error / card full). */
    size_t write(const void* data, size_t len);
    size_t write(const char* text);
    size_t write(char c) { return write(&c, 1); }
    size_t print(const char* text) { return write(text); }
    size_t println(const char* text = "");
    size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    size_t vprintf(const char* fmt, va_list ap);

    // ---------------- Reading ----------------
    /** Returns bytes read (0 at end of file), or -1 on error. */
    int32_t read(void* buf, size_t len);
    /** Next byte, or -1 at end of file / error. */
    int read();
    /**
     * Reads one line into `buf` (without the "\n" / "\r\n"), always NUL-terminated.
     * Returns the line length, or -1 at end of file / error. Long lines are cut at len-1 and the
     * rest of the line is skipped.
     */
    int32_t readLine(char* buf, size_t len);

    // ---------------- Position ----------------
    bool seek(uint64_t pos);
    bool seekEnd() { return seek(size()); }
    bool rewind() { return seek(0); }
    uint64_t position() const;
    uint64_t size() const;
    bool eof() const;

    /** Flushes cached data and updates the directory entry (size / timestamp). */
    bool sync();
    /** Cuts the file at the current position. */
    bool truncate();

    FRESULT lastResult() const { return lastResult_; }
    FIL* raw() { return &fil_; }

private:
    bool ok(FRESULT r) { lastResult_ = r; return r == FR_OK; }
    FIL fil_;
    bool open_ = false;
    FRESULT lastResult_ = FR_OK;
};

#ifndef SD_CSV_BUFFER_SIZE
#define SD_CSV_BUFFER_SIZE 4096   // RAM buffer per CSVWriter; multiples of 512 write fastest
#endif

/**
 * Buffered CSV writer. Rows are built in RAM and written in large blocks, so a row costs only
 * a memcpy most of the time. Each call to endRow() may write the buffer to the card (when full)
 * and sync (every syncIntervalMs, default 1000 ms), so at most ~1 s of data is lost on power loss.
 *
 *   static CSVWriter log;   // global or static: it holds a 4 kB buffer
 *   log.open("data.csv");
 *   log.writeHeader({"time_s", "alt_m", "state"});
 *   log.add(t, 3).add(alt, 2).add("BOOST").endRow();
 */
class CSVWriter {
public:
    explicit CSVWriter(char delimiter = ',') : delim_(delimiter) {}
    ~CSVWriter() { close(); }
    CSVWriter(const CSVWriter&) = delete;
    CSVWriter& operator=(const CSVWriter&) = delete;

    /**
     * @param append  true: keep existing rows and add to the end (writeHeader() then does nothing
     *                if the file already has data); false: start a new, empty file.
     */
    bool open(const char* path, bool append = false);
    /** Flushes, syncs and closes. */
    bool close();
    bool isOpen() const { return file_.isOpen(); }

    // ---------------- Header ----------------
    bool writeHeader(std::initializer_list<const char*> columns);
    bool writeHeader(const char* const* columns, size_t count);
    /** A header line given as one string, e.g. "t,ax,ay,az" (no newline). */
    bool writeHeader(const char* line);

    // ---------------- Fields (chainable) ----------------
    CSVWriter& add(const char* text);    // quoted automatically if it contains , " or newline
    CSVWriter& add(char c);
    CSVWriter& add(bool b);              // 1 / 0
    CSVWriter& add(int32_t v);
    CSVWriter& add(uint32_t v);
    CSVWriter& add(int64_t v);
    CSVWriter& add(uint64_t v);
    CSVWriter& add(int v) { return add((int32_t)v); }
    CSVWriter& add(unsigned v) { return add((uint32_t)v); }
    /** NaN is written as an empty field. decimals < 0 uses the default (setDecimals). */
    CSVWriter& add(float v, int decimals = -1);
    CSVWriter& add(double v, int decimals = -1);
    /** printf-formatted field (not quoted). */
    CSVWriter& addf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    /** An empty field. */
    CSVWriter& skip();

    /** Ends the row. Returns false if the data could not be written to the card. */
    bool endRow();
    /** Writes a full row of floats in one call. */
    bool writeRow(const float* values, size_t count, int decimals = -1);
    /** Writes a pre-formatted line (newline added). */
    bool writeLine(const char* line);

    // ---------------- Flushing ----------------
    /** Writes the RAM buffer to the file (FatFs cache). */
    bool flush();
    /** flush() + update the directory entry, so the data survives power loss. */
    bool sync();

    // ---------------- Settings ----------------
    void setDecimals(int d) { decimals_ = d; }
    /** Sync at most this often from endRow() (0 = never automatically). Default 1000 ms. */
    void setSyncIntervalMs(uint32_t ms) { syncMs_ = ms; }
    /** Also sync every N rows (0 = off, default). */
    void setSyncEveryRows(uint32_t n) { syncRows_ = n; }
    void setDelimiter(char d) { delim_ = d; }

    // ---------------- Status ----------------
    uint32_t rowCount() const { return rows_; }          // rows written since open (not counting header)
    uint64_t bytesWritten() const { return bytes_; }
    uint32_t errorCount() const { return errors_; }      // failed writes / overflowing rows
    SDFile& file() { return file_; }

private:
    bool put(const char* s, size_t n);
    void beginField();
    void appendRow(const char* s, size_t n);
    bool commitRow();
    void afterRow();

    SDFile file_;
    char buf_[SD_CSV_BUFFER_SIZE];
    size_t len_ = 0;          // committed bytes in buf_
    char row_[512];           // row being built
    size_t rowLen_ = 0;
    bool rowOverflow_ = false;
    bool firstField_ = true;
    bool hasData_ = false;    // file already had content when opened
    char delim_;
    int decimals_ = 4;
    uint32_t syncMs_ = 1000;
    uint32_t syncRows_ = 0;
    uint32_t rowsSinceSync_ = 0;
    absolute_time_t lastSync_ = nil_time;
    uint32_t rows_ = 0;
    uint64_t bytes_ = 0;
    uint32_t errors_ = 0;
};

/**
 * Reads a CSV file row by row. Handles quoted fields ("a, b" and "" escapes) that don't span lines.
 *
 *   static CSVReader csv;   // ~1.1 kB
 *   csv.open("config.csv");
 *   csv.readRow();                         // header
 *   while (csv.readRow()) { float x = csv.getFloat(1); }
 */
class CSVReader {
public:
    static constexpr size_t MAX_FIELDS = 32;
    static constexpr size_t LINE_LEN = 512;

    explicit CSVReader(char delimiter = ',') : delim_(delimiter) {}
    bool open(const char* path);
    bool close() { return file_.close(); }
    bool isOpen() const { return file_.isOpen(); }

    /** Reads and splits the next non-empty line. False at end of file. */
    bool readRow();
    size_t fieldCount() const { return count_; }
    /** Field text ("" if out of range). */
    const char* field(size_t i) const { return i < count_ ? fields_[i] : ""; }
    /** Index of a column name in the current row (use on the header row), or -1. */
    int indexOf(const char* name) const;
    float getFloat(size_t i, float fallback = NAN) const;
    double getDouble(size_t i, double fallback = NAN) const;
    long getInt(size_t i, long fallback = 0) const;
    uint32_t lineNumber() const { return line_; }

private:
    SDFile file_;
    char buf_[LINE_LEN];
    char* fields_[MAX_FIELDS];
    size_t count_ = 0;
    uint32_t line_ = 0;
    char delim_;
};
