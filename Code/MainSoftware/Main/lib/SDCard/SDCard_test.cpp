#include "SDCard_test.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[SDCard] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

constexpr const char* DIR_ = "_sdtest";

// The test objects are large (CSVWriter ~5 kB); keep them off the 2 kB stack
CSVWriter g_csv;
CSVReader g_reader;
SDFile g_file;

uint32_t countEntries(SDCard& sd, const char* path) {
    uint32_t n = 0;
    sd.listDir(path, [](const SDCard::FileInfo&, void* u) { (*(uint32_t*)u)++; }, &n);
    return n;
}

} // namespace

bool SDCard_test(SDCard& sd, bool verbose, uint32_t speedKB) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== SDCard test =====\n");

    // --- Card detect / init (begin() sets up the pins, initializes the card and mounts) ---
    sd.end();
    bool begun = sd.begin();
    if (!check("card present (chip detect)", sd.cardPresent())) {
        printf("[SDCard] no card in the socket (or the CD polarity is wrong), aborting\n");
        return false;
    }
    if (!check("card init (CMD0/CMD8/ACMD41/CSD)", sd.cardInitialized())) {
        printf("[SDCard] card did not respond in SPI mode: check wiring, MISO pull-up and power, aborting\n");
        return false;
    }
    VLOG("  card: %s, %lu sectors, SPI %.1f MHz\n", SDCard::cardTypeName(sd.cardType()),
         (unsigned long)sd.sectorCount(), sd.spiFrequency() / 1e6);
    check("capacity read from CSD (> 1 MB)", sd.sectorCount() > 2048);

    // --- Raw sector 0 (MBR or boot sector ends with 0x55AA) ---
    uint8_t* sector = (uint8_t*)malloc(SDCard::SECTOR_SIZE * 2);
    bool rawOk = sector && sd.readSectors(0, sector, 1);
    check("read sector 0", rawOk);
    check("sector 0 signature 0x55AA", rawOk && sector[510] == 0x55 && sector[511] == 0xAA);
    check("multi-sector read (sectors 0-1)", sector && sd.readSectors(0, sector, 2));
    free(sector);

    // --- Filesystem ---
    if (!check("mount filesystem", begun && sd.isMounted())) {
        printf("[SDCard] mount failed: %s. Format the card as FAT32/exFAT (or call sd.format()), aborting\n",
               sd.lastError());
        return false;
    }
    if (verbose) sd.printInfo();
    check("filesystem type known", sd.fsType() != SDCard::FsType::None);
    check("free space > 1 MB", sd.freeBytes() > 1024 * 1024);

    // Clean up a previous, interrupted run
    if (sd.exists(DIR_)) sd.removeRecursive(DIR_);

    // --- Directories ---
    check("mkdir nested (_sdtest/a/b)", sd.mkdir("_sdtest/a/b"));
    check("mkdir existing is ok", sd.mkdir("_sdtest/a"));
    check("isDirectory", sd.isDirectory("_sdtest/a/b") && !sd.isDirectory("_sdtest/none"));

    // --- Whole-file write / read back ---
    const char* text = "hello from the flight computer\nline 2\r\nline 3";
    check("writeFile", sd.writeFile("_sdtest/hello.txt", text));
    check("fileSize matches", sd.fileSize("_sdtest/hello.txt") == (int64_t)strlen(text));
    char rb[96] = {0};
    int32_t n = sd.readFile("_sdtest/hello.txt", rb, sizeof rb - 1);
    check("readFile matches", n == (int32_t)strlen(text) && memcmp(rb, text, n) == 0);
    check("appendFile", sd.appendFile("_sdtest/hello.txt", "\nline 4"));
    check("size after append", sd.fileSize("_sdtest/hello.txt") == (int64_t)strlen(text) + 7);

    // --- SDFile: readLine / printf / seek ---
    bool linesOk = g_file.open("_sdtest/hello.txt", SDFile::Mode::Read);
    char line[48];
    const char* expect[] = {"hello from the flight computer", "line 2", "line 3", "line 4"};
    for (const char* e : expect) linesOk &= g_file.readLine(line, sizeof line) >= 0 && strcmp(line, e) == 0;
    linesOk &= g_file.readLine(line, sizeof line) == -1 && g_file.eof();
    g_file.close();
    check("readLine (LF and CRLF)", linesOk);

    bool pfOk = g_file.open("_sdtest/printf.txt", SDFile::Mode::Write);
    pfOk &= g_file.printf("%d,%.3f,%s", 42, 3.14159, "ok") == 11;  // "42,3.142,ok"
    pfOk &= g_file.seek(3) && g_file.read() == '3' && g_file.position() == 4;
    g_file.close();
    check("printf / seek / read", pfOk);

    check("open missing file fails", !g_file.open("_sdtest/missing.txt", SDFile::Mode::Read) &&
                                         g_file.lastResult() == FR_NO_FILE);
    check("CreateNew on existing fails", !g_file.open("_sdtest/printf.txt", SDFile::Mode::CreateNew));

    // --- Rename / delete ---
    check("rename", sd.rename("_sdtest/printf.txt", "_sdtest/a/moved.txt") && sd.exists("_sdtest/a/moved.txt") &&
                        !sd.exists("_sdtest/printf.txt"));
    check("remove", sd.remove("_sdtest/a/moved.txt") && !sd.exists("_sdtest/a/moved.txt"));
    check("remove non-empty dir fails", !sd.remove("_sdtest/a"));

    // --- nextFreeName ---
    char name[48];
    bool nfOk = sd.nextFreeName("_sdtest/logs/run_", ".csv", name, sizeof name) &&
                strcmp(name, "_sdtest/logs/run_000.csv") == 0;
    nfOk &= sd.writeFile(name, "x");
    nfOk &= sd.nextFreeName("_sdtest/logs/run_", ".csv", name, sizeof name) &&
            strcmp(name, "_sdtest/logs/run_001.csv") == 0;
    check("nextFreeName (creates folder, counts up)", nfOk);

    // --- CSV write + read back ---
    const uint32_t ROWS = 500;
    bool csvOk = g_csv.open("_sdtest/data.csv");
    csvOk &= g_csv.writeHeader({"i", "x", "label", "flag"});
    for (uint32_t i = 0; i < ROWS; i++) {
        g_csv.add(i).add(i * 0.5f, 2).add(i == 7 ? "has, comma \"q\"" : "row").add(i % 2 == 0);
        csvOk &= g_csv.endRow();
    }
    csvOk &= g_csv.rowCount() == ROWS && g_csv.errorCount() == 0;
    csvOk &= g_csv.close();
    check("CSVWriter 500 rows", csvOk);

    bool rdOk = g_reader.open("_sdtest/data.csv") && g_reader.readRow() && g_reader.fieldCount() == 4 &&
                g_reader.indexOf("label") == 2;
    uint32_t rows = 0;
    while (rdOk && g_reader.readRow()) {
        bool rowOk = g_reader.fieldCount() == 4 && g_reader.getInt(0, -1) == (long)rows &&
                     fabsf(g_reader.getFloat(1) - rows * 0.5f) < 0.006f &&
                     strcmp(g_reader.field(2), rows == 7 ? "has, comma \"q\"" : "row") == 0 &&
                     g_reader.getInt(3, -1) == (rows % 2 == 0);
        if (!rowOk) {
            printf("[SDCard] CSV row %lu mismatch: '%s' '%s' '%s' '%s'\n", (unsigned long)rows, g_reader.field(0),
                   g_reader.field(1), g_reader.field(2), g_reader.field(3));
            rdOk = false;
        }
        rows++;
    }
    g_reader.close();
    check("CSVReader reads back every row (incl. quoted field)", rdOk && rows == ROWS);

    bool apOk = g_csv.open("_sdtest/data.csv", /*append*/ true);
    apOk &= g_csv.writeHeader({"i", "x", "label", "flag"});  // must be skipped
    g_csv.add(ROWS).add(1.0f).add("last").add(true);
    apOk &= g_csv.endRow() && g_csv.close();
    uint32_t total = 0;
    apOk &= g_reader.open("_sdtest/data.csv");
    while (g_reader.readRow()) total++;
    g_reader.close();
    check("CSV append (header not repeated)", apOk && total == ROWS + 2);

    // --- Listing ---
    check("listDir counts entries", countEntries(sd, DIR_) == 4);  // a, hello.txt, logs, data.csv
    if (verbose) sd.printDir(DIR_, true);

    // --- Speed ---
    if (speedKB) {
        const size_t CHUNK = 4096;
        uint8_t* buf = (uint8_t*)malloc(CHUNK);
        bool spOk = buf && g_file.open("_sdtest/speed.bin", SDFile::Mode::Write);
        if (buf) for (size_t i = 0; i < CHUNK; i++) buf[i] = (uint8_t)i;
        uint32_t chunks = speedKB * 1024 / CHUNK;
        uint64_t t0 = time_us_64();
        for (uint32_t i = 0; spOk && i < chunks; i++) spOk &= g_file.write(buf, CHUNK) == CHUNK;
        spOk &= g_file.sync();
        uint64_t tw = time_us_64() - t0;
        g_file.close();

        spOk &= g_file.open("_sdtest/speed.bin", SDFile::Mode::Read);
        t0 = time_us_64();
        for (uint32_t i = 0; spOk && i < chunks; i++) {
            spOk &= g_file.read(buf, CHUNK) == (int32_t)CHUNK && buf[1] == 1 && buf[CHUNK - 1] == (uint8_t)(CHUNK - 1);
        }
        uint64_t tr = time_us_64() - t0;
        g_file.close();
        free(buf);
        check("speed test write + read back", spOk);
        if (verbose) printf("[SDCard] speed: write %.0f kB/s, read %.0f kB/s (%lu kB, 4 kB chunks, SPI %.1f MHz)\n",
               speedKB * 1e6 / (double)tw, speedKB * 1e6 / (double)tr, (unsigned long)speedKB,
               sd.spiFrequency() / 1e6);
    }

    // --- Cleanup ---
    check("removeRecursive cleans up", sd.removeRecursive(DIR_) && !sd.exists(DIR_));

    printf("[SDCard] %s: %d passed, %d failed\n", check.fail ? "FAIL" : "PASS", check.pass, check.fail);
    return check.fail == 0;
}
