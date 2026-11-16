#include "SDCard.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "diskio.h"

// SD commands used in SPI mode
namespace {
constexpr uint8_t CMD0   = 0;   // GO_IDLE_STATE
constexpr uint8_t CMD1   = 1;   // SEND_OP_COND (MMC)
constexpr uint8_t CMD8   = 8;   // SEND_IF_COND
constexpr uint8_t CMD9   = 9;   // SEND_CSD
constexpr uint8_t CMD10  = 10;  // SEND_CID
constexpr uint8_t CMD12  = 12;  // STOP_TRANSMISSION
constexpr uint8_t CMD16  = 16;  // SET_BLOCKLEN
constexpr uint8_t CMD17  = 17;  // READ_SINGLE_BLOCK
constexpr uint8_t CMD18  = 18;  // READ_MULTIPLE_BLOCK
constexpr uint8_t CMD24  = 24;  // WRITE_BLOCK
constexpr uint8_t CMD25  = 25;  // WRITE_MULTIPLE_BLOCK
constexpr uint8_t CMD55  = 55;  // APP_CMD
constexpr uint8_t CMD58  = 58;  // READ_OCR
constexpr uint8_t CMD59  = 59;  // CRC_ON_OFF
constexpr uint8_t ACMD13 = 13;  // SD_STATUS
constexpr uint8_t ACMD23 = 23;  // SET_WR_BLK_ERASE_COUNT
constexpr uint8_t ACMD41 = 41;  // SD_SEND_OP_COND

constexpr uint8_t TOKEN_START_BLOCK = 0xFE;  // single read/write, multi read
constexpr uint8_t TOKEN_MULTI_WRITE = 0xFC;
constexpr uint8_t TOKEN_STOP_TRAN   = 0xFD;

constexpr uint32_t INIT_HZ = 400'000;
constexpr uint32_t INIT_TIMEOUT_MS = 1000;
constexpr uint32_t READ_TOKEN_TIMEOUT_MS = 200;
constexpr uint32_t WRITE_BUSY_TIMEOUT_MS = 500;
constexpr uint32_t STOP_BUSY_TIMEOUT_MS = 1000;

inline uint32_t nowMs() { return to_ms_since_boot(get_absolute_time()); }
} // namespace

SDCard* SDCard::active_ = nullptr;

SDCard::SDCard(spi_inst_t* spi, uint csPin, uint sckPin, uint mosiPin, uint misoPin, uint cdPin,
               bool cdActiveLow, uint32_t spiHz)
    : spi_(spi), cs_(csPin), sck_(sckPin), mosi_(mosiPin), miso_(misoPin), cd_(cdPin),
      cdActiveLow_(cdActiveLow), spiHz_(spiHz) {
    memset(&fs_, 0, sizeof fs_);
}

SDCard::~SDCard() {
    end();
}

// ============================================================================
// Bus helpers
// ============================================================================
void SDCard::configureBus(uint32_t hz) {
    // Set every transaction so other devices on the same SPI bus can use other speeds/modes.
    actualHz_ = spi_set_baudrate(spi_, hz);
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

uint8_t SDCard::xfer(uint8_t b) {
    uint8_t r = 0xFF;
    spi_write_read_blocking(spi_, &b, &r, 1);
    return r;
}

void SDCard::select() {
    configureBus(busHz_);
    gpio_put(cs_, 0);
    xfer(0xFF);  // a few clocks so the card sees CS before the command
}

void SDCard::deselect() {
    gpio_put(cs_, 1);
    xfer(0xFF);  // the card only releases DO (MISO) after a clock with CS high
}

bool SDCard::waitNotBusy(uint32_t timeoutMs) {
    uint32_t start = nowMs();
    do {
        if (xfer(0xFF) == 0xFF) return true;
    } while (nowMs() - start < timeoutMs);
    return false;
}

uint8_t SDCard::crc7(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t d = data[i];
        for (int b = 0; b < 8; b++) {
            crc <<= 1;
            if ((d ^ crc) & 0x80) crc ^= 0x09;
            d <<= 1;
        }
    }
    return crc & 0x7F;
}

uint16_t SDCard::crc16(const uint8_t* data, size_t len) {
    // CRC-16-CCITT (XModem), polynomial 0x1021, init 0 - the SD data CRC
    static uint16_t table[256];
    static bool ready = false;
    if (!ready) {
        for (int i = 0; i < 256; i++) {
            uint16_t c = (uint16_t)(i << 8);
            for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
            table[i] = c;
        }
        ready = true;
    }
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) crc = (uint16_t)((crc << 8) ^ table[((crc >> 8) ^ data[i]) & 0xFF]);
    return crc;
}

/**
 * Sends a command and returns its R1 response (0xFF on timeout). Leaves CS asserted so the
 * caller can read any extra response bytes / data; the caller must deselect().
 */
uint8_t SDCard::command(uint8_t cmd, uint32_t arg) {
    if (cmd != CMD12) {
        deselect();
        select();
        // CMD0 is sent before the card is in SPI mode, so don't wait for "ready" on it
        if (cmd != CMD0 && !waitNotBusy(WRITE_BUSY_TIMEOUT_MS)) return 0xFF;
    }

    uint8_t frame[6] = {
        (uint8_t)(0x40 | cmd), (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg, 0,
    };
    frame[5] = (uint8_t)((crc7(frame, 5) << 1) | 1);
    spi_write_blocking(spi_, frame, 6);

    if (cmd == CMD12) xfer(0xFF);  // stuff byte after STOP_TRANSMISSION

    uint8_t r = 0xFF;
    for (int i = 0; i < 10; i++) {
        r = xfer(0xFF);
        if (!(r & 0x80)) break;
    }
    return r;
}

uint8_t SDCard::appCommand(uint8_t cmd, uint32_t arg) {
    uint8_t r = command(CMD55, 0);
    if (r > 1) return r;
    return command(cmd, arg);
}

bool SDCard::readBlock(uint8_t* buf, uint32_t len) {
    uint32_t start = nowMs();
    uint8_t token;
    do {
        token = xfer(0xFF);
    } while (token == 0xFF && nowMs() - start < READ_TOKEN_TIMEOUT_MS);
    if (token != TOKEN_START_BLOCK) return false;

    spi_read_blocking(spi_, 0xFF, buf, len);
    uint16_t crc = (uint16_t)(xfer(0xFF) << 8);
    crc |= xfer(0xFF);
    return !crc_ || crc == crc16(buf, len);
}

bool SDCard::writeBlock(const uint8_t* buf, uint8_t token) {
    if (!waitNotBusy(WRITE_BUSY_TIMEOUT_MS)) return false;
    xfer(token);
    if (token == TOKEN_STOP_TRAN) return true;

    spi_write_blocking(spi_, buf, SECTOR_SIZE);
    uint16_t crc = crc_ ? crc16(buf, SECTOR_SIZE) : 0xFFFF;
    xfer((uint8_t)(crc >> 8));
    xfer((uint8_t)crc);
    uint8_t resp = xfer(0xFF);
    return (resp & 0x1F) == 0x05;  // data accepted
}

bool SDCard::readRegister(uint8_t cmd, uint8_t* out16) {
    bool ok = command(cmd, 0) == 0 && readBlock(out16, 16);
    deselect();
    return ok;
}

// ============================================================================
// Initialization
// ============================================================================
bool SDCard::begin(bool initBus) {
    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);

    if (cd_ != NO_PIN) {
        gpio_init(cd_);
        gpio_set_dir(cd_, GPIO_IN);
        if (cdActiveLow_) gpio_pull_up(cd_);
        else gpio_pull_down(cd_);
        sleep_us(50);  // let the pull settle before cardPresent()
    }

    if (initBus) {
        if (!(spi_get_const_hw(spi_)->cr1 & SPI_SSPCR1_SSE_BITS)) {
            spi_init(spi_, INIT_HZ);
        }
        gpio_set_function(sck_, GPIO_FUNC_SPI);
        gpio_set_function(mosi_, GPIO_FUNC_SPI);
        gpio_set_function(miso_, GPIO_FUNC_SPI);
        gpio_pull_up(miso_);  // SD DO is open-drain-ish during init and needs a pull-up
    }

    if (!initCard()) return false;
    return mount();
}

void SDCard::end() {
    if (mounted_) unmount();
    type_ = CardType::None;
}

bool SDCard::cardPresent() const {
    if (cd_ == NO_PIN) return true;
    return gpio_get(cd_) == !cdActiveLow_;
}

bool SDCard::initCard() {
    type_ = CardType::None;
    sectors_ = 0;
    if (!cardPresent()) return false;

    sleep_ms(1);  // card needs >= 1 ms after power-up
    busHz_ = INIT_HZ;
    configureBus(busHz_);

    // >= 74 clocks with CS and DI high puts the card in a known state
    gpio_put(cs_, 1);
    for (int i = 0; i < 10; i++) xfer(0xFF);

    // CMD0: enter SPI mode / idle
    uint8_t r = 0xFF;
    for (int i = 0; i < 10 && r != 0x01; i++) {
        r = command(CMD0, 0);
        if (r != 0x01) {
            deselect();
            sleep_ms(10);
        }
    }
    if (r != 0x01) {
        deselect();
        return false;
    }

    bool crcOn = crc_;
    CardType t = CardType::None;
    uint32_t start = nowMs();

    r = command(CMD8, 0x1AA);
    if (r == 0x01) {
        // SD v2+: check the echoed voltage range / pattern
        uint8_t r7[4];
        for (uint8_t& b : r7) b = xfer(0xFF);
        if ((r7[2] & 0x0F) == 0x01 && r7[3] == 0xAA) {
            if (crcOn && command(CMD59, 1) > 1) crcOn = false;
            while ((r = appCommand(ACMD41, 1u << 30)) != 0 && nowMs() - start < INIT_TIMEOUT_MS) {
                sleep_ms(1);
            }
            if (r == 0 && command(CMD58, 0) == 0) {
                uint8_t ocr[4];
                for (uint8_t& b : ocr) b = xfer(0xFF);
                t = (ocr[0] & 0x40) ? CardType::SDHC : CardType::SDv2;  // CCS bit
            }
        }
    } else {
        // SD v1 or MMC v3
        if (crcOn && command(CMD59, 1) > 1) crcOn = false;
        uint8_t initCmd;
        bool app;
        if (appCommand(ACMD41, 0) <= 1) {
            t = CardType::SDv1;
            initCmd = ACMD41;
            app = true;
        } else {
            t = CardType::MMC;
            initCmd = CMD1;
            app = false;
        }
        while ((r = app ? appCommand(initCmd, 0) : command(initCmd, 0)) != 0 &&
               nowMs() - start < INIT_TIMEOUT_MS) {
            sleep_ms(1);
        }
        if (r != 0 || command(CMD16, SECTOR_SIZE) != 0) t = CardType::None;
    }
    deselect();
    crc_ = crcOn;
    if (t == CardType::None) return false;

    // Full speed from here on
    busHz_ = spiHz_;
    configureBus(busHz_);

    if (!readRegister(CMD9, csd_)) return false;
    readRegister(CMD10, cid_);

    // Capacity from the CSD
    uint8_t csdVer = csd_[0] >> 6;
    if (csdVer == 1) {  // CSD 2.0 (SDHC/SDXC)
        uint32_t cSize = ((uint32_t)(csd_[7] & 0x3F) << 16) | ((uint32_t)csd_[8] << 8) | csd_[9];
        sectors_ = (cSize + 1) << 10;
    } else if (csdVer == 2) {  // CSD 3.0 (SDUC): more than 2 TB, clamp to what FatFs can address
        sectors_ = 0xFFFFFFFFu;
    } else {  // CSD 1.0 (SDSC / MMC)
        uint32_t readBlLen = csd_[5] & 0x0F;
        uint32_t cSize = ((uint32_t)(csd_[6] & 0x03) << 10) | ((uint32_t)csd_[7] << 2) | (csd_[8] >> 6);
        uint32_t cSizeMult = ((csd_[9] & 0x03) << 1) | (csd_[10] >> 7);
        sectors_ = (cSize + 1) << (cSizeMult + 2 + readBlLen - 9);
    }

    type_ = t;
    return true;
}

uint32_t SDCard::eraseBlockSectors() {
    if (type_ == CardType::SDHC) {
        // AU_SIZE from the 64-byte SD status
        uint8_t status[64];
        bool ok = false;
        if (appCommand(ACMD13, 0) == 0) {
            xfer(0xFF);  // second byte of the R2 response
            ok = readBlock(status, 64);
        }
        deselect();
        if (ok) {
            uint32_t au = status[10] >> 4;
            if (au) return 16u << au;  // 16 sectors (8 kB) << AU_SIZE
        }
        return 1;
    }
    if (type_ == CardType::SDv1 || type_ == CardType::SDv2) {
        uint32_t sectorSize = (((csd_[10] & 0x3F) << 1) | (csd_[11] >> 7)) + 1;
        return sectorSize << ((csd_[13] >> 6) - 1);
    }
    return 1;
}

// ============================================================================
// Raw sectors
// ============================================================================
bool SDCard::readSectors(uint32_t sector, uint8_t* buf, uint32_t count) {
    if (type_ == CardType::None || count == 0) return false;
    uint32_t addr = (type_ == CardType::SDHC) ? sector : sector * SECTOR_SIZE;

    if (count == 1) {
        if (command(CMD17, addr) == 0 && readBlock(buf, SECTOR_SIZE)) count = 0;
    } else if (command(CMD18, addr) == 0) {
        do {
            if (!readBlock(buf, SECTOR_SIZE)) break;
            buf += SECTOR_SIZE;
        } while (--count);
        command(CMD12, 0);
    }
    deselect();
    return count == 0;
}

bool SDCard::writeSectors(uint32_t sector, const uint8_t* buf, uint32_t count) {
    if (type_ == CardType::None || count == 0) return false;
    uint32_t addr = (type_ == CardType::SDHC) ? sector : sector * SECTOR_SIZE;

    if (count == 1) {
        if (command(CMD24, addr) == 0 && writeBlock(buf, TOKEN_START_BLOCK)) count = 0;
    } else {
        if (type_ != CardType::MMC) appCommand(ACMD23, count);  // pre-erase hint, speeds up the write
        if (command(CMD25, addr) == 0) {
            do {
                if (!writeBlock(buf, TOKEN_MULTI_WRITE)) break;
                buf += SECTOR_SIZE;
            } while (--count);
            if (!writeBlock(nullptr, TOKEN_STOP_TRAN)) count = 1;
            xfer(0xFF);  // one byte before busy is signalled after the stop token
        }
    }
    // Wait for the card to finish programming so a following power loss can't lose this write
    bool ok = waitNotBusy(STOP_BUSY_TIMEOUT_MS) && count == 0;
    deselect();
    return ok;
}

bool SDCard::waitReady(uint32_t timeoutMs) {
    if (type_ == CardType::None) return false;
    select();
    bool ok = waitNotBusy(timeoutMs);
    deselect();
    return ok;
}

// ============================================================================
// Filesystem
// ============================================================================
bool SDCard::mount() {
    if (type_ == CardType::None && !initCard()) {
        lastResult_ = FR_NOT_READY;
        return false;
    }
    if (active_ && active_ != this && active_->mounted_) active_->unmount();
    active_ = this;
    mounted_ = fsOk(f_mount(&fs_, "", 1));
    return mounted_;
}

bool SDCard::unmount() {
    bool ok = true;
    if (active_ == this) {
        ok = fsOk(f_mount(nullptr, "", 0));
        active_ = nullptr;
    }
    mounted_ = false;
    return ok;
}

bool SDCard::format(bool exfat, const char* label) {
    if (type_ == CardType::None && !initCard()) {
        lastResult_ = FR_NOT_READY;
        return false;
    }
    if (mounted_) unmount();
    if (active_ && active_ != this && active_->mounted_) active_->unmount();
    active_ = this;

    // Bigger work buffer = fewer, larger writes while formatting
    UINT workLen = 16 * SECTOR_SIZE;
    uint8_t* work = (uint8_t*)malloc(workLen);
    if (!work) {
        workLen = SECTOR_SIZE;
        work = (uint8_t*)malloc(workLen);
    }
    if (!work) {
        lastResult_ = FR_NOT_ENOUGH_CORE;
        return false;
    }
    MKFS_PARM opt = {(BYTE)(exfat ? FM_EXFAT : FM_FAT32), 0, 0, 0, 0};
    bool ok = fsOk(f_mkfs("", &opt, work, workLen));
    free(work);
    if (!ok) return false;

    if (!mount()) return false;
    if (label && *label) setLabel(label);
    return true;
}

SDCard::FsType SDCard::fsType() const {
    if (!mounted_) return FsType::None;
    switch (fs_.fs_type) {
        case FS_FAT12: return FsType::FAT12;
        case FS_FAT16: return FsType::FAT16;
        case FS_FAT32: return FsType::FAT32;
        case FS_EXFAT: return FsType::ExFAT;
        default: return FsType::None;
    }
}

const char* SDCard::fsTypeName(FsType t) {
    switch (t) {
        case FsType::FAT12: return "FAT12";
        case FsType::FAT16: return "FAT16";
        case FsType::FAT32: return "FAT32";
        case FsType::ExFAT: return "exFAT";
        default: return "none";
    }
}

const char* SDCard::cardTypeName(CardType t) {
    switch (t) {
        case CardType::MMC: return "MMC";
        case CardType::SDv1: return "SDv1";
        case CardType::SDv2: return "SDv2 (SDSC)";
        case CardType::SDHC: return "SDHC/SDXC";
        default: return "none";
    }
}

uint64_t SDCard::totalBytes() {
    if (!mounted_) return 0;
    return (uint64_t)(fs_.n_fatent - 2) * fs_.csize * SECTOR_SIZE;
}

uint64_t SDCard::freeBytes() {
    if (!mounted_) return 0;
    DWORD freeClusters = 0;
    FATFS* fs = nullptr;
    if (!fsOk(f_getfree("", &freeClusters, &fs))) return 0;
    return (uint64_t)freeClusters * fs->csize * SECTOR_SIZE;
}

bool SDCard::getLabel(char* out, size_t len) {
    if (!out || len == 0) return false;
    char tmp[36] = {0};  // exFAT labels can be up to 11 UTF-16 chars
    DWORD vsn = 0;
    bool ok = fsOk(f_getlabel("", tmp, &vsn));
    snprintf(out, len, "%s", ok ? tmp : "");
    return ok;
}

bool SDCard::setLabel(const char* label) {
    return fsOk(f_setlabel(label ? label : ""));
}

void SDCard::printInfo() {
    printf("SD card: %s, %.2f GB, SPI %.1f MHz, CRC %s\n", cardTypeName(type_),
           cardSizeBytes() / 1e9, actualHz_ / 1e6, crc_ ? "on" : "off");
    if (type_ != CardType::None) {
        // CID: manufacturer ID, OEM, product name, revision, serial
        printf("  CID: MID 0x%02X  OEM %c%c  name %.5s  rev %u.%u  serial %08lX\n", cid_[0], cid_[1], cid_[2],
               (const char*)&cid_[3], cid_[8] >> 4, cid_[8] & 0x0F,
               (unsigned long)(((uint32_t)cid_[9] << 24) | (cid_[10] << 16) | (cid_[11] << 8) | cid_[12]));
    }
    if (!mounted_) {
        printf("  filesystem: not mounted (%s)\n", lastError());
        return;
    }
    char label[36];
    getLabel(label, sizeof label);
    uint64_t total = totalBytes(), avail = freeBytes();
    printf("  filesystem: %s, label \"%s\", cluster %lu B, %.2f GB total, %.2f GB free\n",
           fsTypeName(fsType()), label, (unsigned long)(fs_.csize * SECTOR_SIZE), total / 1e9, avail / 1e9);
}

// ============================================================================
// Files and directories
// ============================================================================
namespace {
bool isRoot(const char* path) {
    return !path || !*path || (path[0] == '/' && !path[1]);
}
} // namespace

bool SDCard::exists(const char* path) {
    if (isRoot(path)) return mounted_;
    FILINFO fi;
    return fsOk(f_stat(path, &fi));
}

bool SDCard::isDirectory(const char* path) {
    if (isRoot(path)) return mounted_;
    FILINFO fi;
    return fsOk(f_stat(path, &fi)) && (fi.fattrib & AM_DIR);
}

int64_t SDCard::fileSize(const char* path) {
    FILINFO fi;
    if (!fsOk(f_stat(path, &fi))) return -1;
    return (int64_t)fi.fsize;
}

bool SDCard::remove(const char* path) {
    return fsOk(f_unlink(path));
}

bool SDCard::rename(const char* from, const char* to) {
    return fsOk(f_rename(from, to));
}

bool SDCard::mkdir(const char* path) {
    if (isRoot(path)) return mounted_;
    char tmp[FF_MAX_LFN + 1];
    size_t n = strlen(path);
    if (n >= sizeof tmp) {
        lastResult_ = FR_INVALID_NAME;
        return false;
    }
    memcpy(tmp, path, n + 1);
    while (n > 1 && tmp[n - 1] == '/') tmp[--n] = 0;  // trailing slashes

    // Create every parent in turn
    for (size_t i = 1; i <= n; i++) {
        if (tmp[i] != '/' && tmp[i] != 0) continue;
        char saved = tmp[i];
        tmp[i] = 0;
        FRESULT r = f_mkdir(tmp);
        tmp[i] = saved;
        if (r != FR_OK && r != FR_EXIST) return fsOk(r);
    }
    return isDirectory(tmp);
}

bool SDCard::listDir(const char* path, ListCallback cb, void* user) {
    // DIR + FILINFO are ~330 bytes; keep them off the (small) stack so callbacks can recurse
    struct Ctx { DIR dir; FILINFO fi; };
    Ctx* c = new Ctx;
    bool ok = fsOk(f_opendir(&c->dir, isRoot(path) ? "" : path));
    while (ok) {
        FRESULT r = f_readdir(&c->dir, &c->fi);
        if (r != FR_OK) {
            ok = fsOk(r);
            break;
        }
        if (!c->fi.fname[0]) break;  // end of directory
        if (cb) {
            FileInfo info;
            info.name = c->fi.fname;
            info.size = c->fi.fsize;
            info.isDir = c->fi.fattrib & AM_DIR;
            info.readOnly = c->fi.fattrib & AM_RDO;
            info.hidden = c->fi.fattrib & AM_HID;
            info.year = (uint16_t)((c->fi.fdate >> 9) + 1980);
            info.month = (c->fi.fdate >> 5) & 0x0F;
            info.day = c->fi.fdate & 0x1F;
            info.hour = c->fi.ftime >> 11;
            info.minute = (c->fi.ftime >> 5) & 0x3F;
            info.second = (c->fi.ftime & 0x1F) * 2;
            cb(info, user);
        }
    }
    f_closedir(&c->dir);
    delete c;
    return ok;
}

namespace {
struct PrintCtx {
    SDCard* sd;
    char* path;      // shared buffer, current directory path
    size_t cap;
    int depth;
    bool recursive;
};

void printEntry(const SDCard::FileInfo& f, void* user) {
    PrintCtx* p = (PrintCtx*)user;
    printf("  %04u-%02u-%02u %02u:%02u  ", f.year, f.month, f.day, f.hour, f.minute);
    if (f.isDir) printf("%12s  ", "<DIR>");
    else printf("%12llu  ", (unsigned long long)f.size);
    printf("%*s%s%s\n", p->depth * 2, "", f.name, f.isDir ? "/" : "");

    if (f.isDir && p->recursive) {
        size_t len = strlen(p->path);
        int n = snprintf(p->path + len, p->cap - len, "%s%s", len ? "/" : "", f.name);
        if (n > 0 && (size_t)n < p->cap - len) {
            p->depth++;
            p->sd->listDir(p->path, printEntry, p);
            p->depth--;
        }
        p->path[len] = 0;
    }
}
} // namespace

bool SDCard::printDir(const char* path, bool recursive) {
    char* buf = new char[FF_MAX_LFN + 1];
    snprintf(buf, FF_MAX_LFN + 1, "%s", isRoot(path) ? "" : path);
    PrintCtx ctx = {this, buf, FF_MAX_LFN + 1, 0, recursive};
    printf("Directory of /%s\n", buf);
    bool ok = listDir(buf, printEntry, &ctx);
    if (!ok) printf("  (error: %s)\n", lastError());
    delete[] buf;
    return ok;
}

bool SDCard::removeRecursive(const char* path) {
    if (isRoot(path)) {
        lastResult_ = FR_INVALID_NAME;
        return false;
    }
    if (!isDirectory(path)) return remove(path);
    char* buf = new char[FF_MAX_LFN + 1];
    snprintf(buf, FF_MAX_LFN + 1, "%s", path);
    bool ok = removeTree(buf, FF_MAX_LFN + 1);
    delete[] buf;
    return ok;
}

namespace {
/** Puts the first entry of directory `path` into `name`. Returns 1 found, 0 empty, -1 error. */
int firstEntry(const char* path, char* name, size_t cap, bool& isDir, FRESULT& res) {
    struct Ctx { DIR dir; FILINFO fi; };
    Ctx* c = new Ctx;
    int found = -1;
    res = f_opendir(&c->dir, path);
    if (res == FR_OK) {
        res = f_readdir(&c->dir, &c->fi);
        if (res == FR_OK) {
            found = c->fi.fname[0] ? 1 : 0;
            snprintf(name, cap, "%s", c->fi.fname);
            isDir = c->fi.fattrib & AM_DIR;
        }
        f_closedir(&c->dir);
    }
    delete c;
    return found;
}
} // namespace

bool SDCard::removeTree(char* path, size_t cap) {
    // Repeatedly delete the first entry until the directory is empty. No DIR stays open while
    // recursing, so the depth is limited only by the path length.
    size_t len = strlen(path);
    while (true) {
        bool isDir = false;
        FRESULT res;
        if (len + 2 >= cap) {
            lastResult_ = FR_INVALID_NAME;
            return false;
        }
        path[len] = 0;
        // Opens "path" and writes the entry name just past its terminator, ready to be joined
        int f = firstEntry(path, path + len + 1, cap - len - 1, isDir, res);
        if (f < 0) return fsOk(res);
        if (f == 0) break;
        path[len] = '/';
        bool ok = isDir ? removeTree(path, cap) : fsOk(f_unlink(path));
        path[len] = 0;
        if (!ok) return false;
    }
    return fsOk(f_unlink(path));
}

bool SDCard::nextFreeName(const char* prefix, const char* suffix, char* out, size_t outLen, int digits) {
    if (!out || outLen == 0) return false;
    if (digits < 1) digits = 1;
    if (digits > 9) digits = 9;

    // Make sure the folder part of the prefix exists
    const char* slash = strrchr(prefix, '/');
    if (slash && slash != prefix) {
        char dir[FF_MAX_LFN + 1];
        size_t n = (size_t)(slash - prefix);
        if (n < sizeof dir) {
            memcpy(dir, prefix, n);
            dir[n] = 0;
            if (!mkdir(dir)) return false;
        }
    }

    uint32_t limit = 1;
    for (int i = 0; i < digits; i++) limit *= 10;
    for (uint32_t i = 0; i < limit; i++) {
        int n = snprintf(out, outLen, "%s%0*lu%s", prefix, digits, (unsigned long)i, suffix ? suffix : "");
        if (n < 0 || (size_t)n >= outLen) {
            lastResult_ = FR_INVALID_NAME;
            return false;
        }
        FILINFO fi;
        FRESULT r = f_stat(out, &fi);
        if (r == FR_NO_FILE) {
            lastResult_ = FR_OK;
            return true;
        }
        if (r != FR_OK) return fsOk(r);
    }
    lastResult_ = FR_EXIST;
    return false;
}

namespace {
// SDFile holds a 512-byte sector buffer, so the helpers allocate it instead of using the stack
bool putFile(const char* path, const void* data, size_t len, SDFile::Mode mode, FRESULT& res) {
    SDFile* f = new SDFile;
    bool ok = f->open(path, mode);
    if (ok) {
        ok = f->write(data, len) == len;
        FRESULT wr = f->lastResult();
        ok &= f->close();
        res = (wr != FR_OK) ? wr : f->lastResult();
    } else {
        res = f->lastResult();
    }
    delete f;
    return ok;
}
} // namespace

bool SDCard::writeFile(const char* path, const void* data, size_t len) {
    return putFile(path, data, len, SDFile::Mode::Write, lastResult_);
}

bool SDCard::appendFile(const char* path, const void* data, size_t len) {
    return putFile(path, data, len, SDFile::Mode::Append, lastResult_);
}

bool SDCard::writeFile(const char* path, const char* text) {
    return writeFile(path, text, strlen(text));
}

bool SDCard::appendFile(const char* path, const char* text) {
    return appendFile(path, text, strlen(text));
}

int32_t SDCard::readFile(const char* path, void* buf, size_t maxLen) {
    SDFile* f = new SDFile;
    int32_t n = f->open(path, SDFile::Mode::Read) ? f->read(buf, maxLen) : -1;
    lastResult_ = f->lastResult();
    delete f;
    return n;
}

const char* SDCard::resultName(FRESULT r) {
    static const char* const names[] = {
        "OK", "disk error", "internal error", "not ready", "no such file", "no such path",
        "invalid name", "denied (full or read-only)", "already exists", "invalid object",
        "write protected", "invalid drive", "not mounted", "no filesystem", "mkfs aborted",
        "timeout", "locked", "out of memory", "too many open files", "invalid parameter",
    };
    return (unsigned)r < sizeof names / sizeof names[0] ? names[r] : "unknown";
}

// ============================================================================
// Clock for file timestamps
// ============================================================================
namespace {
int64_t g_baseEpoch = 1767225600;  // 2026-01-01 00:00:00 UTC
uint64_t g_baseUs = 0;

// Howard Hinnant's civil-date algorithms
int64_t daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

void civilFromDays(int64_t z, int& y, unsigned& m, unsigned& d) {
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    y = (int)(yoe + era * 400);
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y += m <= 2;
}
} // namespace

void SDCard::setDateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute,
                         uint8_t second) {
    g_baseEpoch = daysFromCivil(year, month, day) * 86400 + hour * 3600 + minute * 60 + second;
    g_baseUs = time_us_64();
}

uint32_t SDCard::fatTime() {
    int64_t t = g_baseEpoch + (int64_t)((time_us_64() - g_baseUs) / 1000000);
    int64_t days = t / 86400;
    int64_t secs = t % 86400;
    int y;
    unsigned m, d;
    civilFromDays(days, y, m, d);
    if (y < 1980) y = 1980;
    if (y > 2107) y = 2107;
    return ((uint32_t)(y - 1980) << 25) | ((uint32_t)m << 21) | ((uint32_t)d << 16) |
           ((uint32_t)(secs / 3600) << 11) | ((uint32_t)((secs / 60) % 60) << 5) | ((uint32_t)(secs % 60) / 2);
}

// ============================================================================
// FatFs disk I/O glue (drive 0 = SDCard::active())
// ============================================================================
extern "C" {

DSTATUS disk_status(BYTE pdrv) {
    SDCard* sd = SDCard::active();
    if (pdrv != 0 || !sd) return STA_NOINIT;
    if (!sd->cardPresent()) return STA_NOINIT | STA_NODISK;
    return sd->cardInitialized() ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv) {
    SDCard* sd = SDCard::active();
    if (pdrv != 0 || !sd) return STA_NOINIT;
    if (!sd->cardPresent()) return STA_NOINIT | STA_NODISK;
    if (!sd->cardInitialized() && !sd->initCard()) return STA_NOINIT;
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
    SDCard* sd = SDCard::active();
    if (pdrv != 0 || !sd || !count) return RES_PARERR;
    if (!sd->cardInitialized()) return RES_NOTRDY;
    return sd->readSectors((uint32_t)sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
    SDCard* sd = SDCard::active();
    if (pdrv != 0 || !sd || !count) return RES_PARERR;
    if (!sd->cardInitialized()) return RES_NOTRDY;
    return sd->writeSectors((uint32_t)sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
    SDCard* sd = SDCard::active();
    if (pdrv != 0 || !sd) return RES_PARERR;
    if (!sd->cardInitialized()) return RES_NOTRDY;
    switch (cmd) {
        case CTRL_SYNC:
            return sd->waitReady() ? RES_OK : RES_ERROR;
        case GET_SECTOR_COUNT:
            *(LBA_t*)buff = sd->sectorCount();
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = SDCard::SECTOR_SIZE;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = sd->eraseBlockSectors();
            return RES_OK;
        default:
            return RES_PARERR;
    }
}

DWORD get_fattime(void) {
    return SDCard::fatTime();
}

} // extern "C"
