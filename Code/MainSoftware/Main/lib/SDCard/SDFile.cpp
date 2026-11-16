// SDFile, CSVWriter and CSVReader (see SDCard.h)
#include "SDCard.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
// SDFile
// ============================================================================
bool SDFile::open(const char* path, Mode mode) {
    close();
    BYTE flags;
    switch (mode) {
        case Mode::Read:      flags = FA_READ | FA_OPEN_EXISTING; break;
        case Mode::Write:     flags = FA_READ | FA_WRITE | FA_CREATE_ALWAYS; break;
        case Mode::Append:    flags = FA_READ | FA_WRITE | FA_OPEN_APPEND; break;
        case Mode::ReadWrite: flags = FA_READ | FA_WRITE | FA_OPEN_EXISTING; break;
        case Mode::CreateNew: flags = FA_READ | FA_WRITE | FA_CREATE_NEW; break;
        default:              flags = FA_READ; break;
    }
    open_ = ok(f_open(&fil_, path, flags));
    return open_;
}

bool SDFile::close() {
    if (!open_) return true;
    open_ = false;
    return ok(f_close(&fil_));
}

size_t SDFile::write(const void* data, size_t len) {
    if (!open_ || !len) return 0;
    UINT bw = 0;
    ok(f_write(&fil_, data, (UINT)len, &bw));
    if (lastResult_ == FR_OK && bw < len) lastResult_ = FR_DENIED;  // card full
    return bw;
}

size_t SDFile::write(const char* text) {
    return text ? write(text, strlen(text)) : 0;
}

size_t SDFile::println(const char* text) {
    size_t n = write(text);
    return n + write("\r\n", 2);
}

size_t SDFile::vprintf(const char* fmt, va_list ap) {
    char small[128];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(small, sizeof small, fmt, ap);
    size_t out = 0;
    if (n < 0) {
        out = 0;
    } else if ((size_t)n < sizeof small) {
        out = write(small, (size_t)n);
    } else {
        char* big = (char*)malloc((size_t)n + 1);
        if (big) {
            vsnprintf(big, (size_t)n + 1, fmt, ap2);
            out = write(big, (size_t)n);
            free(big);
        }
    }
    va_end(ap2);
    return out;
}

size_t SDFile::printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    size_t n = vprintf(fmt, ap);
    va_end(ap);
    return n;
}

int32_t SDFile::read(void* buf, size_t len) {
    if (!open_) return -1;
    UINT br = 0;
    if (!ok(f_read(&fil_, buf, (UINT)len, &br))) return -1;
    return (int32_t)br;
}

int SDFile::read() {
    uint8_t c;
    return read(&c, 1) == 1 ? c : -1;
}

int32_t SDFile::readLine(char* buf, size_t len) {
    if (!buf || len == 0) return -1;
    size_t n = 0;
    bool any = false;
    int c;
    while ((c = read()) >= 0) {
        any = true;
        if (c == '\n') break;
        if (c == '\r') continue;
        if (n < len - 1) buf[n++] = (char)c;
    }
    buf[n] = 0;
    return any ? (int32_t)n : -1;
}

bool SDFile::seek(uint64_t pos) {
    return open_ && ok(f_lseek(&fil_, (FSIZE_t)pos));
}

uint64_t SDFile::position() const {
    return open_ ? f_tell(&fil_) : 0;
}

uint64_t SDFile::size() const {
    return open_ ? f_size(&fil_) : 0;
}

bool SDFile::eof() const {
    return !open_ || f_eof(&fil_);
}

bool SDFile::sync() {
    return open_ && ok(f_sync(&fil_));
}

bool SDFile::truncate() {
    return open_ && ok(f_truncate(&fil_));
}

// ============================================================================
// CSVWriter
// ============================================================================
bool CSVWriter::open(const char* path, bool append) {
    close();
    len_ = rowLen_ = 0;
    rowOverflow_ = false;
    firstField_ = true;
    rows_ = rowsSinceSync_ = errors_ = 0;
    bytes_ = 0;
    lastSync_ = get_absolute_time();
    if (!file_.open(path, append ? SDFile::Mode::Append : SDFile::Mode::Write)) return false;
    hasData_ = append && file_.size() > 0;
    return true;
}

bool CSVWriter::close() {
    if (!file_.isOpen()) return true;
    bool ok = sync();
    ok &= file_.close();
    return ok;
}

bool CSVWriter::writeHeader(std::initializer_list<const char*> columns) {
    return writeHeader(columns.begin(), columns.size());
}

bool CSVWriter::writeHeader(const char* const* columns, size_t count) {
    if (hasData_) return true;  // appending to a file that already has its header
    for (size_t i = 0; i < count; i++) add(columns[i]);
    bool ok = commitRow();
    hasData_ = true;
    return ok;
}

bool CSVWriter::writeHeader(const char* line) {
    if (hasData_) return true;
    bool ok = put(line, strlen(line)) && put("\n", 1);
    if (!ok) errors_++;
    hasData_ = true;
    return ok;
}

void CSVWriter::appendRow(const char* s, size_t n) {
    if (rowLen_ + n >= sizeof row_ - 1) {  // keep 1 byte for the newline
        rowOverflow_ = true;
        return;
    }
    memcpy(row_ + rowLen_, s, n);
    rowLen_ += n;
}

void CSVWriter::beginField() {
    if (!firstField_) appendRow(&delim_, 1);
    firstField_ = false;
}

CSVWriter& CSVWriter::add(const char* text) {
    beginField();
    if (!text) return *this;
    bool quote = false;
    for (const char* p = text; *p; p++) {
        if (*p == delim_ || *p == '"' || *p == '\n' || *p == '\r') {
            quote = true;
            break;
        }
    }
    if (!quote) {
        appendRow(text, strlen(text));
        return *this;
    }
    appendRow("\"", 1);
    for (const char* p = text; *p; p++) {
        if (*p == '"') appendRow("\"\"", 2);
        else appendRow(p, 1);
    }
    appendRow("\"", 1);
    return *this;
}

CSVWriter& CSVWriter::add(char c) {
    char s[2] = {c, 0};
    return add(s);
}

CSVWriter& CSVWriter::add(bool b) {
    beginField();
    appendRow(b ? "1" : "0", 1);
    return *this;
}

CSVWriter& CSVWriter::add(int32_t v) {
    return addf("%ld", (long)v);
}

CSVWriter& CSVWriter::add(uint32_t v) {
    return addf("%lu", (unsigned long)v);
}

CSVWriter& CSVWriter::add(int64_t v) {
    return addf("%lld", (long long)v);
}

CSVWriter& CSVWriter::add(uint64_t v) {
    return addf("%llu", (unsigned long long)v);
}

CSVWriter& CSVWriter::add(float v, int decimals) {
    return add((double)v, decimals);
}

CSVWriter& CSVWriter::add(double v, int decimals) {
    if (isnan(v)) return skip();
    return addf("%.*f", decimals < 0 ? decimals_ : decimals, v);
}

CSVWriter& CSVWriter::addf(const char* fmt, ...) {
    beginField();
    if (rowOverflow_) return *this;
    size_t room = sizeof row_ - 1 - rowLen_;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(row_ + rowLen_, room, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= room) rowOverflow_ = true;
    else rowLen_ += (size_t)n;
    return *this;
}

CSVWriter& CSVWriter::skip() {
    beginField();
    return *this;
}

bool CSVWriter::commitRow() {
    bool ok = !rowOverflow_;
    if (ok) {
        row_[rowLen_++] = '\n';
        ok = put(row_, rowLen_);
    }
    if (!ok) errors_++;
    rowLen_ = 0;
    rowOverflow_ = false;
    firstField_ = true;
    return ok;
}

void CSVWriter::afterRow() {
    rows_++;
    rowsSinceSync_++;
    bool due = (syncRows_ && rowsSinceSync_ >= syncRows_) ||
               (syncMs_ && absolute_time_diff_us(lastSync_, get_absolute_time()) >= (int64_t)syncMs_ * 1000);
    if (due) sync();
}

bool CSVWriter::endRow() {
    bool ok = commitRow();
    afterRow();
    return ok;
}

bool CSVWriter::writeRow(const float* values, size_t count, int decimals) {
    for (size_t i = 0; i < count; i++) add(values[i], decimals);
    return endRow();
}

bool CSVWriter::writeLine(const char* line) {
    bool ok = put(line, strlen(line)) && put("\n", 1);
    if (!ok) errors_++;
    afterRow();
    return ok;
}

bool CSVWriter::put(const char* s, size_t n) {
    if (!file_.isOpen()) return false;
    // Fill the buffer to exactly full before writing, so writes stay large and sector-aligned
    while (n) {
        size_t room = sizeof buf_ - len_;
        size_t chunk = n < room ? n : room;
        memcpy(buf_ + len_, s, chunk);
        len_ += chunk;
        bytes_ += chunk;
        s += chunk;
        n -= chunk;
        if (len_ == sizeof buf_ && !flush()) {
            if (len_ == sizeof buf_) {  // nothing could be written: drop the rest of this data
                bytes_ -= n;
                return false;
            }
        }
    }
    return true;
}

bool CSVWriter::flush() {
    if (!len_) return true;
    if (!file_.isOpen()) return false;
    size_t written = file_.write(buf_, len_);
    if (written < len_) {
        // Keep what didn't make it, to retry next time
        memmove(buf_, buf_ + written, len_ - written);
        len_ -= written;
        errors_++;
        return false;
    }
    len_ = 0;
    return true;
}

bool CSVWriter::sync() {
    bool ok = flush();
    ok &= file_.sync();
    lastSync_ = get_absolute_time();
    rowsSinceSync_ = 0;
    return ok;
}

// ============================================================================
// CSVReader
// ============================================================================
bool CSVReader::open(const char* path) {
    count_ = 0;
    line_ = 0;
    return file_.open(path, SDFile::Mode::Read);
}

bool CSVReader::readRow() {
    count_ = 0;
    int32_t n;
    do {
        n = file_.readLine(buf_, sizeof buf_);
        if (n < 0) return false;
        line_++;
    } while (n == 0);

    char* p = buf_;
    while (count_ < MAX_FIELDS) {
        char* start = p;
        char* out = p;
        if (*p == '"') {
            p++;
            while (*p) {
                if (*p == '"') {
                    if (p[1] == '"') {  // escaped quote
                        *out++ = '"';
                        p += 2;
                        continue;
                    }
                    p++;  // closing quote
                    break;
                }
                *out++ = *p++;
            }
            while (*p && *p != delim_) p++;  // ignore anything between the closing quote and delimiter
        } else {
            while (*p && *p != delim_) *out++ = *p++;
        }
        char end = *p;
        *out = 0;
        fields_[count_++] = start;
        if (!end) break;
        p++;
    }
    return true;
}

int CSVReader::indexOf(const char* name) const {
    for (size_t i = 0; i < count_; i++) {
        if (strcmp(fields_[i], name) == 0) return (int)i;
    }
    return -1;
}

float CSVReader::getFloat(size_t i, float fallback) const {
    const char* s = field(i);
    char* end;
    float v = strtof(s, &end);
    return end == s ? fallback : v;
}

double CSVReader::getDouble(size_t i, double fallback) const {
    const char* s = field(i);
    char* end;
    double v = strtod(s, &end);
    return end == s ? fallback : v;
}

long CSVReader::getInt(size_t i, long fallback) const {
    const char* s = field(i);
    char* end;
    long v = strtol(s, &end, 10);
    return end == s ? fallback : v;
}
