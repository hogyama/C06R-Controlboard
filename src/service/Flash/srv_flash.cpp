#include "srv_flash.h"

bool isAllFF(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (data[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

SrvFlash::SrvFlash(int32_t origin_lat_1e7, int32_t origin_lon_1e7)
    : origin_lat_1e7(origin_lat_1e7),
      origin_lon_1e7(origin_lon_1e7),
      current_session_id(0),
      active_map_region(0),
      current_update_count(NO_MAP_UPDATE_COUNT),
      active_file_idx(-1),
      current_log_record_count(0),
      used_file_bit_flag(0)
{
}

SrvFlash::~SrvFlash()
{
    flash.end();
}


bool SrvFlash::init(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, uint32_t clock_hz, uint8_t mode)
{
    HalS25FL128L::Config config{};
    config.sck = sck;
    config.miso = miso;
    config.mosi = mosi;
    config.cs = cs;
    config.clock_hz = clock_hz;
    config.mode = mode;
    if (!flash.begin(config)) {
        return false;
    }
    scanSessionId();
    return true;
}

bool SrvFlash::startNewSequence(uint8_t* map_scratch_buffer)
{
    if (map_scratch_buffer == nullptr) {
        return false;
    }

    const uint32_t previous_session_id = current_session_id;
    const uint8_t previous_map_region = active_map_region;
    const uint32_t previous_update_count = current_update_count;
    const int8_t previous_file_idx = active_file_idx;
    const uint32_t previous_log_record_count = current_log_record_count;
    const uint8_t previous_used_file_flags = used_file_bit_flag;

    uint8_t next_file_idx = MAX_FILES;
    for (uint8_t i = 0; i < MAX_FILES; i++) {
        if ((used_file_bit_flag & (1U << i)) == 0) {
            next_file_idx = i;
            break;
        }
    }
    if (next_file_idx >= MAX_FILES) {
        active_file_idx = -1;
        return false;
    }

    const bool has_previous_map =
        loadLatestMap(map_scratch_buffer) == MapLoadResult::Valid;

    const uint8_t next_map_region = (active_map_region == 0) ? 1 : 0;
    if (!eraseMap(next_map_region)) {
        return false;
    }
    // scanFiles()またはeraseFILE()で消去済みと確認した未使用領域を選ぶ。
    // 5MBの再消去をここで行うとシーケンス冒頭のログを失うため省略する。

    current_session_id++;
    active_map_region = next_map_region;
    current_update_count = NO_MAP_UPDATE_COUNT;
    active_file_idx = next_file_idx;
    current_log_record_count = 0;
    
    // もし前のMAPが存在していれば、それを新しいMAPにコピーする
    if (has_previous_map) {
        if (!saveMap(map_scratch_buffer)) {
            // 継承MAPの保存に失敗した場合は、旧セッションを有効状態へ戻す。
            current_session_id = previous_session_id;
            active_map_region = previous_map_region;
            current_update_count = previous_update_count;
            active_file_idx = previous_file_idx;
            current_log_record_count = previous_log_record_count;
            used_file_bit_flag = previous_used_file_flags;
            return false;
        }
    }
    used_file_bit_flag |= (1U << next_file_idx);
    return true;
}

SrvFlash::MapLoadResult SrvFlash::loadLatestMap(uint8_t* map_buffer_out)
{
    if (map_buffer_out == nullptr) return MapLoadResult::InvalidArgument;
    if (current_update_count == NO_MAP_UPDATE_COUNT) return MapLoadResult::NotFound;

    uint8_t record[MAP_RECORD_SIZE];
    const uint32_t addr = calculateMapAddress(active_map_region, current_update_count);
    if (!flash.read(addr, record, sizeof(record))) {
        return MapLoadResult::ReadError;
    }

    MapHeader* header = reinterpret_cast<MapHeader*>(record);
    if (header->data.magic != MAP_VALID_MAGIC) {
        return MapLoadResult::Corrupt;
    }
    if (header->data.crc32 != calculateMapCRC32(record)) {
        return MapLoadResult::Corrupt;
    }
    if (header->data.origin_lat_1e7 != origin_lat_1e7 ||
        header->data.origin_lon_1e7 != origin_lon_1e7) {
        return MapLoadResult::OriginMismatch;
    }
    if (header->data.format_version != Flash::MAP_FORMAT_VERSION ||
        header->data.bits_per_cell != Flash::MAP_BITS_PER_CELL) {
        return MapLoadResult::Corrupt;
    }
    memcpy(map_buffer_out, record + MAP_HEADER_SIZE, MAP_SIZE);
    return MapLoadResult::Valid;
}

bool SrvFlash::saveMap(const uint8_t* map_data)
{
    if (map_data == nullptr) {
        return false;
    }
    const uint32_t next_update_count =
        (current_update_count == NO_MAP_UPDATE_COUNT) ? 0 : (current_update_count + 1);
    if (next_update_count >= MAX_MAPS) {
        return false;
    }
    uint8_t record[MAP_RECORD_SIZE];
    memset(record, 0xFF, sizeof(record));

    MapHeader* header = reinterpret_cast<MapHeader*>(record);
    header->data.magic = MAP_VALID_MAGIC;
    header->data.crc32 = 0xFFFFFFFFUL;
    header->data.session_id = current_session_id;
    header->data.update_count = next_update_count;
    header->data.origin_lat_1e7 = origin_lat_1e7;
    header->data.origin_lon_1e7 = origin_lon_1e7;
    header->data.format_version = Flash::MAP_FORMAT_VERSION;
    header->data.bits_per_cell = Flash::MAP_BITS_PER_CELL;
    memcpy(record + MAP_HEADER_SIZE, map_data, MAP_SIZE);
    header->data.crc32 = calculateMapCRC32(record);

    const uint32_t addr = calculateMapAddress(active_map_region, next_update_count);
    if ((next_update_count % MAPS_PER_SECTOR) == 0) {
        if (!flash.erase4K(addr)) {
            return false;
        }
    }

    if (!flash.program(addr + 8, record + 8, sizeof(record) - 8, true)) {
        return false;
    }
    if (!flash.program(addr + offsetof(MapHeader, data.crc32),
                       &header->data.crc32,
                       sizeof(header->data.crc32),
                       true)) {
        return false;
    }
    if (!flash.program(addr, &header->data.magic, sizeof(header->data.magic), true)) {
        return false;
    }

    current_update_count = next_update_count;
    return true;
}

bool SrvFlash::eraseMap(uint8_t map_region)
{
    if (map_region > 1) {
        return false;
    }

    const uint32_t start = (map_region == 0) ? MAP0_START_ADDRESS : MAP1_START_ADDRESS;
    for (uint32_t offset = 0; offset < MAP_REGION_SIZE; offset += SECTOR_SIZE) {
        if (!flash.erase4K(start + offset)) {
            return false;
        }
    }
    if(map_region == active_map_region) {
        current_update_count = NO_MAP_UPDATE_COUNT;
    }
    return true;
}

bool SrvFlash::resetAllMaps()
{
    bool res0 = eraseMap(0);
    bool res1 = eraseMap(1);

    active_map_region = 0;
    current_update_count = NO_MAP_UPDATE_COUNT;

    return (res0 && res1);
}

bool SrvFlash::saveLog(const Flash::LogFrame& packet)
{
    if (active_file_idx < 0 || current_log_record_count >= MAX_LOGS_PER_FILE) {
        return false;
    }

    LogRecord record;
    memset(record.raw, 0xFF, sizeof(record.raw));
    record.data.header.magic = LOG_VALID_MAGIC;
    record.data.header.crc32 = 0xFFFFFFFFUL;
    record.data.header.session_id = current_session_id;
    record.data.header.record_count = current_log_record_count;
    record.data.packet = packet;
    record.data.header.crc32 = calculateLogCRC32(record.raw);

    const uint32_t addr = calculateLogAddress(active_file_idx, current_log_record_count);

    // startNewSequence()で5MB領域全体を消去済みなので、16件ごとの4KB再消去は不要。
    // magic以外を一度に書き、最後にmagicを書いて途中電断レコードを有効扱いしない。
    constexpr size_t commit_offset = offsetof(LogHeader, crc32);
    if (!flash.program(addr + commit_offset,
                       record.raw + commit_offset,
                       LOG_RECORD_SIZE - commit_offset,
                       true)) {
        return false;
    }

    if (!flash.program(addr, &record.data.header.magic, sizeof(record.data.header.magic), true)) {
        return false;
    }

    current_log_record_count++;
    used_file_bit_flag |= (1U << active_file_idx);
    if (current_log_record_count >= MAX_LOGS_PER_FILE) {
        active_file_idx = -1;
    }
    return true;
}

SrvFlash::LogReadResult SrvFlash::loadLog(uint8_t file_idx, uint32_t log_index, Flash::LogFrame& packet_out)
{
    if (file_idx >= MAX_FILES || log_index >= MAX_LOGS_PER_FILE) {
        return LogReadResult::InvalidArgument;
    }

    LogRecord record;
    if (!flash.read(calculateLogAddress(file_idx, log_index), record.raw, LOG_RECORD_SIZE)) {
        return LogReadResult::ReadError;
    }
    if (isAllFF(record.raw, LOG_RECORD_SIZE)) {
        return LogReadResult::EndOfFile;
    }
    if (record.data.header.magic != LOG_VALID_MAGIC) {
        return LogReadResult::Corrupt;
    }
    if (record.data.header.crc32 != calculateLogCRC32(record.raw)) {
        return LogReadResult::Corrupt;
    }
    if (record.data.header.record_count != log_index) {
        return LogReadResult::Corrupt;
    }

    packet_out = record.data.packet;
    return LogReadResult::Valid;
}

bool SrvFlash::eraseFILE(uint8_t file_idx)
{
    if (file_idx >= MAX_FILES) {
        return false;
    }

    const uint32_t start = FILES_START_ADDRESS + static_cast<uint32_t>(file_idx) * FILE_SIZE;
    // 5MB領域は64KBブロック単位で消去し、4KBを1280回消去する待ち時間を避ける。
    constexpr uint32_t erase_block_size = 0x10000UL;
    for (uint32_t offset = 0; offset < FILE_SIZE; offset += erase_block_size) {
        if (!flash.erase64K(start + offset)) {
            return false;
        }
    }

    used_file_bit_flag &= ~(1U << file_idx);
    if (active_file_idx == static_cast<int8_t>(file_idx)) {
        active_file_idx = -1;
        current_log_record_count = 0;
    }
    return true;
}

bool SrvFlash::resetAllFiles()
{
    bool all_erased = true;

    // 一括消去命令にはせず、既存の1ファイル消去を先頭から繰り返す。
    // 途中で失敗しても残りのファイルは消去を試みる。
    for (uint8_t file_idx = 0; file_idx < MAX_FILES; ++file_idx) {
        if (!eraseFILE(file_idx)) {
            all_erased = false;
        }
    }

    // 消去結果から使用中ファイルと使用済みフラグを作り直す。
    scanFiles();
    return all_erased;
}

uint32_t SrvFlash::getSessionId() const
{
    return current_session_id;
}

int8_t SrvFlash::getActiveFileIndex() const
{
    return active_file_idx;
}

uint8_t SrvFlash::getUsedFileBitFlag() const
{
    constexpr uint8_t FILE_FLAG_MASK =
        static_cast<uint8_t>((1U << MAX_FILES) - 1U);
    return used_file_bit_flag & FILE_FLAG_MASK;
}

bool SrvFlash::isStorageFull() const
{
    return active_file_idx < 0;
}

void SrvFlash::scanSessionId()
{
    current_session_id = 0;
    scanMap();
    scanFiles();
}

void SrvFlash::scanMap()
{
    active_map_region = 0;
    current_update_count = NO_MAP_UPDATE_COUNT;

    uint32_t best_session_id = 0;
    uint32_t best_update_count = NO_MAP_UPDATE_COUNT;
    uint8_t best_region = 0;

    for (uint8_t region = 0; region < 2; region++) {
        for (uint32_t record_index = 0; record_index < MAX_MAPS; record_index++) {
            uint8_t record[MAP_RECORD_SIZE];
            const uint32_t addr = calculateMapAddress(region, record_index);
            if (!flash.read(addr, record, sizeof(record))) {
                continue;
            }

            MapHeader* header = reinterpret_cast<MapHeader*>(record);
            if (header->data.magic != MAP_VALID_MAGIC) {
                continue;
            }
            if (header->data.crc32 != calculateMapCRC32(record)) {
                continue;
            }
            if (header->data.update_count != record_index) {
                continue;
            }
            if (header->data.format_version != Flash::MAP_FORMAT_VERSION ||
                header->data.bits_per_cell != Flash::MAP_BITS_PER_CELL) {
                continue;
            }

            const bool newer_session = header->data.session_id > best_session_id;
            const bool same_session_newer_update =
                header->data.session_id == best_session_id &&
                (best_update_count == NO_MAP_UPDATE_COUNT ||
                 header->data.update_count > best_update_count);
            const bool same_position_region1 =
                header->data.session_id == best_session_id &&
                header->data.update_count == best_update_count &&
                region == 1;

            if (newer_session || same_session_newer_update || same_position_region1) {
                best_session_id = header->data.session_id;
                best_update_count = header->data.update_count;
                best_region = region;
            }
        }
    }

    if (best_update_count != NO_MAP_UPDATE_COUNT) {
        current_session_id = best_session_id;
        current_update_count = best_update_count;
        active_map_region = best_region;
    }
}

void SrvFlash::scanFiles()
{
    used_file_bit_flag = 0;
    active_file_idx = -1;
    current_log_record_count = 0;

    for (uint8_t file_idx = 0; file_idx < MAX_FILES; file_idx++) {
        LogRecord record{};
        const uint32_t file_start =
            FILES_START_ADDRESS + static_cast<uint32_t>(file_idx) * FILE_SIZE;

        // 各64KBブロックは先頭レコードから順番に使うため、先頭128Bを調べれば
        // 途中書込み・旧1MB形式・中断された消去も保守的に使用済み判定できる。
        bool region_has_data = false;
        bool first_record_read = false;
        for (uint32_t offset = 0;
             offset < FILE_SIZE;
             offset += HalS25FL128L::BLOCK_SIZE_64K) {
            LogRecord probe{};
            if (!flash.read(file_start + offset, probe.raw, LOG_RECORD_SIZE)) {
                region_has_data = true; // 読出し不能領域は上書きしない。
                break;
            }
            if (offset == 0) {
                record = probe;
                first_record_read = true;
            }
            if (!isAllFF(probe.raw, LOG_RECORD_SIZE)) {
                region_has_data = true;
                break;
            }
        }
        if (!region_has_data) {
            continue;
        }

        used_file_bit_flag |= (1U << file_idx);
        if (!first_record_read || isAllFF(record.raw, LOG_RECORD_SIZE)) {
            continue;
        }
        if (record.data.header.magic != LOG_VALID_MAGIC) {
            continue;
        }
        if (record.data.header.crc32 != calculateLogCRC32(record.raw)) {
            continue;
        }
        if (record.data.header.record_count != 0) {
            continue;
        }

        if (record.data.header.session_id > current_session_id) {
            current_session_id = record.data.header.session_id;
        }
    }

    for (uint8_t i = 0; i < MAX_FILES; i++) {
        if ((used_file_bit_flag & (1U << i)) == 0) {
            active_file_idx = i;
            current_log_record_count = 0;
            break;
        }
    }
}

uint32_t SrvFlash::calculateMapAddress(uint8_t region, uint32_t update_cnt)
{
    const uint32_t base = (region == 0) ? MAP0_START_ADDRESS : MAP1_START_ADDRESS;
    const uint32_t sector = update_cnt / MAPS_PER_SECTOR;
    const uint32_t pos = update_cnt % MAPS_PER_SECTOR;
    return base + sector * SECTOR_SIZE + pos * MAP_RECORD_SIZE;
}

uint32_t SrvFlash::calculateLogAddress(uint8_t file_idx, uint32_t record_cnt)
{
    return FILES_START_ADDRESS +
           static_cast<uint32_t>(file_idx) * FILE_SIZE +
           record_cnt * LOG_RECORD_SIZE;
}

uint32_t SrvFlash::calculateMapCRC32(const uint8_t* data)
{
    if (data == nullptr) {
        return 0;
    }
    return esp_rom_crc32_le(0, data + 8, MAP_RECORD_SIZE - 8);
}

uint32_t SrvFlash::calculateLogCRC32(const uint8_t* data)
{
    if (data == nullptr) {
        return 0;
    }
    return esp_rom_crc32_le(0, data + 8, LOG_RECORD_SIZE - 8);
}
