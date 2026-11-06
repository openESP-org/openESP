#include "PTC.h"
#include <stdint.h>
#include <string.h>

/* Fixed-size photo-only entry point. The caller owns the output allocation. */
int openesp_ptc_photo(const uint8_t* source, int length, uint8_t* output, int output_size)
{
    if (!source || !output || length < 48 || output_size != 256 * 256 * 2)
        return -1;
    uint32_t width, height, offset;
    memcpy(&width, source + 12, 4);
    memcpy(&height, source + 16, 4);
    memcpy(&offset, source + 32, 4);
    if (source[0] != 1 || memcmp(source + 4, "PTC\0", 4) != 0 ||
        source[9] != PT1555 || width != 256 || height != 256 || offset != 44 || source[40] != 0)
        return -1;
    source += 48;
    length -= 48;
    if (length < (int)(sizeof(PTCFileHeader) + 4 + sizeof(PTCFrameHeader)))
        return -1;
    PTCFileHeader file;
    PTCFrameHeader frame;
    memcpy(&file, source, sizeof(file));
    if (file.NumFrames != 1 || file.OffsetToFrame < 0 || file.OffsetToFrame > length - (int)sizeof(file) - 4 - (int)sizeof(frame))
        return -1;
    memcpy(&frame, source + sizeof(file) + file.OffsetToFrame + 4, sizeof(frame));
    if (frame.Width != 256 || frame.Height != 256 || frame.ChunkWidth != 256 ||
        frame.TileCount != 16 || frame.TileLength <= 0 || frame.TileLength > length ||
        frame.CompressedLength <= 0 || frame.CompressedLength > length ||
        frame.NumChannels < 1 || frame.NumChannels > 4 || frame.BitDepth < 1 || frame.BitDepth > 16 ||
        frame.OffsetToData < 0 || frame.OffsetToData > length)
        return -1;
    if (file.OffsetToFrame != 0 || frame.CompressedLength != length - (int)sizeof(file) - 4)
        return -1;
    int64_t table = (int64_t)sizeof(file) + 4 + sizeof(frame) + frame.OffsetToData;
    if (table < 0 || table + 16 * 4 > length)
        return -1;
    int64_t end = table + 16 * 4;
    for (int i = 0; i < 16; ++i) {
        int32_t tile_size;
        memcpy(&tile_size, source + table + 4 * i, 4);
        if (tile_size <= 0 || tile_size > frame.TileLength)
            return -1;
        end += tile_size;
        if (end > length)
            return -1;
    }
    WriteRowParams row = {0};
    DecodeParams params = {0};
    row.Dest = output;
    row.Type = PT1555;
    row.RowWidth = 256;
    row.StrideBytes = 512;
    row.PixelParams.NumBitsColor = 5;
    row.PixelParams.NumBitsAlpha = 1;
    params.LP2 = 4;
    params.LP4 = 7;
    params.MipGenerate[0] = 1;
    params.RowParams[0] = &row;
    return PTCDecompress(&params, source, length) == length ? output_size : -1;
}
