/* 
 */

#include "decoder.h"

// matches the arduino function
uint16_t crc16_update(uint16_t crc, uint8_t a)
{
	int i;

	crc ^= a;
	for (i = 0; i < 8; ++i)
	{
	    if (crc & 1)
		crc = (crc >> 1) ^ 0xA001;
	    else
		crc = (crc >> 1);
	}

	return crc;
}

static int scott_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t *b;
    int r,i;
    int candidates[BITBUF_ROWS + 1];

    bitbuffer_invert(bitbuffer);

    // If we have repeated rows, then check them first.
    // This might mean we check a row more than once, but that's not the end of the world.
    candidates[0] = bitbuffer_find_repeated_row(bitbuffer, 3, 49);
    candidates[1] = bitbuffer_find_repeated_row(bitbuffer, 2, 49);
    for (i=0; i< bitbuffer->num_rows; ++i) {
        candidates[i+2] = i;
    }

    for (i = 0; i < bitbuffer->num_rows+2; ++i) {
        int id, seq, temp, humid, barom, crc, computed_crc;
        uint8_t crcbuf[16];

        uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init);

        r = candidates[i];

        // find_repeated_row may have returned -1
        if (r<0) {
            continue;
        }

        if (bitbuffer->bits_per_row[r] != 49) {
            continue;
        }

        b = bitbuffer->bb[r];
        id = b[0] >> 3; // 5 bits
        seq =  ((b[0] & 0x07) << 1) | (b[1] >> 7); // 4 bits
        temp = ((b[1] & 0x7F) << 3) | (b[2] >> 5); // 10 bits
        humid = ((b[2] & 0x1F) << 5) | (b[3] >> 3); // 10 bits
        barom = ((b[3] & 0x07) << 9) | (b[4] << 1) | (b[5]>>7); // 12 bits
        crc = ((b[5] & 0x7F) << 1) | (b[6] >> 7); // 8 bits

        computed_crc = 0;
        computed_crc = crc16_update(computed_crc, id);
        computed_crc = crc16_update(computed_crc, seq);
        computed_crc = crc16_update(computed_crc, temp>>8);
        computed_crc = crc16_update(computed_crc, temp&0xFF);
        computed_crc = crc16_update(computed_crc, humid>>8);
        computed_crc = crc16_update(computed_crc, humid&0xFF);
        computed_crc = crc16_update(computed_crc, barom>>8);
        computed_crc = crc16_update(computed_crc, barom&0xFF);

        computed_crc = computed_crc & 0xFF;

        if (computed_crc == crc) {
            data = data_make(
                "id"   ,         "",            DATA_INT, id,
                "seq"   ,        "",            DATA_INT, seq,
                "temp"   ,       "",            DATA_INT, temp,
                "humid"   ,      "",            DATA_INT, humid,
                "barom"   ,      "",            DATA_INT, barom,
                "crc"   ,        "",            DATA_INT, crc,
                NULL);
            decoder_output_data(decoder, data);
            //printf("id: %d, seq: %d, temp: %d, humid: %d, barom: %d, crc: %d, computed_crc: %d\n", id, seq, temp, humid, barom, crc, computed_crc);
            return 1;
        } else {
            fprintf(stderr, "CRC MISMATCH. id: %d, seq: %d, temp: %d, humid: %d, barom: %d, crc: %d, computed_crc: %d\n", id, seq, temp, humid, barom, crc, computed_crc);
        }
    }

    return 0;
}


static char *output_fields[] = {
    "id",
    "seq",
    "temp",
    "humid",
    "barom",
    "crc",

    NULL
};

r_device scott = {
    .name           = "Scott Sensor",
    .modulation     = OOK_PULSE_PPM,
    .short_width    = 352, // Length of '0' in us
    .long_width     = 1056, // Length of '1' in us
    .reset_limit    = 12000, // Maximum gap size before End Of Message [us]
    .tolerance      = 100,
    // sync_width and gap_limit unset
    .decode_fn      = &scott_callback,
    .disabled       = 0,
    .fields         = output_fields
};
