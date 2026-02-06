#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdlib.h>

/// Identifier unique to each team
#define TEAM_ID 0

typedef struct UtcTime {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} UtcTime;

typedef struct DecimalDegrees {
    int16_t degrees;
    uint32_t millionths;
} DecimalDegrees;

typedef enum GpsFixType {
    NoFix = 0,
    Gps = 1,
    Dgps = 2
} GpsFixType;

/// Altitude relative to Mean Sea Level, in decimeters (tenths of meters)
typedef int32_t Altitude;

typedef uint8_t NumSats;

typedef struct GpsData {
    UtcTime time;
    DecimalDegrees latitude;
    DecimalDegrees longitude;
    GpsFixType fix_type;
    Altitude altitude;
    NumSats num_sats;
} GpsData;

uint8_t clamp_u8(uint8_t num, uint8_t min, uint8_t max) {
    if (num < min) {
        return min;
    } else if (num > max) {
        return max;
    } else {
        return num;
    }
}
int32_t clamp_i32(int32_t num, int32_t min, int32_t max) {
    if (num < min) {
        return min;
    } else if (num > max) {
        return max;
    } else {
        return num;
    }
}

void encode_binary(GpsData* data, uint8_t out[14]) {
    uint8_t hours  = clamp_u8(data->time.hours,    0, 23);
    uint8_t mins   = clamp_u8(data->time.minutes,  0, 59);
    uint8_t secs   = clamp_u8(data->time.seconds,  0, 59);
    uint8_t n_sats = clamp_u8(data->num_sats,      0, 15);
    int32_t alt    = clamp_i32(data->altitude, -0x80000, 0x7FFFF);
    
    // If -ve, make millionths -ve too
    int32_t lat_mult = (data->latitude.degrees < 0) ? -1 : 1;
    int32_t lat = ((int32_t)data->latitude.degrees) *10000000 + lat_mult*((int32_t)data->latitude.millionths) *10;
    
    int32_t lon_mult = (data->longitude.degrees < 0) ? -1 : 1;
    int32_t lon = ((int32_t)data->longitude.degrees)*10000000 + lon_mult*((int32_t)data->longitude.millionths)*10;
    
    bool fix_ok = data->fix_type != NoFix;
    bool is_dgps = data->fix_type == Dgps;
    
    // team_id | hours[5..2]
    out[0] = (TEAM_ID << 3) + (hours >> 2);
    
    // hours[1..0] | minutes
    out[1] = ((hours & 0b11) << 6) | mins;

    // seconds | fix_ok | is_dgps
    out[2] = (secs << 2) | (fix_ok << 1) | is_dgps;

    // num_sats | altitude[20..17]
    out[3] = (n_sats << 4) | (uint8_t)( (alt >> 16) & 0xF );
    
    // altitude[16..8]
    out[4] = (uint8_t)(alt >> 8);
    // altitude[7..0]
    out[5] = (uint8_t)(alt >> 0);
    
    // latitude[32..25]
    out[6] = (uint8_t)(lat >> 24);
    // latitude[24..17]
    out[7] = (uint8_t)(lat >> 16);
    // latitude[16..8]
    out[8] = (uint8_t)(lat >> 8);
    // latitude[7..0]
    out[9] = (uint8_t)(lat >> 0);
    
    // longitude[32..25]
    out[10] = (uint8_t)(lon >> 24);
    // longitude[24..17]
    out[11] = (uint8_t)(lon >> 16);
    // longitude[16..8]
    out[12] = (uint8_t)(lon >> 8);
    // longitude[7..0]
    out[13] = (uint8_t)(lon >> 0);
}

uint8_t decode_binary(uint8_t in[14], GpsData* out) {
    uint8_t id = in[0] >> 3;
    out->time.hours = ((in[0] & 0b111) << 2) | (in[1] >> 6);
    out->time.minutes = in[1] & 0b00111111;
    out->time.seconds = in[2] >> 2;
    
    uint8_t fix_ok = in[2] & 0b10;
    uint8_t dif_fix = in[2] & 0b1;
    if (fix_ok) {
        if (dif_fix) {
            out->fix_type = Dgps;
        } else {
            out->fix_type = Gps;
        }
    } else {
        out->fix_type = NoFix;
    }
    
    out->num_sats = in[3] >> 4;
    
    int32_t altitude_upper_bits = (in[3] & 0b1000) ? 0xFFF00000 : 0; // Sign-extend altitude if necessary
    out->altitude = altitude_upper_bits | (((int32_t)in[3]) & 0x0F) << 16 | (((int32_t)in[4]) << 8) | (((int32_t)in[5]) << 0);
    
    int32_t longitude_1e7 = (((int32_t)in[10]) << 24) | (((int32_t)in[11]) << 16) | (((int32_t)in[12]) << 8) | (((int32_t)in[13]) << 0);
    int32_t latitude_1e7  = (((int32_t)in[6]) << 24) | (((int32_t)in[7]) << 16) | (((int32_t)in[8]) << 8) | (((int32_t)in[9]) << 0);
    
    out->latitude.degrees = (latitude_1e7 / 10000000);
    int32_t lat_mult = (out->latitude.degrees < 0) ? -1 : 1; // Correct sign if necessary
    out->latitude.millionths = lat_mult*(latitude_1e7 - (10000000*out->latitude.degrees)) / 10;
    
    out->longitude.degrees = (longitude_1e7 / 10000000);
    int32_t lon_mult = (out->longitude.degrees < 0) ? -1 : 1;
    out->longitude.millionths = lon_mult*(longitude_1e7 - (10000000*out->longitude.degrees)) / 10;
    
    return id;
}

/** Testing stuff **/

void print_bits_in_byte(uint8_t byte) {
    for (int8_t i = 0; i < 8; i++) {
        bool is_set = byte & (1 << (7-i));
        if (is_set) {
            printf("1");
        }
        else {
            printf("0");
        }
    }
}
void print_bits_in_bytes(uint8_t byte[], uint8_t len) {
    for (int8_t i = 0; i < len; i++) {
        print_bits_in_byte(byte[i]);
        if (i != len-1) {
            printf("_");
        }
    }
    printf("\n");
}

void compare_vals(GpsData* data1, GpsData* data2) {
    if (data1->time.hours != data2->time.hours) {
        printf("Hours:\n");
        printf("%u\n", data1->time.hours);
        printf("%u\n", data2->time.hours);
        assert(data1->time.hours == data2->time.hours);
    }
    if (data1->time.minutes != data2->time.minutes) {
        printf("\n");
        printf("Minutes:\n");
        printf("%u\n", data1->time.minutes);
        printf("%u\n", data2->time.minutes);
        assert(data1->time.minutes == data2->time.minutes);
    }
    if (data1->time.seconds != data2->time.seconds) {
        printf("\n");
        printf("Seconds:\n");
        printf("%u\n", data1->time.seconds);
        printf("%u\n", data2->time.seconds);
        assert(data1->time.seconds == data2->time.seconds);
    }
    if (data1->latitude.degrees != data2->latitude.degrees) {
        printf("\n");
        printf("Lat (deg):\n");
        printf("%i\n", data1->latitude.degrees);
        printf("%i\n", data2->latitude.degrees);
        assert(data1->latitude.degrees == data2->latitude.degrees);
    }
    if (data1->latitude.millionths != data2->latitude.millionths) {
        printf("\n");
        printf("Lat (mils):\n");
        printf("%u\n", data1->latitude.millionths);
        printf("%u\n", data2->latitude.millionths);
        assert(data1->latitude.millionths == data2->latitude.millionths);
    }
    if (data1->longitude.degrees != data2->longitude.degrees) {
        printf("\n");
        printf("Long (deg):\n");
        printf("%i\n", data1->longitude.degrees);
        printf("%i\n", data2->longitude.degrees);
        assert(data1->longitude.degrees == data2->longitude.degrees);
    }
    if (data1->longitude.millionths != data2->longitude.millionths) {
        printf("\n");
        printf("Lat (mils):\n");
        printf("%u\n", data1->longitude.millionths);
        printf("%u\n", data2->longitude.millionths);
        assert(data1->longitude.millionths == data2->longitude.millionths);
    }
    if (data1->fix_type != data2->fix_type) {
        printf("\n");
        printf("Fix type:\n");
        printf("%u\n", data1->fix_type);
        printf("%u\n", data2->fix_type);
        assert(data1->fix_type == data2->fix_type);
    }
    if (data1->altitude != data2->altitude) {
        printf("\n");
        printf("Altitude:\n");
        printf("%i\n", data1->altitude);
        printf("%i\n", data2->altitude);
        assert(data1->altitude == data2->altitude);
    }
    if (data1->num_sats != data2->num_sats) {
        printf("\n");
        printf("Num satellites:\n");
        printf("%u\n", data1->num_sats);
        printf("%u\n", data2->num_sats);
        assert(data1->num_sats == data2->num_sats);
    }
}

uint8_t rand_u8_in_range(uint8_t min, uint8_t max) {
    return (rand() % (max - min + 1)) + min;
}
uint8_t rand_i16_in_range(int16_t min, int16_t max) {
    return (rand() % (max - min + 1)) + min;
}
int32_t rand_i32_in_range(int32_t min, int32_t max) {
    return (rand() % (max - min + 1)) + min;
}

void random_test() {
    UtcTime time = {.hours = rand_u8_in_range(0,23), .minutes = rand_u8_in_range(0,59), .seconds = rand_u8_in_range(0,59)};
    DecimalDegrees lat = {.degrees = rand_i16_in_range(0, 360)-180, .millionths = rand_i32_in_range(0, 999999)};
    DecimalDegrees lon = {.degrees = rand_i16_in_range(0, 360)-180, .millionths = rand_i32_in_range(0, 999999)};
    GpsFixType fix_type = rand_u8_in_range(0,2);
    NumSats n = rand_u8_in_range(0,15);
    Altitude alt = rand_i32_in_range(0, 0xFFFFF)-0x80000; // -0x80000 to 0x7FFFF
    
    GpsData in = {.time = time, .latitude = lat, .longitude = lon, .fix_type = fix_type, .num_sats = n, .altitude = alt};
    GpsData out;
    uint8_t binary[14];
    
    encode_binary(&in, binary);
    decode_binary(binary, &out);
    compare_vals(&in, &out);
}

void random_tests() {
    #define NUM_TESTS 100000000
    for (int i = 1; i <= NUM_TESTS; i++) {
        random_test();
        if (i % (NUM_TESTS/50) == 0) {
            printf("%u tests complete\n", i);
        }
    }
}

int main() {
    random_tests();
    return 0;
}
