#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <arpa/inet.h> // for ntohl/htonl

static void serialize_float(unsigned char **, const float);
static float unserialize_float(unsigned char **);
static uint32_t pack754_32(float);
static float unpack754_32(uint32_t);
void bufferization_sample(void);

/* Buffer serialization functions */
static void serialize_float(unsigned char **buffer, const float data)
{
    const uint32_t packed = pack754_32(data);
    const uint32_t netend = htonl(packed);
    memcpy(*buffer, &netend, 4);
    *buffer += 4;
}

static float unserialize_float(unsigned char **buffer)
{
    uint32_t netend;
    memcpy(&netend, *buffer, 4);
    *buffer += 4;
    const uint32_t hostend = ntohl(netend);
    return unpack754_32(hostend);
}

/* Beej's IEEE 754 floating-point arithmetic serialization functions */
static uint32_t pack754_32(float f)
{
    const unsigned bits = 32, expbits = 8;
    float fnorm;
    int shift;
    uint32_t sign, exp, significand;
    unsigned significandbits = bits - expbits - 1;

    if (fabs(f) < 0.00001f)
        return 0;
    if (f < 0)
    {
        sign = 1;
        fnorm = -f;
    }
    else
    {
        sign = 0;
        fnorm = f;
    }

    shift = 0;
    while (fnorm >= 2.0f)
    {
        fnorm /= 2.0f;
        shift++;
    }
    while (fnorm < 1.0f)
    {
        fnorm *= 2.0f;
        shift--;
    }
    fnorm -= 1.0f;

    significand = (uint32_t)(fnorm * ((1U << significandbits) + 0.5f));
    exp = shift + ((1 << (expbits - 1)) - 1);
    return (sign << (bits - 1)) | (exp << (bits - expbits - 1)) | significand;
}

static float unpack754_32(uint32_t i)
{
    const unsigned bits = 32, expbits = 8;
    unsigned bias = (1 << (expbits - 1)) - 1;
    unsigned significandbits = bits - expbits - 1;
    if (i == 0)
        return 0.0f;

    float result = (float)(i & ((1U << significandbits) - 1));
    result /= (1U << significandbits);
    result += 1.0f;

    int shift = ((i >> significandbits) & ((1U << expbits) - 1)) - bias;
    while (shift > 0)
    {
        result *= 2.0f;
        shift--;
    }
    while (shift < 0)
    {
        result /= 2.0f;
        shift++;
    }

    return result * (((i >> (bits - 1)) & 1) ? -1.0f : 1.0f);
}

void bufferization_sample()
{
    unsigned char tx_args_ser[6 * 4];
    unsigned char *p = tx_args_ser;
    float vals[6] = {800.0f, 0.0f, 270.0f, 0.0f, 180.0f, 0.0f};
    for (int i = 0; i < 6; ++i)
        serialize_float(&p, vals[i]);
}

void unbufferization_sample(const unsigned char rx_ret_ser[6 * 4], float out[6])
{
    unsigned char *mvptr = (unsigned char *)rx_ret_ser;
    for (int i = 0; i < 6; ++i)
    {
        out[i] = unserialize_float(&mvptr);
    }
}

void unbufferization_sample()
{
    unsigned char rx_ret_ser[6 * 4];
    unsigned char *rx_ret_ser_mvptr = &rx_ret_ser[0];

    // Presume rx_ret_ser has valid serialized data

    // Unserializing values
    float deser_x = unserialize_float(&rx_ret_ser_mvptr);
    float deser_y = unserialize_float(&rx_ret_ser_mvptr);
    float deser_z = unserialize_float(&rx_ret_ser_mvptr);
    float deser_a = unserialize_float(&rx_ret_ser_mvptr);
    float deser_e = unserialize_float(&rx_ret_ser_mvptr);
    float deser_r = unserialize_float(&rx_ret_ser_mvptr);

    return;
}

int main(void)
{
    const char *filename = "serialized.txt";
    FILE *fp = fopen(filename, "r");
    if (!fp)
    {
        perror("Failed to open file");
        return EXIT_FAILURE;
    }

    char line[256];
    unsigned char rx_ret_ser[6 * 4]; // 6 floats, each 4 bytes
    unsigned char *ptr;

    while (fgets(line, sizeof(line), fp))
    {
        // Skip empty lines
        if (line[0] == '\n' || line[0] == '\0')
            continue;

        // Parse hex bytes
        char *token = strtok(line, " \t\r\n");
        int idx = 0;
        while (token && idx < 24)
        {
            unsigned int byte;
            if (sscanf(token, "%x", &byte) != 1)
            {
                fprintf(stderr, "Invalid byte '%s' in input.\n", token);
                break;
            }
            rx_ret_ser[idx++] = (unsigned char)byte;
            token = strtok(NULL, " \t\r\n");
        }

        if (idx < 24)
        {
            fprintf(stderr, "Warning: expected 24 bytes, got %d. Skipping line.\n", idx);
            continue;
        }

        // Deserialize floats
        ptr = rx_ret_ser;
        float deserialized[6];
        for (int i = 0; i < 6; ++i)
        {
            deserialized[i] = unserialize_float(&ptr);
        }

        // Print the 6 floats
        printf("deser_x = %f, deser_y = %f, deser_z = %f, \
               deser_a = %f, deser_e = %f, deser_r = %f\n",
               deserialized[0], deserialized[1], deserialized[2],
               deserialized[3], deserialized[4], deserialized[5]);
    }

    fclose(fp);
    return EXIT_SUCCESS;
}