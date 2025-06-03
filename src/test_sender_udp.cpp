
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cerrno>
#include <time.h>
#include <unistd.h>


static void     _serialize_float (unsigned char**, const float);
static uint32_t _pack754_32 (float);

int                CENT_sockfd;
struct sockaddr_in CENT_sockaddr_serv;


/* Buffer serialization functions */
static void _serialize_float(unsigned char** buffer, const float data){
  const uint32_t packed = _pack754_32(data);
  const uint32_t netend = htonl(packed);
  memcpy(*buffer, &netend, 4);
  *buffer += 4;
  return;
}

/* Beej's IEEE 754 floating-point arithmetic serialization functions */
static uint32_t _pack754_32(float f) {
  const unsigned bits = 32;
  const unsigned expbits = 8;
  float fnorm;
  int shift;
  uint32_t sign, exp, significand;
  unsigned significandbits = bits - expbits - 1; /* -1 for sign bit */

  if (fabs(f) < 0.00001f) return 0; /* Handle near-zero values */

  /* check sign and begin normalization */
  if (f < 0) { sign = 1; fnorm = -f; }
  else { sign = 0; fnorm = f; }

  /* get the normalized form of f and track the exponent */
  shift = 0;
  while (fnorm >= 2.0f) { fnorm /= 2.0f; shift++; }
  while (fnorm < 1.0f) { fnorm *= 2.0f; shift--; }
  fnorm -= 1.0f;

  /* calculate the binary form (non-float) of the significand data */
  significand = (uint32_t)(fnorm * ((1U << significandbits) + 0.5f));

  /* get the biased exponent */
  exp = shift + ((1 << (expbits - 1)) - 1); /* shift + bias */

  return (sign << (bits - 1)) | (exp << (bits - expbits - 1)) | significand;
}



int main(void)
{
    /* Comau (192.170.10.122) -> Centauro (192.170.10.120) */

    /* Socket init */
    memset((char*)&CENT_sockaddr_serv, 0, sizeof(struct sockaddr_in));
    CENT_sockaddr_serv.sin_family = AF_INET;  /* IPv4 */
    if ((CENT_sockaddr_serv.sin_addr.s_addr = inet_addr("127.0.0.1")) == INADDR_NONE) 
    {
      fprintf(stderr, "%s\n", "Invalid IP address.");
      return SO_ERROR;
    }
    CENT_sockaddr_serv.sin_port = htons(atoi("65000"));
    
    /* Creating socket file descriptor */
    if ( (CENT_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1 ) 
    {
      fprintf(stderr, "%s %s.\n", "Error in socket creation:", strerror(errno));
      return SO_ERROR;
    }
    
    /* Wait for the socket to finish the setup */
    sleep(2);
    
    while(1)
    {
        /* Data setup */
        float pos_x = 0;
        float pos_y = 0;
        float pos_z = 0;
        float pos_a = 0;
        float pos_e = 0;
        float pos_r = 0;
        
        /* Write original to file */
        FILE* file_orig = fopen("file_orig.txt", "a");
        if (file_orig == NULL) {
                printf("Error opening file\n");
        }
        
        fprintf(file_orig, "%.3f ",  pos_x);
        fprintf(file_orig, "%.3f ",  pos_y);
        fprintf(file_orig, "%.3f ",  pos_z);
        fprintf(file_orig, "%.3f ",  pos_a);
        fprintf(file_orig, "%.3f ",  pos_e);
        fprintf(file_orig, "%.3f\n", pos_r);
        
        fclose(file_orig);
        
        /* Serialization */
        const unsigned short buffer_size = 6*4; /* data*byte */
        unsigned char tx_args_ser[buffer_size];
        unsigned char* tx_args_ser_mvptr = &tx_args_ser[0];
        
        _serialize_float(&tx_args_ser_mvptr, pos_x);
        _serialize_float(&tx_args_ser_mvptr, pos_y);
        _serialize_float(&tx_args_ser_mvptr, pos_z);
        _serialize_float(&tx_args_ser_mvptr, pos_a);
        _serialize_float(&tx_args_ser_mvptr, pos_e);
        _serialize_float(&tx_args_ser_mvptr, pos_r);
        
        /* Send serialized to socket */
        if( sendto(CENT_sockfd, tx_args_ser, buffer_size, 0, (const struct sockaddr *) &CENT_sockaddr_serv, sizeof(struct sockaddr_in)) != buffer_size)
                printf("Error in writing to socket\n");
    }
    
    close(CENT_sockfd);
    return 0;
}