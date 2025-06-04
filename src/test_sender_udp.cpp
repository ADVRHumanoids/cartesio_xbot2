
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

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>       // for std::this_thread::sleep_for
#include <chrono>       // for std::chrono::milliseconds


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
    
    //while(1)
    //{
        /* Data setup */
        /*float pos_x = 700.0;
        float pos_y = 500.00;
        float pos_z = 1200.0;
        float pos_a = 0.0;
        float pos_e = 180.0;
        float pos_r = 0.0;
        */
        
        
        /* Serialization */
        //const unsigned short buffer_size = 6*4; /* data*byte */
        /*unsigned char tx_args_ser[buffer_size];
        unsigned char* tx_args_ser_mvptr = &tx_args_ser[0];
        
        _serialize_float(&tx_args_ser_mvptr, pos_x);
        _serialize_float(&tx_args_ser_mvptr, pos_y);
        _serialize_float(&tx_args_ser_mvptr, pos_z);
        _serialize_float(&tx_args_ser_mvptr, pos_a);
        _serialize_float(&tx_args_ser_mvptr, pos_e);
        _serialize_float(&tx_args_ser_mvptr, pos_r); */



  std::ifstream infile("deserialized.txt");
  if (!infile.is_open()) {
      std::cerr << "Failed to open deserialized.txt" << std::endl;
      return 1;
  }

  std::string line;
  while (std::getline(infile, line)) {
      std::istringstream iss(line);

      float pos_x, pos_y, pos_z, pos_a, pos_e, pos_r;
      if (!(iss >> pos_x >> pos_y >> pos_z >> pos_a >> pos_e >> pos_r)) {
          std::cerr << "Invalid line format, skipping: " << line << std::endl;
          continue; // skip malformed line
      }

      /* Serialization */
      const unsigned short buffer_size = 6 * 4; // 6 floats * 4 bytes each
      unsigned char tx_args_ser[buffer_size];
      unsigned char* tx_args_ser_mvptr = &tx_args_ser[0];

      _serialize_float(&tx_args_ser_mvptr, pos_x);
      _serialize_float(&tx_args_ser_mvptr, pos_y);
      _serialize_float(&tx_args_ser_mvptr, pos_z);
      _serialize_float(&tx_args_ser_mvptr, pos_a);
      _serialize_float(&tx_args_ser_mvptr, pos_e);
      _serialize_float(&tx_args_ser_mvptr, pos_r);

      /* Send serialized data to socket */
      if (sendto(CENT_sockfd, tx_args_ser, buffer_size, 0, (const struct sockaddr *) &CENT_sockaddr_serv, sizeof(struct sockaddr_in)) != buffer_size) {
          printf("Error in writing to socket\n");
      }

      // Sleep for 2 milliseconds
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      // Or use: usleep(2000); // 2000 microseconds = 2 ms
  }

  
        
        /* Send serialized to socket */
       /* if( sendto(CENT_sockfd, tx_args_ser, buffer_size, 0, (const struct sockaddr *) &CENT_sockaddr_serv, sizeof(struct sockaddr_in)) != buffer_size)
                printf("Error in writing to socket\n");
    }*/

    infile.close();
    close(CENT_sockfd);
    return 0;
}