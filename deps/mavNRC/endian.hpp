#ifndef ENDIAN_HPP
#define ENDIAN_HPP

#ifdef _WIN32
#include <winsock2.h>
#else
#include <netinet/in.h>
#endif

#include <cstdint> // for uint32

void Struct_ntohl(void *data, int bytes)
{
    int i;

    uint32_t    *data_uint32;
    data_uint32 = (uint32_t *)data;

    for(i=0; i<bytes/sizeof(uint32_t); i++) data_uint32[i] = ntohl(data_uint32[i]);
}

void Swap_Double(double *dbl)
{

    uint32_t *lng;
    uint32_t tmp;

    lng = (uint32_t *)(dbl);
    tmp = lng[0];
    lng[0] = lng[1];
    lng[1] = tmp;
}

#endif // ENDIAN_HPP