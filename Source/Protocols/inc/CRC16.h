#ifndef __CRC_16_H
#define __CRC_16_H

#ifdef __cplusplus
extern "C" {
#endif

unsigned short CRC16( 
                     const unsigned char *puchMsg, 
                     unsigned short usDataLen 
                    );                             /* The function returns the CRC as a unsigned short type */
                    
extern const unsigned char auchCRCHi[];
extern const char auchCRCLo[];                    

#ifdef __cplusplus
}
#endif
					
#endif //__CRC_16_H
