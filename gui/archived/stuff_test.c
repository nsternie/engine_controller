#include <stdio.h>
#include "inttypes.h"

uint8_t a[] = {
0 ,
0 ,
0 ,
0 ,
57,
255 ,
57 ,
255 ,
200 ,
254 ,
57 ,
255 ,
57 ,
255 ,
57 ,
255 ,
57 ,
255 ,
50 ,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
97,
172 ,
0 ,
0 ,
0 ,
0 ,
82,
46,
17 ,
0 ,
10,
0 ,
244,
1 ,
0 ,
0 ,
0 ,
0 ,
204,
199,
1 ,
0 ,
155,
65,
2 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
84,
2 ,
90,
2 ,
89,
2 ,
89,
2 ,
86,
2 ,
85,
2 ,
86,
2 ,
46,
2 ,
72,
1 ,
0 ,
0 ,
249,
1 ,
0 ,
0 ,
101,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
0 ,
10,
0 ,
17,
1 ,
8 ,
1 ,
0 ,
0 ,
9 ,
144,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0};

uint8_t b[255];

uint8_t PACKET_SEP = '\n';
uint8_t PACKET_SIZE = 125;

//#define FinishBlock(X) (*code_ptr = (X), code_ptr = stuffed++, code = 0x01)

#define StartBlock()    (code_ptr = dst++, code = 1)
#define FinishBlock()   (*code_ptr = code)


void stuff_telem(uint8_t *ptr, uint8_t *dst){

    uint8_t length = PACKET_SIZE;
    // uint8_t *code_ptr = stuffed++;
    // uint8_t code = 0x01;

    const uint8_t *start = dst, *end = ptr + length;
    uint8_t code, *code_ptr; /* Where to insert the leading count */

    StartBlock();
    while (ptr < end) {
        if (code != 0xFF) {
            uint8_t c = *ptr++;
            if (c != '\n') {
                *dst++ = c;
                code++;
                continue;
            }
        }
        FinishBlock();
        StartBlock();
    }
    FinishBlock();

    // while (unstuffed < end){
    //     if (*unstuffed == PACKET_SEP)
    //         FinishBlock(code);
    //     else{
    //         *stuffed++ = *unstuffed;
    //         if (++code == 0xFF){
    //             FinishBlock(code);
    //         }
    //     }
    //     unstuffed++;
    // }
    // *stuffed = '\n';

    // FinishBlock(code);
}

int main(){
    stuff_telem(a, b);
    for(int n = 0; n < 130; n++){
        printf("%d:\t%d\r\n", n, b[n]);
    }
    return 0;
}



