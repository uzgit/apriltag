/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdlib.h>
#include "tagCustom48h12.h"

static uint64_t codedata[10] = {
   0x0000d6c8ae76dff0UL,
   0x0000d6c8e905e5b5UL,
   0x0000d6c92394eb7aUL,
   0x0000d6c95e23f13fUL,
   0x0000d6c998b2f704UL,
   0x0000d6c9d341fcc9UL,
   0x0000d6ca0dd1028eUL,
   0x0000d6ca48600853UL,
   0x0000d6ca82ef0e18UL,
};
apriltag_family_t *tagCustom48h12_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagCustom48h12");
   tf->h = 12;
   tf->ncodes = 10;
   tf->codes = codedata;
   tf->nbits = 48;
   tf->bit_x = calloc(48, sizeof(uint32_t));
   tf->bit_y = calloc(48, sizeof(uint32_t));
   tf->bit_x[0] = -2;
   tf->bit_y[0] = -2;
   tf->bit_x[1] = -1;
   tf->bit_y[1] = -2;
   tf->bit_x[2] = 0;
   tf->bit_y[2] = -2;
   tf->bit_x[3] = 1;
   tf->bit_y[3] = -2;
   tf->bit_x[4] = 2;
   tf->bit_y[4] = -2;
   tf->bit_x[5] = 3;
   tf->bit_y[5] = -2;
   tf->bit_x[6] = 4;
   tf->bit_y[6] = -2;
   tf->bit_x[7] = 5;
   tf->bit_y[7] = -2;
   tf->bit_x[8] = 6;
   tf->bit_y[8] = -2;
   tf->bit_x[9] = 1;
   tf->bit_y[9] = 1;
   tf->bit_x[10] = 2;
   tf->bit_y[10] = 1;
   tf->bit_x[11] = 3;
   tf->bit_y[11] = 1;
   tf->bit_x[12] = 7;
   tf->bit_y[12] = -2;
   tf->bit_x[13] = 7;
   tf->bit_y[13] = -1;
   tf->bit_x[14] = 7;
   tf->bit_y[14] = 0;
   tf->bit_x[15] = 7;
   tf->bit_y[15] = 1;
   tf->bit_x[16] = 7;
   tf->bit_y[16] = 2;
   tf->bit_x[17] = 7;
   tf->bit_y[17] = 3;
   tf->bit_x[18] = 7;
   tf->bit_y[18] = 4;
   tf->bit_x[19] = 7;
   tf->bit_y[19] = 5;
   tf->bit_x[20] = 7;
   tf->bit_y[20] = 6;
   tf->bit_x[21] = 4;
   tf->bit_y[21] = 1;
   tf->bit_x[22] = 4;
   tf->bit_y[22] = 2;
   tf->bit_x[23] = 4;
   tf->bit_y[23] = 3;
   tf->bit_x[24] = 7;
   tf->bit_y[24] = 7;
   tf->bit_x[25] = 6;
   tf->bit_y[25] = 7;
   tf->bit_x[26] = 5;
   tf->bit_y[26] = 7;
   tf->bit_x[27] = 4;
   tf->bit_y[27] = 7;
   tf->bit_x[28] = 3;
   tf->bit_y[28] = 7;
   tf->bit_x[29] = 2;
   tf->bit_y[29] = 7;
   tf->bit_x[30] = 1;
   tf->bit_y[30] = 7;
   tf->bit_x[31] = 0;
   tf->bit_y[31] = 7;
   tf->bit_x[32] = -1;
   tf->bit_y[32] = 7;
   tf->bit_x[33] = 4;
   tf->bit_y[33] = 4;
   tf->bit_x[34] = 3;
   tf->bit_y[34] = 4;
   tf->bit_x[35] = 2;
   tf->bit_y[35] = 4;
   tf->bit_x[36] = -2;
   tf->bit_y[36] = 7;
   tf->bit_x[37] = -2;
   tf->bit_y[37] = 6;
   tf->bit_x[38] = -2;
   tf->bit_y[38] = 5;
   tf->bit_x[39] = -2;
   tf->bit_y[39] = 4;
   tf->bit_x[40] = -2;
   tf->bit_y[40] = 3;
   tf->bit_x[41] = -2;
   tf->bit_y[41] = 2;
   tf->bit_x[42] = -2;
   tf->bit_y[42] = 1;
   tf->bit_x[43] = -2;
   tf->bit_y[43] = 0;
   tf->bit_x[44] = -2;
   tf->bit_y[44] = -1;
   tf->bit_x[45] = 1;
   tf->bit_y[45] = 4;
   tf->bit_x[46] = 1;
   tf->bit_y[46] = 3;
   tf->bit_x[47] = 1;
   tf->bit_y[47] = 2;
   tf->width_at_border = 6;
   tf->total_width = 10;
   tf->reversed_border = true;
   return tf;
}

void tagCustom48h12_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
