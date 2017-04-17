#include <avr/pgmspace.h>
const float Elemax=82.29;
const uint8_t EleAngle[] PROGMEM={0,16,38,59,81,102,123,144,164,183,200,215,223,223,214,200,183,164,144,123,102,80,59,37,16,0, 
0,17,38,60,81,103,124,145,165,184,202,216,224,224,215,201,183,164,144,123,102,81,59,38,16,0, 
0,18,39,61,82,103,125,145,166,185,203,217,225,225,216,202,184,165,144,123,102,81,59,38,16,0, 
0,18,40,61,83,104,125,146,166,186,203,218,227,226,217,202,184,165,145,124,103,81,60,38,17,0, 
3,19,40,62,83,105,126,147,167,187,204,219,228,227,218,203,185,165,145,124,103,81,60,38,17,0, 
2,19,41,63,84,105,127,148,168,188,205,220,229,228,219,204,185,166,145,124,103,82,60,39,17,0, 
1,20,42,63,85,106,127,148,169,188,206,221,230,229,220,204,186,166,146,125,103,82,60,39,17,0, 
1,21,42,64,85,107,128,149,170,189,207,222,231,231,220,205,186,166,146,125,103,82,61,39,18,0, 
0,21,43,64,86,107,129,150,170,190,208,224,233,232,221,205,187,167,146,125,104,82,61,39,18,0, 
1,22,43,65,86,108,129,150,171,191,209,225,234,233,222,206,187,167,146,125,104,82,61,40,18,0, 
1,23,44,66,87,109,130,151,172,192,210,226,235,234,223,207,188,167,147,126,104,83,61,40,18,0, 
2,23,45,66,88,109,130,152,172,192,211,227,236,235,224,207,188,168,147,126,104,83,61,40,19,0, 
3,24,45,67,88,110,131,152,173,193,212,228,237,236,224,208,188,168,147,126,105,83,62,40,19,2, 
3,24,46,67,89,110,132,153,174,194,213,229,238,237,225,208,189,168,147,126,105,83,62,40,19,2, 
4,25,46,68,89,111,132,153,174,195,214,230,239,238,226,209,189,169,148,126,105,84,62,41,19,2, 
4,26,47,68,90,111,133,154,175,195,214,231,241,239,226,209,190,169,148,127,105,84,62,41,20,2, 
5,26,48,69,90,112,133,155,176,196,215,232,242,240,227,210,190,169,148,127,105,84,63,41,20,1, 
5,27,48,69,91,112,134,155,176,197,216,233,243,241,228,210,190,170,148,127,106,84,63,41,20,1, 
6,27,49,70,92,113,134,156,177,197,217,234,244,242,228,210,191,170,149,127,106,84,63,42,20,1, 
7,28,49,71,92,114,135,156,177,198,218,235,245,242,229,211,191,170,149,128,106,85,63,42,20,1, 
7,28,50,71,92,114,135,157,178,199,218,235,246,243,230,211,191,170,149,128,106,85,63,42,21,0, 
8,29,50,72,93,115,136,157,179,199,219,236,247,244,230,212,192,171,149,128,106,85,64,42,21,0, 
8,29,51,72,93,115,136,158,179,200,220,237,248,245,231,212,192,171,150,128,107,85,64,42,21,0, 
9,30,51,72,94,115,137,158,180,200,220,238,249,246,231,212,192,171,150,128,107,85,64,43,21,0, 
9,30,52,73,94,116,137,159,180,201,221,239,250,247,232,213,192,171,150,129,107,86,64,43,22,1, 
10,31,52,73,95,116,138,159,181,201,222,240,251,248,232,213,193,172,150,129,107,86,64,43,22,1, 
10,31,52,74,95,117,138,160,181,202,222,240,252,248,233,214,193,172,150,129,107,86,65,43,22,1, 
11,32,53,74,96,117,139,160,181,202,223,241,253,249,233,214,193,172,151,129,108,86,65,44,22,2, 
11,32,53,75,96,118,139,160,182,203,223,242,254,250,234,214,193,172,151,129,108,86,65,44,23,2, 
12,33,54,75,96,118,139,161,182,203,224,243,255,251,234,214,194,172,151,129,108,87,65,44,23,2};