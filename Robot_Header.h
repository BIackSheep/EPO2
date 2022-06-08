#ifndef ROBOT_HEADER_H_INCLUDED
#define ROBOT_HEADER_H_INCLUDED

/*for HANDLE type*/
#include <Windows.h>

extern int detection[2];
extern int instruction[2];

/*function prototypes*/
int Lee (int station1, int station2);
void shortest_route(int*new_comb,int *shifted_comb,int nr_stations);
void maze_init (int list_len, int* block_list);
void print_matrix (void);
void current_crossing(int* stationinput);
void mine_detected(void);
void routeconcat(int *list);
int *permute(int*i,int h);
void recursive_permute(int*i,int *j,int n);
void explore(void);
void treasure(void);

void initSio(HANDLE hSerial);
int readByte(HANDLE hSerial, char *buffRead);
int writeByte(HANDLE hSerial, char *buffWrite);
int zigbee(void);

#endif // ROBOT_HEADER_H_INCLUDED
