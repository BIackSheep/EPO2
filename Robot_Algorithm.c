#include <stdio.h>
#include <stdlib.h>
#include "Robot_Header.h"

/*challenge a:
 The list with stations is given, no blocked edges.

 challenge b:
 The list with stations is given. Some part has to keep track of the position of the robot.
 When a mine is detected, a blocked edge must be given to the algorithm together with the already
 existing blocked edges and stations, with the starting station at the current location (which isn't
 a classical station but an edge). Then the journey can continue.

 a and b are implementable when a way of working with several stations is found. Besides that, of course communication with X-CTU or
 the self-made C program (prefered) has to be made, a way to funnel the sensor inputs into the algorithm (should not be too difficult)
 and a way to send instructions to the robot determined by current location and the algorithm.

 challenge c: (not yet implementable)
 Exploration:
 The robot has to traverse the entire field (except station entries) in an as short as possible timespan. Everytime a mine is encountered it has to be entered again in the algorithm (but for the algorithm stations have to be given so it will have to be changed??)
 Treasure hunt: a new mine is placed, which has to be found in an as short as possible time.
 */

/*pseudocode of instructions to the robot
 n = 0
 m = 0

 if (crossing_sensor = 1) {
 current_location = route[n]
 n++

 if(route[n] = left of current crossing) {
 instruction = left  ('10')
 else if (route[n] = right of current crossing) {
 instruction = right  ('01')
 else
 instruction = straight on  ('11')          backwards is '00', will this be needed?
 }
 }

 if (mine_sensor = 1) {
 input_list[m] = current_location(2 numbers) + instruction (letter)      add to input list options for north and west?
 maze_init()
 Lee()
 m++
 n--
 instruction = back  (not needed, will vhdl do itself)
 }
 */

int maze[13][13];
int *stations[12];
int *crossings[5][5];
int **update_array;
int **update_array_new;
int *route;             /*holds a route between 2 stations*/
int *totalroute;        /*holds a route between all stations*/
int nr_of_stations;

/*external variable*/
int instruction[2] = {0,0};

int main(int argc, char const *argv[]) {
    int input_len;
    int *input_list; /*list with blocked edges*/
    int *stationinput;   /*list with stations to visit, beginning at station 0*/
    int n;  /*temporary variable*/

    puts("Enter number of blocked edges:");
    /*first input, gives input list length*/
    scanf("%i",&input_len);

    puts("Enter the blocked edges:");
    /*each edge has 3 array elements*/
    input_list = (int*)calloc(3*input_len, sizeof(int));
    stationinput = (int*)calloc(input_len, sizeof(int));

    /*reads input_list*/
    for(n=0; n<(3*input_len); ++n) {
        /*reads the characters*/
        if(!((n+1)%3)) {
            /*reads \n or space */
            getchar();
            /*reads needed char*/
            input_list[n] = (int)getchar();
        }
        /*reads the integers*/
        else {
            scanf("%i",&input_list[n]);
        }
    }

    puts("Enter the number of stations:");
    /*gives number of stations*/
    scanf("%i",&nr_of_stations);

    puts("Enter the stations:");
    /*reads stations*/
    for(n=0; n<nr_of_stations; ++n) {
        scanf("%i",&stationinput[n]);
    }

    shortest_route(input_len, input_list, stationinput);


    /*will have to quit when the last station is reached*/
    n=0;
    while (n!=5) {

        /*has to be reset after zigbee has been called*/
        instruction[0] = 0;
        instruction[1] = 0;

        /*calls zigbee function, which will wait on detections*/
        zigbee();

        /*mine detected*/
        if(detection[1]) {
            detection[1] = 0;
            puts("mine detected!");
            /*new shortest route will be detected in a dedicated mine function*/
        }

        /*crossing detected*/
        if (detection[0]) {
            current_crossing(stationinput);
        }

        /*for the temporary while condition, to quit the loop*/
        puts("enter 5 to quit the loop");
        scanf("%i",&n);
    }

    /*clean up*/
    free(input_list);
    free(stationinput);
    return 0;
}

void shortest_route(int input_len, int *input_list, int *stationinput) {
    int m=0;
    int index = 0;
    int routelen = 0;

 /* is this too slow? C is pretty fast...*/
 recursive_permute(stationinput+1,stationinput+1,nr_of_stations-1);

    /*print total route*/
    puts("\n");
    for(m=0;m<index;m++) {
        if(totalroute[m]>=10) {
            printf("c%i ", totalroute[m]);
        }
        else {
            printf("c0%i ", totalroute[m]);
        }
    }
    puts("\n");
}


void routeconcat(int *num, int length)
{
    int i=0,n=0;
    for (i = 0 ; i < length ; i++) {
        printf("%d ", num[i]);
    }
    printf("\n");
    totalroute = (int*)calloc(100,sizeof(int));

    /*calculates shortest route between every 2 stations*/
     for(n=0;n<nr_of_stations-1;++n) {
        maze_init(input_len, input_list);
        routelen = Lee(stationinput[n],stationinput[n+1]);
        /*adds the route between 2 stations to the total route*/
        for(m=0;m<routelen;m++,index++) {
            totalroute[index] = route[m];
        }
     }
}

/*switches the number in the given address with the number of address+h*/
int*permute(int*i,int h)
{
     int temp = *i;
    *i = *(i+h);
    *(i+h) = temp;
return i+1;

}

void recursive_permute(int*i,int *j,int n)
{
    if((j-i)==n-1) {
        routeconcat(i,n);
        return;
    }

    int *tmparray=(int*)malloc(n*sizeof(int));
    memcpy(tmparray,i,n*sizeof(int));
    recursive_permute(tmparray,tmparray+(j-i+1),n);

    for (int h=1;h<n-(j-i);h++) {
        recursive_permute(tmparray,permute(tmparray+(j-i),h),n);
    }
}

/*calculates shortest route between 2 stations, returns number of crossings*/
int Lee (int station1, int station2) {
    int count = 1;
    int n=0;        /*is used for indexing the route array*/
    int m=0;        /*keeps track of index in update_array*/
    int p=0;        /*keeps track of index in update_array_new*/

    //test
    printf("station1 : %i\n",station1);
    printf("station2 : %i\n",station2);

    update_array = (int**)calloc(30,sizeof(int*)); /*what size? 20 is maximum I found, 30 is for security*/
    update_array_new = (int**)calloc(30,sizeof(int*));

    update_array[0] = stations[station2 - 1];
    *update_array[0] = count;

    /*expand fase*/
    while(*stations[station1 - 1]==0) {

        ++count;                                             /*gives higher value to consecutive neighbours*/

        while(update_array[m]!=NULL) {

            /*right neighbour*/
            if(*(update_array[m]+1)==0) {                    /*update_array[m] contains current neighbours of the location of last loop*/
                *(update_array[m]+1) = count;                /*gives higher value to the neighbour*/
                update_array_new[p] = update_array[m]+1;     /*makes a new update_array with neighbours to 'update' for next loop*/
                p++;                                         /*increases index for the new update array*/
            }

            /*left neighbour*/
            if(*(update_array[m]-1)==0) {
                *(update_array[m]-1) = count;
                update_array_new[p] = update_array[m]-1;
                p++;
            }

            /*above neighbour*/
            if((*(update_array[m]-13)==0)&&(update_array[m]-13>*maze)) {      /*one * still gives base adress of maze*/
                *(update_array[m]-13) = count;
                update_array_new[p] = update_array[m]-13;
                p++;
            }

            /*below neighbour*/
            if((*(update_array[m]+13)==0)&&(update_array[m]+13<*maze+169)) {
                *(update_array[m]+13) = count;
                update_array_new[p] = update_array[m]+13;
                p++;
            }

            m++;
        }

        /*makes update_array ready for transfer of update_array_new, also makes m=0*/
        while(m>0) {
            update_array[m-1] = NULL;
            m--;
        }

        /*transfers update_array_new to update_array, and empties it for new content*/
        while(p>0) {
            update_array[p-1] = update_array_new[p-1];
            update_array_new[p-1] = NULL;
            p--;
        }

        if(count>1000) {                /*arbitrary number of counts*/
            fputs("Error! No path was found between these stations!\n",stderr);
            exit(1);
        }

    }

    /*implement a preference for straight on instead of making a turn?*/
    /*Trace back fase*/
    update_array[0] = stations[station1 - 1];      /*repurposing of update_array for current and visited location*/
    update_array_new[0] = update_array[0];              /*repurposing of update_array_new for updating update_array*/
    m=0;    /*only for clarification, should already be 0*/

    while(update_array[m]!=stations[station2 - 1]) {
        /*right neighbour*/
        if((*(update_array[m]+1)<*(update_array[m]))&&(*(update_array[m]+1)>0)) {
            update_array_new[0] = update_array[m]+1;
        }

        /*left neighbour*/
        else if((*(update_array[m]-1)<*(update_array[m]))&&(*(update_array[m]-1)>0)) {
            update_array_new[0] = update_array[m]-1;
        }

        /*above neighbour*/
        else if(((*(update_array[m]-13)<*(update_array[m]))&&(update_array[m]-13>*maze))&&(*(update_array[m]-13)>0)) {
            update_array_new[0] = update_array[m]-13;
        }

        /*below neighbour*/
        else if((*(update_array[m]+13)<*(update_array[m]))&&(update_array[m]+13<*maze+169)&&(*(update_array[m]+13)>0)) {
            update_array_new[0] = update_array[m]+13;
        }

        else {
            fputs("Error! No path was found between these stations!\n",stderr);
            exit(1);
        }

        m++;
        update_array[m] = update_array_new[0];

    }

    /*building the route array*/
    route = (int*)calloc(count,sizeof(int));

    for(n=0;n<count;n++) {
        for(m=0;m<5;m++) {
            for(p=0;p<5;p++) {
                if(update_array[n] == crossings[m][p]) {
                    *(route+n) = (10*m)+p;
                }
            }
        }
    }

    /*removing the empty spaces in the route array*/
    m=0;
    for(n=1;n<count-1;n++) {
        if(!(n%2)) {
            *(route+m) = *(route+n);
            m++;
        }
    }

    /*print route*/ /*
    print_matrix();
    puts("\n");
    for(n=0;n<(count-3)/2;n++) {
        if(route[n]>=10) {
            printf("c%i ", route[n]);
        }
        else {
            printf("c0%i ", route[n]);
        }
    }
    puts("\n");
    */

    /*free allocated memory*/
    free(update_array);
    free(update_array_new);
    free(route);

    return (count-3)/2;
}

void maze_init (int list_len, int* block_list) {

    /*temporary variables*/
    int i=0, j=0, k=0, m=0;

    /*default matrix filler*/
    for(i=0;i<13;i++) {
        for(j=0;j<13;j++) {
            /*Unblocked edges between terminals 1-9, 2-8, 3-7, 4-12, 5-11, 6-10*/
            if(i==4||i==6||i==8||j==4||j==6||j==8) {
                maze[i][j] = 0;
            }
            /*Unblocked edges in the square i=2, i=10, j=2, j=10*/
            else if((i==2&&j<11&&j>1)||(i==10&&j<11&&j>1)||(j==2&&i<11&&i>1)||(j==10&&i<11&&i>1)) {
                maze[i][j] = 0;
            }
            /*Blocked points outside the tracks*/
            else {
                maze[i][j] = -1;
            }
        }
    }

    /*blocked edges*/
    for(k=0; k<list_len; ++k) {
        i = 2*block_list[3*k] + 2;            /*row*/
        j = 2*block_list[3*k+1] +2;           /*column*/
        if(block_list[3*k+2]==(int)'s') {     /*chooses edge to south or to east of specified cross point*/
            ++i;                              /*south*/
        }
        else {
            ++j;                              /*east*/
        }
        maze[i][j] = -1;
    }

    /*storing locations of the stations inside an array*/
    stations[0]  = &maze[12][4];    /*location of station 1 in maze*/
    stations[1]  = &maze[12][6];
    stations[2]  = &maze[12][8];
    stations[3]  = &maze[8][12];
    stations[4]  = &maze[6][12];
    stations[5]  = &maze[4][12];
    stations[6]  = &maze[0][8];
    stations[7]  = &maze[0][6];
    stations[8]  = &maze[0][4];
    stations[9]  = &maze[4][0];
    stations[10] = &maze[6][0];
    stations[11] = &maze[8][0];    /*station 12*/

    /*storing locations of the crossings inside a matrix*/
    for(i=0;i<5;i++) {
        for(j=0;j<5;++j) {
            k = 2+2*i;
            m = 2+2*j;
            crossings[i][j] = &maze[k][m];
        }
    }
}

void print_matrix (void) {
    /*temporary variables*/
    int i=0, j=0;
    /*column indication*/
    printf("   ]  1] 2] 3] 4] 5] 6] 7] 8] 9]10]11]12]13\n");
    /*matrix printer*/
    for(i=0;i<13;i++) {
        /*horizontal lines*/
        printf("___]___]__]__]__]__]__]__]__]__]__]__]__]__\n");
        /*row indication*/
        printf("%*i] ",3,i+1);
        for(j=0;j<13;j++) {
            printf("%*i]",2,maze[i][j]);
        }
        putchar('\n');
    }
}

void current_crossing(int* stationinput) {
    puts("current_crossing test");
    instruction[0] = 1;
    instruction[1] = 1;
    /*calls zigbee to follow the instruction*/
    zigbee();
}
