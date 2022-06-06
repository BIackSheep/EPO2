#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Robot_Header.h"

/*challenge a:
 The list with stations is given, no blocked edges.  (should be ready now (03-06-2022))

 challenge b:
 The list with stations is given. Some part has to keep track of the position of the robot.
 When a mine is detected, a blocked edge must be given to the algorithm together with the already
 existing blocked edges and stations, with the starting station at the current location (which isn't
 a classical station but an edge). Then the journey can continue. ((should be ready now (06-06-2022))

 challenge c:
 Exploration:
 The robot has to traverse the entire field (except station entries) in an as short as possible timespan. Everytime a mine is encountered it has to be entered again in the algorithm (but for the algorithm stations have to be given so it will have to be changed??)
 Treasure hunt: a new mine is placed, which has to be found in an as short as possible time.
 */

int maze[13][13];       /*is a matrix representation of all distinct locations on the real table*/
int *stations[12];      /*Every index corresponds to that station, with address in maze as content*/
int *crossings[5][5];   /*Every index corresponds to that crossing, with address in maze as content*/
int *route;             /*holds a route between 2 stations*/
int *totalroute;        /*holds a route between all stations*/
int totalroutelen;      /*contains the length of totalroute*/
int nr_of_stations;     /*contains the number of stations given as input*/
int input_len;          /*contains the number of blocked edges given as input*/
int *input_list;        /*list with blocked edges*/
int *stationinput;      /*list with stations to visit, beginning at station 0*/
int direction;          /*keeps track of the current direction of the robot*/
int current_index;      /*keeps track of the current index in the totalroute array*/
int passed_crossings;   /*keeps track of the number of crossings (or mines) that have been passed*/
int passed_stations;    /*keeps track nr of stations that have been visited (minus the start station)*/
int *optimalstations;   /*is a list of the stations in the order corresponding to totalroute*/
int *next_crossing;     /*keeps track of next station the robot visits, according to old totalroute*/
int challengec;         /*is true if challenge c is being done*/

/*external variable*/
int instruction[2] = {0,0}; /*contains the instructions for zigbee.c to send*/
int detection[2] = {0,0};   //xcode

int main(int argc, char const *argv[]) {
    int n;  /*temporary variable*/

    puts("Enter number of blocked edges:");
    /*first input, gives input list length*/
    scanf("%i",&input_len);

    if(input_len) {
        puts("Enter the blocked edges:");
        /*each edge has 3 array elements*/
        input_list = (int*)calloc(3*input_len, sizeof(int));
        if(!input_list) {
            fputs("Could not allocate that space!",stderr);
            exit(1);
        }
    
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
    }

    puts("Enter 1 if this is challenge c, otherwise enter 0");
    scanf("%i",&challengec);
    /*start the first part of challenge c*/
    explore();
    
    puts("Enter the number of stations:");
    /*gives number of stations*/
    scanf("%i",&nr_of_stations);
    
    /*contains a list of all stations in the order given at input*/
    stationinput = (int*)calloc(nr_of_stations, sizeof(int));
    if(!stationinput) {
        fputs("Could not allocate that space!",stderr);
        exit(2);
    }
    /*will contain a list of all stations in order of the optimal route*/
    optimalstations = (int*)calloc(nr_of_stations, sizeof(int));
    if(!optimalstations) {
        fputs("Could not allocate that space!",stderr);
        exit(3);
    }

    puts("Enter the stations, the first one is where you start:");
    /*reads stations*/
    for(n=0; n<nr_of_stations; ++n) {
        scanf("%i",&stationinput[n]);
    }

    /*allocates enough space for the total route*/
    /*totalroute = (int*)calloc(100,sizeof(int));
    if(!totalroute) {
        fputs("Could not allocate that space!",stderr);
        exit(4);
    }*/ /*not needed? totalroute is only given adresses in temptotalroute, which is allocated*/

    /*starts the search for the shortest route*/
    shortest_route(stationinput+1,stationinput+1,nr_of_stations-1);

    /*prints shortest route*/
    for(n=0;n<totalroutelen;n++) {
        if(totalroute[n]>=10) {
            printf("c%i ", totalroute[n]);
        }
        else {
            printf("c0%i ", totalroute[n]);
        }
    }
    puts("\n");

    /*initialisation of the location of the robot*/
    current_index = 0;
    passed_crossings = 0;
    passed_stations = 0;

    /*initial direction set towards the north*/
    if (stationinput[0] == 1 || stationinput[0] == 2 || stationinput[0] == 3){
        direction = 1;
    }
    /*initial direction set towards the east*/
    else if (stationinput[0] == 10 || stationinput[0] == 11 || stationinput[0] == 12){
        direction = 2;
    }
    /*initial direction set towards the south*/
    else if (stationinput[0] == 9 || stationinput[0] == 8 || stationinput[0] == 7){
        direction = 3;
    }
    /*initial direction set towards the west*/
    else if(stationinput[0] == 6 || stationinput[0] == 5 || stationinput[0] == 4){
        direction = 4;
    }

    /*will have to quit when the last station is reached (5 is thus temporary)*/
    while (detection[0]!=5&&current_index<totalroutelen) {

        /*has to be reset after zigbee has been called*/
        instruction[0] = 0;
        instruction[1] = 0;

        /*calls zigbee function, which will wait on detections*/
        //zigbee();     xcode

        /*mine detected*/
        if(detection[1]) {
            mine_detected();
        }

        /*crossing detected (is not set off for a mine)*/
        if (detection[0]) {
            current_crossing(stationinput);
        }

        /*for the temporary while condition, to quit the loop*/
        puts("enter 5 to quit the loop, 1 for a crossing\ndetection simulation, or 0 for continuing");
        scanf("%i",&detection[0]);
        puts("enter 1 for a mine detection simulation\nor 0 for continuing");
        scanf("%i",&detection[1]);
    }

    puts("final station has been reached!");
    
    /*clean up*/
    free(input_list);
    free(stationinput);
    free(optimalstations);
    /*following 2 may give issues? use if statement??*/
    free(totalroute);
    free(route);
    return 0;
}

void explore() {
    puts("Explore!");
}

void mine_detected() {
    int n=0;
    
    puts("mine detected!");
    
    /*resetting the mine detection*/
    detection[1] = 0;
    
    /*resetting the stationinput, to allow a new "input"*/
    for(n=0;n<nr_of_stations;n++) {
        stationinput[n] = 0;
    }
    
    /*special code for the Lee function to interpret as: start from a crossing, not a station*/
    stationinput[0]=13;
    
    /*passing the station that are yet to be passed to the stationinput*/
    for(n=1;n+passed_stations<nr_of_stations;++n) {
        stationinput[n] = optimalstations[n+passed_stations];
    }
    
    /*resetting the optimal stations array, to allow a shorter array to be filled in*/
    for(n=0;n<nr_of_stations;n++) {
        optimalstations[n] = 0;
    }
    
    /*to tell that there are less stations to be visited*/
    nr_of_stations -= passed_stations;
    
    /*to allow new counting, for this function to work again*/
    passed_stations = 0;
    
    /*extra blocked edge*/
    ++input_len;
    
    /*allocate extra memory for extra blocked edge*/
    input_list = realloc(input_list,3*input_len);
    if(!input_list) {
        fputs("Error reallocating memory!",stderr);
        exit(5);
    }
    
    /*puts the new blocked edge into the input_list*/
    if(direction==1) {
        input_list[3*(input_len-1)] = totalroute[current_index] / 10;
        input_list[3*(input_len-1)+1] = totalroute[current_index] % 10;
        input_list[3*(input_len-1)+2] = 115; /*'s'*/
    }
    else if(direction==2) {
        input_list[3*(input_len-1)] = totalroute[current_index-1] / 10;
        input_list[3*(input_len-1)+1] = totalroute[current_index-1] % 10;
        input_list[3*(input_len-1)+2] = 101; /*'e'*/
    }
    else if(direction==3) {
        input_list[3*(input_len-1)] = totalroute[current_index-1] / 10;
        input_list[3*(input_len-1)+1] = totalroute[current_index-1] % 10;
        input_list[3*(input_len-1)+2] = 115; /*'s'*/
    }
    else if(direction==4) {
        input_list[3*(input_len-1)] = totalroute[current_index] / 10;
        input_list[3*(input_len-1)+1] = totalroute[current_index] % 10;
        input_list[3*(input_len-1)+2] = 101; /*'e'*/
    }
    
    /*to use the crossing just passed as the next crossing, as location in the maze*/
    --current_index;
    
    /*determines the location of the next crossing that has not been visited yet, needed for Lee*/
    next_crossing = crossings[ totalroute[current_index] / 10 ][ totalroute[current_index] % 10 ];
    
    /*empty the totalroute array*/
    for(n=0;n<totalroutelen;n++) {
        totalroute[n] = 0;
    }
    totalroutelen = 0;
    
    /*sets the current_index to 0 for the new totalroute*/
    current_index = 0;
    
    /*because current_crossing isn't called, so the mine isn't counted as a crossing*/
    ++passed_crossings;
    
    /*starts the search for the shortest route*/
    shortest_route(stationinput+1,stationinput+1,nr_of_stations-1);
}

void current_crossing(int* stationinput) {
    puts("crossing detected");
    
    /*determines the direction the robot will have to take for the next station*/
    int case_argument = totalroute[current_index+1]-totalroute[current_index];
    /*for the last crossing*/
    if(current_index==totalroutelen-1) {
        case_argument = 0;
    }

    /*prints some information*/
    printf("current index: %i\ncurrent direction: %i\ncurrent case: %i\n", current_index, direction, case_argument);

    /*accounts for the mine spots that are also detected as crossings*/
    if (passed_crossings % 2 == 0) {
        /*facing north*/
        if (direction == 1) {
            /*determines the direction to take and the new orientation*/
            switch (case_argument) {
                /*turn right*/
                case 1:
                    instruction[0] = 0;
                    instruction[1] = 1;
                    direction = 2;
                    break;
                /*turn left*/
                case -1:
                    instruction[0] = 1;
                    instruction[1] = 0;
                    direction = 4;
                    break;
                /*straight on*/
                case -10:
                    instruction[0] = 1;
                    instruction[1] = 1;
                    direction = 1;
                    break;
                /*when a station must be visited*/
                case 0:
                    /*turn right*/
                    if (totalroute[current_index] %10 == 4){
                        instruction[0] = 0;
                        instruction[1] = 1;
                        direction = 4;
                    }
                    /*turn left*/
                    else if (totalroute[current_index] %10 == 0){
                        instruction[0] = 1;
                        instruction[1] = 0;
                        direction = 2;
                    }
                    /*straight on*/
                    else if (totalroute[current_index] < 4){
                        instruction[0] = 1;
                        instruction[1] = 1;
                        direction = 3;
                    }
                    /*because the same crossing will be passed twice*/
                    --passed_crossings;
                    
                    /*will be important for calculating new routes after detecting a mine*/
                    ++passed_stations;
                    break;
            }
        }
        /*facing east*/
        else if (direction == 2) {
            switch (case_argument) {
                /*straight on*/
                case 1:
                    instruction[0] = 1;
                    instruction[1] = 1;
                    direction = 2;
                    break;
                /*turn left*/
                case -10:
                    instruction[0] = 1;
                    instruction[1] = 0;
                    direction = 1;
                    break;
                /*turn right*/
                case 10:
                    instruction[0] = 0;
                    instruction[1] = 1;
                    direction = 3;
                    break;
                /*station*/
                case 0:
                    /*turn right*/
                    if (totalroute[current_index] > 40){
                        instruction[0] = 0;
                        instruction[1] = 1;
                        direction = 1;
                    }
                    /*turn left*/
                    else if (totalroute[current_index] < 4){
                        instruction[0] = 1;
                        instruction[1] = 0;
                        direction = 3;
                    }
                    /*straight on*/
                    else if (totalroute[current_index] == 14 || totalroute[current_index] == 24 || totalroute[current_index] == 34){
                        instruction[0] = 1;
                        instruction[1] = 1;
                        direction = 4;
                    }
                    --passed_crossings;
                    ++passed_stations;
                    break;
            }
        }
        /*facing south*/
        else if (direction == 3) {
            switch (case_argument) {
                /*turn left*/
                case +1:
                    instruction[0] = 1;
                    instruction[1] = 0;
                    direction = 2;
                    break;
                /*straight on*/
                case +10:
                    instruction[0] = 1;
                    instruction[1] = 1;
                    direction = 3;
                    break;
                /*turn right*/
                case -1:
                    instruction[0] = 0;
                    instruction[1] = 1;
                    direction = 4;
                    break;
                /*station*/
                case 0:
                    /*turn left*/
                    if (totalroute[current_index] %10 == 4){
                        instruction[0] = 1;
                        instruction[1] = 0;
                        direction = 4;
                    }
                    /*turn right*/
                    else if (totalroute[current_index] %10 == 0){
                        instruction[0] = 0;
                        instruction[1] = 1;
                        direction = 2;
                    }
                    /*straight on mate*/
                    else if (totalroute[current_index] > 40){
                        instruction[0] = 1;
                        instruction[1] = 1;
                        direction = 1;
                    }
                    --passed_crossings;
                    ++passed_stations;
                    break;
            }
        }
        /*facing west*/
        else if (direction == 4) {
            switch (case_argument) {
                /*straight on*/
                case -1:
                    instruction[0] = 1;
                    instruction[1] = 1;
                    direction = 4;
                    break;
                /*turn left*/
                case +10:
                    instruction[0] = 1;
                    instruction[1] = 0;
                    direction = 3;
                    break;
                /*turn right*/
                case -10:
                    instruction[0] = 0;
                    instruction[1] = 1;
                    direction = 1;
                    break;
                /*station*/
                case 0:
                    /*turn right*/
                    if (totalroute[current_index] < 4){
                        instruction[0] = 0;
                        instruction[1] = 1;
                        direction = 4;
                    }
                    /*straight on*/
                    else if (totalroute[current_index] %10 == 0){
                        instruction[0] = 1;
                        instruction[1] = 1;
                        direction = 2;
                    }
                    /*turn left*/
                    else if (totalroute[current_index] > 40){
                        instruction[0] = 1;
                        instruction[1] = 0;
                        direction = 1;
                    }
                    --passed_crossings;
                    ++passed_stations;
                    break;
            }
        }

        /*calls zigbee to follow the instruction*/
        //zigbee();     xcode

        /*updates the current location in the total route array*/
        ++current_index;
    }
    
    /*gives instruction when on an empty mine spot*/
    else{
        instruction[0] = 1;
        instruction[1] = 1;
        //zigbee();      xcode
    }
    
    /*updates the number of crossings that have been passed*/
    ++passed_crossings;
}

/*recursive permutating function, is also called first to start calculating shortest route,
this is why it is named like this but has the strange arguments*/
void shortest_route(int*new_comb,int *shifted_comb,int nr_stations)
{
    /*calculates the shortest route for a unique combinations of stations, when all possible
     permutations for a cycle have been considered*/
    if((shifted_comb-new_comb)==nr_stations-1) {
        routeconcat(new_comb);
        return;
    }

    /*creates a new tmparray for every instance of this recursive function*/
    /*the length nr_of_stations is used instead of nr_stations for routeconcat*/
    int *tmparray=(int*)calloc(nr_of_stations,sizeof(int));
    if(!tmparray) {
        fputs("Could not allocate that space!",stderr);
        exit(6);
    }

    /*copies the new combinations as it stands now to the tmparray*/
    memcpy(tmparray,new_comb,nr_stations*sizeof(int));

    /*calls the recursive functions, now with a shifted tmparray to be used as shifted_comb*/
    shortest_route(tmparray,tmparray+(shifted_comb-new_comb+1),nr_stations);

    /*To this loop will be returned after a calculation with routeconcat has taken place in
     a higher instance of the recursive function. This loop will call the recursive function with
     tmparray that has 2 of its numbers switched*/
    for (int h=1;h<nr_stations-(shifted_comb-new_comb);h++) {
        shortest_route(tmparray,permute(tmparray+(shifted_comb-new_comb),h),nr_stations);
    }

    /*free space*/
    free(tmparray);
}

/*switches the number in the given address with the number of address+h,
 returns given adress +1*/
int*permute(int*i,int h)
{
    int temp = *i;
    *i = *(i+h);
    *(i+h) = temp;
    return i+1;

}

void routeconcat(int *list)
{
    int m=0,n=0;
    int index = 0;
    int routelen = 0;
    int* temptotalroute;
    int temptotalroutelen = 0;
    /*allocates enough space for the total route*/
    temptotalroute = (int*)calloc(100,sizeof(int));
    if(!temptotalroute) {
        fputs("Could not allocate that space!",stderr);
        exit(7);
    }

    /*Because list does not contain the first station, because the robot always starts there*/
    for(n=nr_of_stations-1;n>0;n--) {
        list[n] = list[n-1];
    }
    list[0] = stationinput[0];
    
    /*calculates length of shortest route between every 2 stations*/
    for(n=0;n<nr_of_stations-1;++n) {
        /*fill the map as is stands now*/
        maze_init(input_len, input_list);
        
        puts("test1"); //the errors are thrown between test 1 and 2
        routelen = Lee(list[n],list[n+1]);
        puts("test2");
        
        temptotalroutelen += routelen;
        
        /*adds the route between 2 stations to the temporary total route*/
        for(m=0;m<routelen;m++,index++) {
            /*route has been set by Lee()*/
            temptotalroute[index] = route[m];
        }
        /*free the space that was allocated in lee()*/
        free(route);
    }

    /*print temporary total route*/
    for(m=0;m<index;m++) {
        if(temptotalroute[m]>=10) {
            printf("c%i ", temptotalroute[m]);
        }
        else {
            printf("c0%i ", temptotalroute[m]);
        }
    }
    puts("\n");

    /*if the calculated route is shorter than the already existing one, it is replaced*/
    if ((temptotalroutelen<totalroutelen)||totalroutelen==0) {
        free(totalroute);
        totalroute = temptotalroute;
        totalroutelen = temptotalroutelen;
        
        /*will be needed when a new route has to be calculated*/
        for(n=0;n<nr_of_stations;n++) {
            optimalstations[n] = list[n];
        }
        
    }
    else {
        /*frees the space for temptotalroute otherwise*/
        free(temptotalroute);
    }

    /*print total route*/ /*
    for(m=0;m<totalroutelen;m++) {
        if(totalroute[m]>=10) {
            printf("c%i ", totalroute[m]);
        }
        else {
            printf("c0%i ", totalroute[m]);
        }
    }
    puts("\n");*/

    /*Because list does contain the first station,
    this is not wanted for the function shortest_route*/
    for(n=0;n<nr_of_stations;n++) {
        list[n] = list[n+1];
    }

}

/*calculates shortest route between 2 stations, returns number of crossings*/
int Lee (int station1, int station2) {
    int count = 1;
    int n=0;        /*is used for indexing the route array*/
    int m=0;        /*keeps track of index in update_array*/
    int p=0;        /*keeps track of index in update_array_new*/
    
    int *beginstation; /*keeps track if the location of the begin station received a value*/
    int **update_array; /*contains all neighbouring cells that have to be updated with a higher value*/
    int **update_array_new; /*the same as above, but then for the next cycle*/

    update_array = (int**)calloc(30,sizeof(int*)); /*size 20 is maximum I found, 30 is for security*/
    if(!update_array) {
        fputs("Could not allocate that space!",stderr);
        exit(8);
    }
    update_array_new = (int**)calloc(30,sizeof(int*));
    if(!update_array_new) {
        fputs("Could not allocate that space!",stderr);
        exit(9);
    }

    /*places the location of the end station in update_array*/
    update_array[0] = stations[station2 - 1];
    
    /*Gives the end station value 1*/
    *update_array[0] = count;
    
    /*if the "beginstation" is a place in the middle of the map*/
    if(station1==13) {
        /*this old totalroute can still be used because this is the first calculation with Lee*/
        beginstation = next_crossing;
        /*places the "beginstation" on a crossing, neighbouring the requested crossing
         (normal stations also lie 2 places away from the first crossing)*/
        /*facing north*/
        if(direction==1) {
            beginstation += 26;
        }
        /*facing east*/
        else if(direction==2) {
            beginstation -= 2;
        }
        /*facing south*/
        else if(direction==3) {
            beginstation -= 26;
        }
        /*facing west*/
        else if(direction==4) {
            beginstation += 2;
        }
    }
    /*if the "beginstation" is a real station*/
    else {
        beginstation = stations[station1 - 1];
    }
    
    /*expand fase*/
    while(*beginstation==0) {

        ++count;                                    /*gives higher value to consecutive neighbours*/

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
            exit(10);
        }

    }

    /*implement a preference for straight on instead of making a turn?*/
    
    
    
    /*Trace back fase*/
    update_array[0] = beginstation;    /*repurposing of update_array for current and visited location*/
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
            exit(11);
        }

        m++;
        update_array[m] = update_array_new[0];

    }

    /*freeing the space if another instance of lee allocated space for route*/
    if(route) {
        free(route);
    }
    /*building the route array*/
    route = (int*)calloc(count,sizeof(int));
    if(!route) {
        fputs("Could not allocate that space!",stderr);
        exit(12);
    }

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
    puts("\n");*/
    

    /*free allocated memory*/
    free(update_array);
    free(update_array_new);

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
