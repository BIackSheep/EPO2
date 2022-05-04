#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <string.h>

#include "Robot_Header.h"

#define COMPORT "COM5"
#define BAUDRATE CBR_9600

int detection[2] = {0,1};
extern int instruction[2];

void initSio(HANDLE hSerial){

    COMMTIMEOUTS timeouts ={0};
    DCB dcbSerialParams = {0};

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        //error getting state
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("error getting state \n");
    }

    dcbSerialParams.BaudRate = BAUDRATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if(!SetCommState(hSerial, &dcbSerialParams)){
        //error setting serial port state
        printf("error setting state \n");
    }

    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;

    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if(!SetCommTimeouts(hSerial, &timeouts)){
    //error occureed. Inform user
        printf("error setting timeout state \n");
    }
}

//--------------------------------------------------------------
// Function: readByte
// Description: reads a single byte from the COM port into
//              buffer buffRead
//--------------------------------------------------------------
int readByte(HANDLE hSerial, int *buffRead) {

    DWORD dwBytesRead = 0;

    if (!ReadFile(hSerial, buffRead, 1, &dwBytesRead, NULL))
    {
        printf("error reading bytes from input buffer \n");
    }
    printf("Bytes read from read buffer are: '%i %i' \n", buffRead[0], buffRead[1]);
    return(0);
}

//--------------------------------------------------------------
// Function: writeByte
// Description: writes a single byte stored in buffRead to
//              the COM port
//--------------------------------------------------------------
int writeByte(HANDLE hSerial, int *buffWrite){

    DWORD dwBytesWritten = 0;

    if (!WriteFile(hSerial, buffWrite, 1, &dwBytesWritten, NULL))
    {
        printf("error writing bytes to output buffer \n");
    }
    printf("Bytes written to write buffer are: '%i %i' \n", buffWrite[0], buffWrite[1]);

    return(0);
}

int zigbee()
{
    HANDLE hSerial;

    //----------------------------------------------------------
    // Open COMPORT for reading and writing
    //----------------------------------------------------------
    hSerial = CreateFile(COMPORT,
        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        0
    );

    if(hSerial == INVALID_HANDLE_VALUE){
        if(GetLastError()== ERROR_FILE_NOT_FOUND){
            //serial port does not exist. Inform user.
            printf(" serial port does not exist \n");
        }
        //some other error occurred. Inform user.
        printf(" some other error occured. Inform user.\n");
    }

    //----------------------------------------------------------
    // Initialize the parameters of the COM port
    //----------------------------------------------------------

    initSio(hSerial);

    /*
    A while loop will be needed! first, the Robot_Algorithm.c is executed, initializing the maze, asking for station, etc. Then Zigbee.c has to loop until
    a crossing or mine is detected, which updates an external variable and quits the loop. The Robot_algorithm then gives a new instruction/calculates a new route,
    and then Zigbee.c is called again.
    */
    writeByte(hSerial, instruction);
    readByte(hSerial, detection);

    CloseHandle(hSerial);

    return 0;
}
