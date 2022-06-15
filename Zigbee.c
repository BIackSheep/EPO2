#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <string.h>
#include "Robot_Header.h"

#define COMPORT "COM6"
#define BAUDRATE CBR_9600

/*external variable*/
int detection[2] = {0,0};

/*--------------------------------------------------------------
 Function: initSio
 Description: intializes the parameters as Baudrate, Bytesize,
           Stopbits, Parity and Timeoutparameters of
           the COM port
--------------------------------------------------------------*/
void initSio(HANDLE hSerial){

    COMMTIMEOUTS timeouts ={0};
    DCB dcbSerialParams = {0};

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        /*error getting state*/
        printf("error getting state \n");
        exit(1);
    }

    dcbSerialParams.BaudRate = BAUDRATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if(!SetCommState(hSerial, &dcbSerialParams)){
        /*error setting serial port state*/
        printf("error setting state \n");
        exit(2);
    }

    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;

    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if(!SetCommTimeouts(hSerial, &timeouts)){
        /*error occureed. Inform user*/
        printf("error setting timeout state \n");
        exit(3);
    }
}

/*--------------------------------------------------------------
 Function: readByte
 Description: reads a single byte from the COM port into
              buffer buffRead
--------------------------------------------------------------*/
int readByte(HANDLE hSerial, char *buffRead) {

    DWORD dwBytesRead = 0;

    if (!ReadFile(hSerial, buffRead, 1, &dwBytesRead, NULL))
    {
        fputs("error reading byte from input buffer \n",stderr);
        exit(EXIT_FAILURE);
        exit(4);
    }
    return(0);
}

/*--------------------------------------------------------------
 Function: writeByte
 Description: writes a single byte stored in buffRead to
              the COM port
--------------------------------------------------------------*/
int writeByte(HANDLE hSerial, char *buffWrite){

    DWORD dwBytesWritten = 0;

    if (!WriteFile(hSerial, buffWrite, 1, &dwBytesWritten, NULL))
    {
        fputs("error writing byte to output buffer \n",stderr);
        exit(EXIT_FAILURE);
        exit(5);
    }
    return(0);
}

int zigbee()
{
    HANDLE hSerial;

    char byteBufferRead[BUFSIZ+1];
    char byteBufferWrite[BUFSIZ+1];
    byteBufferRead[0] = 96;

    /*----------------------------------------------------------
            Open COMPORT for reading and writing
    ----------------------------------------------------------*/
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
            /*serial port does not exist. Inform user.*/
            fputs(" serial port does not exist \n",stderr);
            exit(6);
        }
        /*some other error occurred. Inform user.*/
        else {
            printf(" some other error occured. Inform user.\n");
            exit(7);
        }
    }

    /*----------------------------------------------------------
     Initialize the parameters of the COM port
    ----------------------------------------------------------*/

    initSio(hSerial);

    /*sends instructions to VHDL*/
    if (instruction[0]==1||instruction[1]==1) {

        /*is reset to allow for new detection*/
        detection[0] = 0;

        /*this is for the biasing*/
        byteBufferWrite[0] = 96+2*instruction[0]+instruction[1];
        writeByte(hSerial, byteBufferWrite);

        /*check*/
        if (instruction[0]==0&&instruction[1]==1) {
            puts("right");
        }
        if (instruction[0]==1&&instruction[1]==0) {
            puts("left");
        }
        if (instruction[0]==1&&instruction[1]==1) {
            puts("straight on");
        }
    }
    /*waits for a detection*/
    else {
        while ( 1 ) {
            /*Waits for an (update in) input*/
            readByte(hSerial, byteBufferRead);

            /*One byte with information is sent. The byte is biased at 96 (011000 00). The last bit is 1 for a detected crossing (011000 01)(=97)*/
            if (byteBufferRead[0] == 97){
                /*a crossing is detected*/
                detection[0] = 1;
                break;
            }
            /*The second-to-last bit is for a detected mine. This does not come with a detected crossing (99) because the mine sensor is located
            if front of the line sensor*/
            if (byteBufferRead[0] == 98){
                /*a mine is detected*/
                detection[1] = 1;
                break;
            }
        }
    }

    CloseHandle(hSerial);

    return 0;
}
