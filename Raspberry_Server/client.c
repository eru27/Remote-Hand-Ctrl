/* 
    ********************************************************************
    Odsek:          Elektrotehnika i racunarstvo
    Departman:      Racunarstvo i automatika
    Katedra:        Racunarska tehnika i racunarske komunikacije (RT-RK)
    Predmet:        Osnovi Racunarskih Mreza 1
    Godina studija: Treca (III)
    Skolska godina: 2021/22
    Semestar:       Zimski (V)
    
    Ime fajla:      client.c
    Opis:           TCP/IP klijent
    
    Platforma:      Raspberry Pi 2 - Model B
    OS:             Raspbian
    ********************************************************************
*/

#include<stdio.h>      //printf
#include<string.h>     //strlen
#include<sys/socket.h> //socket
#include<arpa/inet.h>  //inet_addr
#include <fcntl.h>     //for open
#include <unistd.h>    //for close

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT   25565

int main(int argc , char *argv[])
{
    int sock;
    struct sockaddr_in server;
    char *message = "ping";
    char server_message[DEFAULT_BUFLEN];
    //Create socket
    int b_sock = socket(AF_INET , SOCK_DGRAM , 0);
    int broadcastEnable = 1;
    sock = setsockopt(b_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (sock == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");

    server.sin_addr.s_addr = inet_addr("10.1.207.255");
    server.sin_family = AF_INET;
    server.sin_port = htons(DEFAULT_PORT);

    //Send some data
    int buff_size = sizeof(struct sockaddr_in); 
    if( sendto(b_sock , message , strlen(message), 0, (struct sockaddr*) &server, buff_size) == -1)
    {
        puts("Send failed");
        
        return 1;
    }
    
    server.sin_addr.s_addr = INADDR_ANY;
    int read_size;
    while((read_size = recvfrom(b_sock , server_message , DEFAULT_BUFLEN , 0, (struct sockaddr*)&server, &buff_size)) > -1)
    {
        printf("Bytes received: %d\n", read_size);
        server_message[read_size] = '\0';
        printf("Client message: %s\n", server_message);
        char* endMessage = "Received details";
        if( sendto(b_sock , endMessage , strlen(endMessage), 0, (struct sockaddr*) &server, buff_size) == -1)
        {
            puts("Send failed");
            return 1;
        }
        
    }
    

    puts("Client message:");
    puts(message);

    return 0;
}

