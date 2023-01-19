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
    struct sockaddr_in server, tcpServer;
    char *message = "ping";
    char server_message[DEFAULT_BUFLEN];
    char ipAdresa[20];
    
    
    //Create socket
    int b_sock = socket(AF_INET , SOCK_DGRAM , 0);
    int broadcastEnable = 1;
    sock = setsockopt(b_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (sock == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");

    //server.sin_addr.s_addr = inet_addr("10.1.207.255");
    server.sin_addr.s_addr = inet_addr("192.168.0.255");
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
        
        strcpy(ipAdresa, inet_ntoa(server.sin_addr));
        break;
    }
    
    printf("%s\n", ipAdresa);
    sleep(2);
    
    printf("ASD\n");
    
    
    tcpServer.sin_addr.s_addr = inet_addr(ipAdresa);
    tcpServer.sin_family = AF_INET;
    tcpServer.sin_port = htons(25566);
   
    int a = 0;
    
    while(a != -1)
    {
        
         char tcpMessage[20];
         
         gets(tcpMessage);
    int tcpSocket = socket(AF_INET , SOCK_STREAM , 0);
    
    if (tcpSocket == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");
        
        if (connect(tcpSocket , (struct sockaddr *)&tcpServer , sizeof(tcpServer)) < 0)
        {
            perror("connect failed. Error");
            return 1;
        }
    
    
   // char* tcpMsg = "./test_app_servo_ctrl w 3 125";
        char* tcpMsg = "r 3";
        if( send(tcpSocket , tcpMessage , strlen(tcpMessage), 0) < 0)
        {
            perror("send failed");
            return 1;
        
        }
        
        while( (read_size = recv(tcpSocket , server_message , DEFAULT_BUFLEN , 0)) > 0 )
        {
            server_message[read_size] = '\0';
            printf(server_message);
            break;
        }
        
        close(tcpSocket);
        
    }

    
    
    
    
    
    

    puts("Client message:");
    puts(message);

    return 0;
}

