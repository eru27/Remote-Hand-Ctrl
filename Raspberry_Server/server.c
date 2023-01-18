/* 
    ********************************************************************
    Odsek:          Elektrotehnika i racunarstvo
    Departman:      Racunarstvo i automatika
    Katedra:        Racunarska tehnika i racunarske komunikacije (RT-RK)
    Predmet:        Osnovi Racunarskih Mreza 1
    Godina studija: Treca (III)
    Skolska godina: 2021/22
    Semestar:       Zimski (V)
    
    Ime fajla:      server.c
    Opis:           UDP server
    
    Platforma:      Raspberry Pi 2 - Model B
    OS:             Raspbian
    ********************************************************************
*/

#include<stdio.h>
#include<string.h>    //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include<unistd.h>    //write

#define USERNAME_MAX_LENGTH 15
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT   25565

int main(int argc , char *argv[])
{
    int socket_desc , c , read_size;
    int startTCP = 0;
    struct sockaddr_in server , client, tcpServer;
    char client_message[DEFAULT_BUFLEN];
   
    char username[USERNAME_MAX_LENGTH];
    
    printf("Please enter your username (max. 15 characters):\n");
    gets(username);
    printf(username);
    
    //Create socket
    socket_desc = socket(AF_INET , SOCK_DGRAM , 0);
    int broadcastEnable = 1;
    int sock = setsockopt(socket_desc, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (sock == -1)
    {
        printf("Could not create socket");
    }
    if (socket_desc == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(DEFAULT_PORT);

    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        //print the error message
        perror("bind failed. Error");
        return 1;
    }
    puts("bind done");

    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);

    //Receive a message from client
    int buff_size = sizeof(struct sockaddr_in); 
    while((read_size = recvfrom(socket_desc , client_message , DEFAULT_BUFLEN , 0, (struct sockaddr*)&client, &buff_size)) > -1)
    {
        printf("Bytes received: %d\n", read_size);
        client_message[read_size] = '\0';
      
        if(strcmp("ping", client_message) == 0) 
        {
            if(sendto(socket_desc , username , strlen(username), 0, (struct sockaddr*) &client, buff_size) == -1)
            {
                puts("Send failed");
                return 1;
            }   
        }
        else if(strcmp("Received details", client_message) == 0)
        {
            break;
        }    
        printf("Client message: %s\n", client_message);
    }   
    if(read_size == -1)
    {
        perror("recv failed");
    }
    
    
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(DEFAULT_PORT);
    
    
    int tcpSocket = socket(AF_INET , SOCK_STREAM , 0);
    tcpServer.sin_family = AF_INET;
    tcpServer.sin_addr.s_addr = INADDR_ANY;
    tcpServer.sin_port = htons(25566);
    
    
    if( bind(tcpSocket,(struct sockaddr *)&tcpServer , sizeof(tcpServer)) < 0)
    {
        //print the error message
        perror("bind failed. Error");
        return 1;
    }
    puts("bind done");

    //Listen
    listen(tcpSocket , 1);
    
    int client_sock = accept(tcpSocket, (struct sockaddr *)&tcpServer, (socklen_t*)&c);
    if (client_sock < 0)
    {
        perror("accept failed");
        return 1;
    }
    puts("Connection accepted");

    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    
    while( (read_size = recv(client_sock , client_message , DEFAULT_BUFLEN , 0)) > 0 )
    {
        printf("Bytes received: %d\n", read_size);
        client_message[read_size] = '\0';
        printf(client_message);
        system(client_message);
    }

    if(read_size == 0)
    {
        puts("Client disconnected");
        fflush(stdout);
    }
    else if(read_size == -1)
    {
        perror("recv failed");
    }
    //getchar();
    close(socket_desc);
    puts("\n    socket closed");

    return 0;
}

