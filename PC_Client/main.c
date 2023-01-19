#include <stdio.h>      //printf
#include <string.h>     //strlen
#include <sys/socket.h> //socket
#include <arpa/inet.h>  //inet_addr
#include <fcntl.h>      //for open
#include <unistd.h>     //for close
#include <pthread.h>

#define MAX_RASP_COUNT 10
#define DEFAULT_BUFF_SIZE 256

const unsigned short BROADCAST_PORT = 25565;
const unsigned short TCP_PORT = 25566;


pthread_mutex_t mutex_all = PTHREAD_MUTEX_INITIALIZER; //over array, console, rasp counter, terminate
pthread_cond_t cond_all = PTHREAD_COND_INITIALIZER; 
int terminate_listen = 0;
int raspbberies_count = 0;

struct raspberry
{
    char name[DEFAULT_BUFF_SIZE];
    struct in_addr info;
};

int ping_rasp()
{
    int bcast_sock;
    int addr_len;
    struct sockaddr_in broadcast_addr;
    char *ping = "ping\0";

    int ret;

    // Make socket
    bcast_sock = socket(AF_INET , SOCK_DGRAM , 0);
    if (bcast_sock == -1)
    {
        puts("Error: Could not create UDP socket\n");

        return 1;
    }

    // Set socket broadcast options
    int broadcastEnable = 1;
    ret = setsockopt(bcast_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (ret == -1)
    {
        puts("Error: Could not set broadcast socket options. Check permitions.\n");

        close(bcast_sock);

        return 2;
    }

    // Set address
    addr_len = sizeof(struct sockaddr_in);
    memset((void*)&broadcast_addr, 0, addr_len); // Init everything to '\0'

    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(BROADCAST_PORT);
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); //konverzija ??????

    // Send ping signal
    ret = sendto(bcast_sock, ping, strlen(ping), 0, (struct sockaddr*) &broadcast_addr, addr_len);
    
    if (ret != strlen(ping))
    {
        puts("Error: broadcast message not sent.\n");

        close(bcast_sock);

        return 3;
    }

    close(bcast_sock);

    return 0;
}

void print_raspberries(struct raspberry raspberries[MAX_RASP_COUNT])
{
    puts("Raspberries available for connection: \n");

    int i;
    for (i = 0; i < raspbberies_count; i++)
    {
        printf("%d. %s\n", i, raspberries[i].name);
    }

    printf("\n");
}

int ping_listen()
{
    int sock;
    int addr_len;
    struct sockaddr_in address;
    char *message = "ping\0";

    int ret;

    sock = socket(AF_INET, SOCK_DGRAM , 0);
    if (sock == -1)
    {
        puts("Error: Could not create UDP socket2.\n");

        return 1;
    }

    memset(&address, 0, sizeof(address));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons(25565);

    int buff_size = sizeof(struct sockaddr_in); 
    if(sendto(sock , message , strlen(message), 0, (struct sockaddr*) &address, buff_size) == -1)
    {
        puts("Error: Could not ping listen. Listening socket might not close.\n");

        close(sock);

        return 1;
    }

    close(sock);

    return 0;
}

static void *listen_raspberries(void *rasp)
{
    //make socket
    int sock;
    int addr_len;
    struct sockaddr_in address, raspberry_adr;
    socklen_t addrlen;
    int i;
    char *message = "ping\0";
    char *return_message = "Received details\0";

    struct raspberry *raspberries = (struct raspberry *) rasp;

    int ret;

    sock = socket(AF_INET, SOCK_DGRAM , 0);
    if (sock == -1)
    {
        pthread_mutex_lock(&mutex_all);
            puts("Error: Could not create UDP listening socket.\n");
            puts("Terminating thread...\n");
        pthread_mutex_unlock(&mutex_all);

        return;
    }

    //own ip and udp port struct // ---ip any---
    memset(&address, 0, sizeof(address));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("0.0.0.0");
    address.sin_port = htons(25565);

    //bind socket
    if (bind(sock, (const struct sockaddr *)&address, sizeof(address)) < 0)
    {
        pthread_mutex_lock(&mutex_all);
            puts("Error: Binding listening socket failed.\n");
            puts("Terminating thread...\n");
        pthread_mutex_unlock(&mutex_all);

        close(sock);

        return;
    }

    //for 0 to 9
    for (int i = 0; i < MAX_RASP_COUNT; i++)
    {
        pthread_mutex_lock(&mutex_all);
        //check for term flag
        

        //listen
        addrlen = sizeof(raspberry_adr);

        pthread_mutex_unlock(&mutex_all);
        if(recvfrom(sock, raspberries[raspbberies_count].name, DEFAULT_BUFF_SIZE, 0, &raspberry_adr, &addrlen) < 0)
        {
            pthread_mutex_lock(&mutex_all);
            puts("Error: Failed to recieve.\n");
            pthread_mutex_unlock(&mutex_all);

            if (terminate_listen == 1)
            {
                break;
            }

            continue;
        }

        pthread_mutex_lock(&mutex_all);

        if (terminate_listen == 1)
        {
            pthread_mutex_unlock(&mutex_all);
            break;
        }

        raspberries[raspbberies_count].info = raspberry_adr.sin_addr;
        raspbberies_count++;

        //load info in array and call handle with pointer
            //send back conformation
        if(sendto(sock , return_message , strlen(return_message), 0, (struct sockaddr*) &raspberries[i].info, addrlen) == -1)
        {
            puts("Error: Failed to send a conformation.");
        }  

        pthread_mutex_unlock(&mutex_all);
    }
    
    close(sock);
}

int check_arguments(int index, char mode, int servo_id, int duty)
{
    int oke = 1;
    if (index < 0 || index > (raspbberies_count - 1))
    {
        oke = 0;
    }
    if (mode != 'w' && mode != 'r')
    {
        oke = 0;
    }
    if (servo_id < 0 || servo_id > 3)
    {
        oke = 0;
    }
    if (duty < 0 || duty > 1000)
    {
        oke = 0;
    }

    return oke;
}

int main()
{
    int ret_thr, ret_ping;
    struct raspberry raspberries[MAX_RASP_COUNT];

    char input;
    int i;

    //hi hello
    pthread_mutex_lock(&mutex_all); //lock mutex - so i can unlock it safely

    // Listen
    pthread_t listen_thr;
    ret_thr = pthread_create(&listen_thr, NULL, &listen_raspberries, (void*) raspberries);

    do
    {
        // Ping
        ret_ping = ping_rasp();
        if (ret_ping != 0)
        {
            break;
        }

        puts("Searching for available raspberries...\n");

        pthread_mutex_unlock(&mutex_all);

        // So listen can execute
        sleep(5);

        pthread_mutex_lock(&mutex_all);

        print_raspberries(raspberries);

        // Check yn
        printf("Check again? (Y/n) ");
        scanf("%c", &input);
        printf("\n");
    } while (input == 'Y' || input == 'y');
    
    //locked mutex
    terminate_listen = 1;
    ping_listen();

    pthread_mutex_unlock(&mutex_all);

    pthread_join(listen_thr, NULL);

    int index, servo_id, duty;
    char mode;
    char command[DEFAULT_BUFF_SIZE] = {};
    
    

    while (1)
    {
        printf("\nFormat komande: [broj zeljene maline] [r/w] [broj motora (0-3)] [w ? ugao pomeranja (0 - 1000)]. Za izlazak upisite 'exit'.\n");
        fgets(command, DEFAULT_BUFF_SIZE, stdin);
        
        if (command[0] == 'e')
        {
            break;
        }
        if (command[2] == 'w')
        {
            sscanf(command, "%d %c %d %d", &index, &mode, &servo_id, &duty);
        }
        else if (command[2] == 'r')
        {
            sscanf(command, "%d %c %d %d", &index, &mode, &servo_id);
            duty = 0;
        }
        else
        {
            puts("Error: Invalid input format.\n");
            continue;
        }

        if (!check_arguments(index, mode, servo_id, duty))
        {
            puts("Error: Invalid input values.\n");
            continue;
        }

        int tcpSocket = socket(AF_INET , SOCK_STREAM , 0);

        struct sockaddr_in current_rasp;
        current_rasp.sin_family = AF_INET;
        current_rasp.sin_port = htons(25566);
        
        if (tcpSocket == -1)
        {
            printf("Error: Could not create TCP socket.");
            return 1;
        }

        current_rasp.sin_addr.s_addr = raspberries[index].info.s_addr;

        if (connect(tcpSocket, (struct sockaddr *)&current_rasp, sizeof(current_rasp)) < 0)
        {
            puts("Error: Connection failed.\n");
            continue;
        }

        char *tcpMessage[DEFAULT_BUFF_SIZE] = {};
        sprintf(tcpMessage, "%c %d %d", mode, servo_id, duty);

        if(send(tcpSocket , tcpMessage , strlen(tcpMessage), 0) < 0)
        {
            puts("Error: Send failed.\n");
            close(tcpSocket);
            continue;
        
        }

        int read_size;
        char *server_message;

        while((read_size = recv(tcpSocket , server_message , DEFAULT_BUFF_SIZE , 0)) > 0 )
        {
            server_message[read_size] = '\0';
            puts(server_message);
            break;
        }
        
        close(tcpSocket);
        

    //try tcp
    //comunicate
    //close sock
    }
    //join listen


    return 0;
}