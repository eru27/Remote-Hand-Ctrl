#include <stdio.h>      //printf
#include <string.h>     //strlen
#include <sys/socket.h> //socket
#include <arpa/inet.h>  //inet_addr
#include <fcntl.h>      //for open
#include <unistd.h>     //for close
#include <pthread.h>

#define MAX_RASP_COUNT 10
#define MAX_RASP_NAME 256

const unsigned short BROADCAST_PORT = 25565;
const unsigned short TCP_PORT = 25566;


pthread_mutex_t mutex_all = PTHREAD_MUTEX_INITIALIZER; //over array, console, rasp counter, terminate
pthread_cond_t cond_all = PTHREAD_COND_INITIALIZER; 
int terminate_listen = 0;
int raspbberies_count = 0;

struct raspberry
{
    char name[MAX_RASP_NAME];
    int index;
    struct sockaddr_in info;
};

int ping_rasp()
{
    int bcast_sock;
    int addr_len;
    struct sockaddr_in broadcast_addr;
    char *ping = "ping\0";

    int ret;

    bcast_sock = socket(AF_INET , SOCK_DGRAM , 0);
    if (bcast_sock == -1)
    {
        pthread_mutex_lock(&mutex_all); // Print error msg
            puts("Error: Could not create UDP socket\n");
        pthread_mutex_unlock(&mutex_all);

        return 1;
    }

    // Set socket broadcast options
    int broadcastEnable = 1;
    ret = setsockopt(bcast_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    if (ret == -1)
    {
        pthread_mutex_lock(&mutex_all); // Print error msg
            puts("Error: Could not set broadcast socket options. Check permitions.\n");
        pthread_mutex_unlock(&mutex_all);

        close(bcast_sock);

        return 2;
    }

    // Set address
    addr_len = sizeof(struct sockaddr_in);
    memset((void*)&broadcast_addr, 0, addr_len); // Init everything to '\0'

    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(BROADCAST_PORT);
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    // Send ping signal
    ret = sendto(bcast_sock, ping, strlen(ping), 0, (struct sockaddr*) &broadcast_addr, addr_len);
    
    if (ret != strlen(ping))
    {
        pthread_mutex_lock(&mutex_all); // Print error msg
            puts("Error: broadcast message not sent.\n");
        pthread_mutex_unlock(&mutex_all);

        close(bcast_sock);

        return 3;
    }

    close(bcast_sock);
    return 0;

}

int handle_raspberry(struct raspberry *rasp)
{
    //send back conformation

    //pthread_mutex_trylock()
        //cond var
        //check term

    //print rasp
    //unlock
    //signal cond
}

void listen_raspberries(struct raspberry raspberries[MAX_RASP_COUNT])
{
    //make socket

    //own ip and udp port struct

    //bind socket

    //for 0 to 9
        //check for term flag
            //break

        //listen

        //load info in array and call handle with pointer
    
    //close socket
}

int main()
{
    int ret_thr, ret_ping;
    struct raspberry raspberries[MAX_RASP_COUNT];

    //hi hello
    //lock mutex - so i can unlock it safely

    // Listen
    pthread_t listen_thr;
    ret_thr = pthread_create(&listen_thr, NULL, listen_raspberries, (void*) raspberries);

    //while Y
        //unlock mutex
        // Ping
        ret_ping = ping_rasp();
        if (ret_ping != 0)
        {
            //break
        }
        pthread_mutex_lock(&mutex_all);
        puts("Searching for available raspberries...\n");
        pthread_mutex_unlock(&mutex_all);

        //sleep_for
        //lock mutex
        //check yn

    //terminate
    //unlock mutex
    //broadcast cond var
    //ping listen_rasp to terminate

    //povezi se na malinu broj?

    //try tcp
    //comunicate
    //close sock

    //join listen


    return 0;
}