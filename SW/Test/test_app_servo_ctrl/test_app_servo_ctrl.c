
#include <stdint.h> // uint16_t and family
#include <stdio.h> // printf and family
#include <string.h> // strerror()
#include <unistd.h> // file ops
#include <fcntl.h> // open() flags
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <stdlib.h>

#define DEV_FN "/dev/servo_ctrl"
#define USERNAME_MAX_LENGTH 15
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT   25565
#define N_SERVOS 4


void usage(FILE* f){
	fprintf(f,
"\nUsage: "\
"\n	test_app_servo_ctrl -h|--help"\
"\n		print this help i.e."\
"\n	test_app_servo_ctrl w servo_idx duty"\
"\n		write angle at servo_idx servo motor"
"\n	test_app_servo_ctrl r servo_idx"\
"\n		read feedback angle of servo_idx servo motor"\
"\n	servo_idx = [0, 4)"\
"\n	duty = [0, 1000]"\
"\n"
	);
}

static inline int c_str_eq(const char* a, const char* b) {
	return !strcmp(a, b);
}

int parse_args(
	int argc,
	char** argv,
	char* p_op,
	int* p_servo_idx,
	int* p_angle
) {
	if(argc == 2){
		if(c_str_eq(argv[1], "-h") || c_str_eq(argv[1], "--help")){
			// Print help.
			usage(stdout);
			return 0;
		}else{
			// Error.
			fprintf(stderr, "ERROR: Wrong argument \"%s\"!\n", argv[1]);
			usage(stderr);
			return 1;
		}
	}else if(argc == 3){
		if(!c_str_eq(argv[1], "r")){
			fprintf(stderr, "ERROR: Wrong command \"%s\"!\n", argv[1]);
			usage(stderr);
			return 2;
		}
		*p_op = 'r';
		int n;
		n = sscanf(argv[2], "%d", p_servo_idx);
		if(n != 1){
			fprintf(stderr, "ERROR: Invalid number \"%s\"!\n", argv[2]);
			return 3;
		}
	}else if(argc == 4){
		if(!c_str_eq(argv[1], "w")){
			fprintf(stderr, "ERROR: Wrong command \"%s\"!\n", argv[1]);
			usage(stderr);
			return 2;
		}
		*p_op = 'w';
		int n;
		n = sscanf(argv[2], "%d", p_servo_idx);
		if(n != 1){
			fprintf(stderr, "ERROR: Invalid number \"%s\"!\n", argv[2]);
			return 3;
		}
		n = sscanf(argv[3], "%d", p_angle);
		if(n != 1){
			fprintf(stderr, "ERROR: Invalid number \"%s\"!\n", argv[3]);
			return 3;
		}
	}else{
		// Error.
		fprintf(stderr, "ERROR: Wrong number of arguments!\n");
		usage(stderr);
		return 1;
	}
	//TODO limits
	return 0;
}


int main(int argc, char** argv){
    char op;
    int servo_idx;
    int duty;
    //int ret = parse_args(argc, argv, &op, &servo_idx, &duty);
    //if(ret){
    //	return ret;
    //}
	
    int udpListenSocket, tcpSocket, c, read_size;
    int startTCP = 0;
    struct sockaddr_in udpServer, client, tcpServer;
    char client_message[DEFAULT_BUFLEN];
    uint16_t duties[N_SERVOS] = {0};
    
    //Every raspberry will have its own username for the user to be able to easly differenciate his raspberry from the others
    char username[USERNAME_MAX_LENGTH];
    printf("Please enter your username (max. 15 characters):\n");
    gets(username);
    printf(username);
    
    udpListenSocket = socket(AF_INET , SOCK_DGRAM , 0);
    int broadcastEnable = 1;
    setsockopt(udpListenSocket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    udpServer.sin_family = AF_INET;
    udpServer.sin_addr.s_addr = INADDR_ANY;
    udpServer.sin_port = htons(DEFAULT_PORT);
    
    tcpSocket = socket(AF_INET , SOCK_STREAM , 0);
    tcpServer.sin_family = AF_INET;
    tcpServer.sin_addr.s_addr = INADDR_ANY;
    tcpServer.sin_port = htons(25566);
    
    
    
    //Create socket
    if (udpListenSocket == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");

    //Prepare the sockaddr_in structure
    

    //Bind
    if( bind(udpListenSocket,(struct sockaddr *)&udpServer , sizeof(udpServer)) < 0)
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
    while((read_size = recvfrom(udpListenSocket , client_message , DEFAULT_BUFLEN , 0, (struct sockaddr*)&client, &buff_size)) > -1)
    {
        printf("Bytes received: %d\n", read_size);
        client_message[read_size] = '\0';
	
	//When rasp gets "ping" message, its send its username to the PC
        if(strcmp("ping", client_message) == 0) 
        {
            if(sendto(udpListenSocket , username , strlen(username), 0, (struct sockaddr*) &client, buff_size) == -1)
            {
                puts("Send failed");
                return 1;
            }   
        }
	//We check if the PC has received our details and ends the UDP message listening 
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
    
    //Closing the UDP socket
    close(udpListenSocket);
    puts("UDP Socket Closed\n");
    
	
    //Binding the TCP socket
    if( bind(tcpSocket,(struct sockaddr *)&tcpServer , sizeof(tcpServer)) < 0)
    {
        //print the error message
        perror("bind failed. Error");
        return 1;
    }
    puts("bind done");
    
    //Listen to 1 PC at a time
    listen(tcpSocket , 1);

    while(1)
    {
	//Accept an incoming TCP connection
	puts("Waiting for incoming connections...");
	int client_sock = accept(tcpSocket, (struct sockaddr *)&tcpServer, (socklen_t*)&c);
	if (client_sock < 0)
	{
	    perror("accept failed");
	    return 1;
	}
	puts("Connection accepted");
    

	
	while( (read_size = recv(client_sock , client_message , DEFAULT_BUFLEN , 0)) > 0 )
	{
	    int fd;
	    fd = open(DEV_FN, O_RDWR);
	    if(fd < 0){
		    fprintf(stderr, "ERROR: \"%s\" not opened!\n", DEV_FN);
		    fprintf(stderr, "fd = %d %s\n", fd, strerror(-fd));
		    return 4;
	    }
	    printf("Bytes received: %d\n", read_size);
	    puts("ASDASD");
	    client_message[read_size] = '\0';
	    printf("%s\n", client_message);
	    char returnMessage[300];
	    char tempMessage[150];
	    char arg1 = client_message[0];
	
	    if(arg1 == 'w')
	    {
		int arg2 = (int)(client_message[2]) - 48;
		char arg3Char[3];
		int i = 0;
		for(i = 3; i < strlen(client_message); i++)
		{
		    arg3Char[i - 3] = client_message[i];
		}
		
		arg3Char[strlen(client_message) - 3] = '\0';
		int arg3 = atoi(arg3Char);
		 
		printf("duty = %d\n", arg3);
		sprintf(tempMessage, "duty = %d\n", arg3);
		strcat(returnMessage, tempMessage);
		tempMessage[0] = '\0';
	    
		duties[arg2] = arg3; // [permilles]
		
		for(int i = 0; i < N_SERVOS; i++){
		    sprintf(tempMessage, "duties[%d] = %d\n", i, duties[i]);
		    strcat(returnMessage, tempMessage);
		    printf("duties[%d] = %d\n", i, duties[i]);
		}
		
		int r = write(fd, (char*)&duties, sizeof(duties));
		if(r != sizeof(duties)){
		    fprintf(stderr, "ERROR: write went wrong!\n");
		    return 4;
		}
		printf("End.");
		strcat(returnMessage, "End.");
		printf(returnMessage);
		
		if( sendto(client_sock , returnMessage , strlen(returnMessage), 0, (struct sockaddr*) &client, buff_size) == -1)
		{
		    puts("Send failed");
		    return 1;
		}
		
		returnMessage[0] = '\0';
		tempMessage[0] = '\0';
		close(client_sock);
	    }else if(arg1 == 'r'){
		puts("Usao u read");
		int r = read(fd, (char*)&duties, sizeof(duties));
		if(r != sizeof(duties)){
			
			fprintf(stderr, "ERROR: read went wrong!\n");
			return 4;
		}
		for(int i = 0; i < N_SERVOS; i++){
			sprintf(tempMessage, "duties[%d] = %d\n", i, duties[i]);
			strcat(returnMessage, tempMessage);
			printf("duties[%d] = %d\n", i, duties[i]);
		}
		int arg2 = (int)(client_message[2]) - 48;
		printf("%d\n", arg2);
		duty = duties[arg2]; // [permilles]
		printf("duty = %d\n", duty);
		sprintf(tempMessage, "duty = %d\n", duty);
		strcat(returnMessage, tempMessage);
		
		
		if( sendto(client_sock , returnMessage , strlen(returnMessage), 0, (struct sockaddr*) &client, buff_size) == -1)
		{
		    puts("Send failed");
		    return 1;
		}
		
		returnMessage[0] = '\0';
		tempMessage[0] = '\0';
		close(client_sock);	
	    }
	
	    printf("End.\n");
	    close(fd);

	   
	}
	
	printf("Socket closed\n");
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
    

	

    return 0;
}
