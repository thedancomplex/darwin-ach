/*******************************************************************************
* Copyright 2022 Daniel M. Lofaro
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Daniel M. Lofaro */


#if !defined(LOFARO_COMS)
#define LOFARO_COMS 1
#endif
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define BUFSIZE 1024

#include <stdio.h>

namespace coms
{

  struct sockaddr_in serveraddr;
  int serverlen;

  typedef struct motor_state_def {
	double pos;	
	double speed;
	double load;	
	double voltage;	
	double temp;		
  }__attribute__((packed)) motor_state_def_t;

  typedef struct vector_3_def {
	double x;	
	double y;
	double z;	
  }__attribute__((packed)) vector_3_def_t;

  typedef struct imu_state_def {
	vector_3_def_t linear[3];
	vector_3_def_t angular[3];
  }__attribute__((packed)) imu_state_def_t;

  motor_state_def_t motor_state[20];

  void error(char *msg) {
    perror(msg);
    exit(0);
  }

  int system_type   = 0;
  const int RECEIVE = 1;
  const int SEND    = 2;
  std::string host_str = "10.116.0.221";
  const char *hostname    = host_str.c_str();
  int portno        = 42024;
  struct hostent *server;
  int sockfd        = 0;

  int setup();
  int setup(int system_type_val);
  int send(imu_state_def_t *buff);


  int setup()
  {
    return setup(SEND);
  }

  int setup(int system_type_val)
  {

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        return 1;
    }

    /* gethostbyname: get the server's DNS entry */
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        return 1;
    }

    /* build the server's Internet address */
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serveraddr.sin_addr.s_addr, server->h_length);
    serveraddr.sin_port = htons(portno);


    serverlen = sizeof(serveraddr);

    if( system_type_val == RECEIVE )
    {
    /* print the server's reply */
    // Bind the socket with the server address 
      if ( bind(sockfd, (const struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0 )
      {
          return 1;
      }
     }

    return 0;
  }

  int send(imu_state_def_t *buff)
  {
    /* send the message to the server */
    int n = sendto(sockfd, buff, sizeof(*buff), 0, (struct sockaddr *) &serveraddr, serverlen);
    if (n < 0) return 1;

    return 0;
  }

}
