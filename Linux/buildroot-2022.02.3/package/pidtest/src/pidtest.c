#define _GNU_SOURCE /* Allow use of pthread_tryjoin_np */

#include <arpa/inet.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <sched.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <unistd.h>


#include "gyro.h"
#include "madgwick.h"
#include "pwm.h"

static const int ADAPTER_NUMBER = 1;
static const int RT_THREAD_STACK_SIZE = PTHREAD_STACK_MIN * 4;

struct rt_transfer {
	struct vec3 dir;
	double elapsed;
	int throttle;
};

struct rt_init {
	sem_t* kill_sig;
	pthread_mutex_t* trans_mutex;
	struct rt_transfer* transfer;
};

pthread_t create_rt_thread(void*(*)(void*), struct rt_init*);
void* rt(void*);
int get_pid(struct vec3, double, double, double, double);

int main(void) {
	int res, pulse, pwm;
	pthread_t rt_thread;
	struct rt_init init;
	sem_t kill_sig;
	pthread_mutex_t trans_mutex;
	struct rt_transfer transfer;

	int socket_desc, client_sock, client_size;
	struct sockaddr_in server_addr, client_addr;
	char server_message[128];
	
	printf("Quadcopter Hardware Test Program v0.0...\r\n");
	
	sem_init(&kill_sig, 0, 0);
	pthread_mutex_init(&trans_mutex, NULL);

	init.kill_sig = &kill_sig;
	init.trans_mutex = &trans_mutex;
	init.transfer = &transfer;

	rt_thread = create_rt_thread(rt, &init);

	res = pthread_tryjoin_np(rt_thread, NULL); /* was pthread_tryjoin_np */
	
	/*
	 * BEGINNING OF SOCKET CODE
	 */

	memset(server_message, '\0', sizeof(server_message));

	socket_desc = socket(AF_INET, SOCK_STREAM, 0);

	if(socket_desc < 0){
		printf("Error while creating socket\r\n");
		goto out;
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(2000);
	server_addr.sin_addr.s_addr = INADDR_ANY;

	if(bind(socket_desc, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
		printf("Couldn't bind to the port\r\n");
		goto out;
	}

	if(listen(socket_desc, 1) < 0) {
		printf("Error while listening\r\n");
		goto out;
	}

	printf("Listening for incoming connections.....\r\n");

	client_size = sizeof(client_addr);
	client_sock = accept(socket_desc, (struct sockaddr*)&client_addr, &client_size);

	printf("Successfully accepted a client with throttle data!\r\n");

	while(1) {
		//printf("Sending data over the network\r\n");
		if(pthread_mutex_lock(&trans_mutex) == 0) {
			sprintf(server_message, 
				"{ \"type\": \"heading\", \"x\": %f, \"y\": %f, \"z\": %f, \"throttle\": %d, \"elapsed\": %f }\0",
				transfer.dir.x, transfer.dir.y, transfer.dir.z, transfer.throttle, transfer.elapsed);
			pthread_mutex_unlock(&trans_mutex);
		}

		if(send(client_sock, server_message, strlen(server_message), 0) < 0) {
			printf("Can't send\r\n");
			goto out;
		}

		usleep(5000);
	}


	/*
	 * END OF SOCKET CODE
	 */

out:
	close(client_sock);
	close(socket_desc);

	printf("Signaling the real time environment to kill itself\r\n");
	res = sem_post(&kill_sig);

	if(res != 0) {
		printf("Failed to signal the kill semaphore, terminating through force\r\n");
		pthread_cancel(rt_thread);
	}

	sem_destroy(&kill_sig);
	
	sleep(1);

	printf("Successfully tested the hardware!\r\n");
}

void* rt(void* args) {
	int gyro, mag, pwm, num, pid, base_throttle, throttle;
	char input[15];
	double elapsed, kp, ki, kd;
	struct gyro_state g_state;
	struct vec3 m_state, dir;
	struct timeval st, et;
	struct rt_init* init;

	printf("Entering the real time environment...\r\n");

	init = (struct rt_init*)args;

	gyro = setup_gyro(ADAPTER_NUMBER);
	mag = setup_mag(ADAPTER_NUMBER);
	pwm = setup_pwm(ADAPTER_NUMBER);
	
	printf("Setting PWM frequency\r\n");
	set_pwm_frequency(pwm, 50);
	set_pwm(pwm, 0, 0, 0);
	
	printf("Type \"ARM\" in all capital letters when ready to arm the system: ");
	scanf("%12[^\n]s", input);
	if(strcmp(input,"ARM") != 0) {
		printf("Invalid arm key, not arming!\r\n");
		exit(1);
	}

	printf("System is armed!\r\n");
	set_pwm(pwm, 0, 0, 205);

	scanf("%c", input); /* Clear buffer of invalid /n character */

	printf("Enter kP value: ");
	scanf("%lf", &kp);
	printf("Enter kI value: ");
	scanf("%lf", &ki);
	printf("Enter kD value: ");
	scanf("%lf", &kd);
	printf("Enter base throttle value: ");
	scanf("%d", &base_throttle);

	printf("PID is set to kP: %f kI: %f kD: %f Base throttle: %d\r\n", kp, ki, kd, base_throttle);

	scanf("%c", input); /* Clear buffer of invalid /n character */

	printf("Type \"THROTTLE UP\" in all capital letters when ready to start PID control of the system: ");
	scanf("%12[^\n]s", input);
	if(strcmp(input,"THROTTLE UP") != 0) {
		printf("Invalid throttle key, not continuing!\r\n");
		exit(1);
	}
	
	gettimeofday(&st, NULL);

	while(sem_trywait(init->kill_sig) != 0) {
		g_state = get_gyro_state(gyro);
		m_state = get_mag_state(mag);

		gettimeofday(&et, NULL);

		elapsed = (et.tv_sec - st.tv_sec) + ((et.tv_usec - st.tv_usec) / 1000000.0f);
		dir = get_angle(g_state.w, g_state.a, m_state, elapsed);
		pid = get_pid(dir, kp, ki, kd, elapsed);
		throttle = base_throttle + pid;
		set_pwm_us(pwm, 0, throttle);
		
		gettimeofday(&st, NULL);

		/* 
		 * Make sure that nothing else is reading from the memory for some reason.
		 * Read the sensor again if something is blocking in order to not waste time 
		 * this also prevents a deadlock because this thread will always take priority
		 * and will spin lock forever because the system won't relenquish control to
		 * the lower priority threads.
		 */
		if(pthread_mutex_trylock(init->trans_mutex) == 0) {
			// printf("Transmitting the data to the main thread over shared memory\r\n");
			init->transfer->dir = dir;
			init->transfer->elapsed = elapsed;
			init->transfer->throttle = throttle;
			pthread_mutex_unlock(init->trans_mutex);
		}

		usleep(100); // Relinquish control to the main thread for a bit
	}

	set_pwm(pwm, 0, 0, 0);

	printf("Exiting the real time environment\r\n");
}

/*
 * Based on tutorial here: https://electronoobs.com/eng_robotica_tut6_2.php
 */
int get_pid(struct vec3 dir, double kp, double ki, double kd, double elapsed) {
	double error, p, i, d;
	int pid;

	static double prev_error = 0;

	const static double target_angle = 0;

	error = dir.z - target_angle;
	    
	p = kp * error;

	if(-3 < error < 3) {
		  i = i + (ki * error);  
	}

	d = kd * ((error - prev_error) / elapsed);

	pid = p + i + d;

	// printf("p %f, i %f, d %f, pid %d, error %f, prev_error %f\r\n", p, i, d, pid, error, prev_error);

	if(pid < -1000) {
		  pid = -1000;
	}

	if(pid > 1000) {
		  pid = 1000;
	}

	prev_error = error;

	return pid;
}


pthread_t create_rt_thread(void*(*fun)(void*), struct rt_init* init) {
	/*
	 * Create a thread with the correct settings for operating under PREEMPT RT.
	 *
	 * Code modified from Linux Foundation documentation:
	 * https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base
	 */

	struct sched_param s_param;
	pthread_attr_t t_attr;
	pthread_t rt_thread;
	int ret;

	ret = mlockall(MCL_CURRENT | MCL_FUTURE); /* Lock all process memory */
	if (ret != 0) {
		printf("Locking the process memory failed\r\n");
		exit(1);
	}
	
	ret = pthread_attr_init(&t_attr); /* Initialize the pthread attributes to their default */
	if (ret != 0) {
		printf("Initializing the pthread attributes failed\r\n");
		exit(1);
	}

	printf("The size of the stack is %d\r\n", RT_THREAD_STACK_SIZE);

	ret = pthread_attr_setstacksize(&t_attr, RT_THREAD_STACK_SIZE); /* Set a specific stack size for the thread */
	if (ret != 0) {
		printf("Setting the thread stack size failed\r\n");
		exit(1);
	}

	ret = pthread_attr_setschedpolicy(&t_attr, SCHED_FIFO); /* Set the first in first out scheduler policy */
	if (ret != 0) {
		printf("Setting the scheduler policy failed\r\n");
		exit(1);
	}

	s_param.sched_priority = 90; /* Set a very high priority so that the process will run in real time */

	ret = pthread_attr_setschedparam(&t_attr, &s_param); /* Set the scheduler parameters for the thread */
	if (ret != 0) {
		printf("Setting the scheduler parameters failed failed\r\n");
		exit(1);
	}
	
	ret = pthread_attr_setinheritsched(&t_attr, PTHREAD_EXPLICIT_SCHED); /* Force use of the scheduler parameters */
	if (ret != 0) {
		printf("Setting the scheduler parameters failed failed\r\n");
		exit(1);
	}

	ret = pthread_create(&rt_thread, &t_attr, fun, (void*)init);
	if (ret != 0) {
		printf("Creating the pthread failed\r\n");
	}

	return rt_thread;
}

