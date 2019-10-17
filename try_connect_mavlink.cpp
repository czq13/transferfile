#include <iostream>
#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <poll.h>
#include "mavlink/v2.0/common/mavlink.h"
using namespace std;

struct sockaddr_in _myaddr;
struct sockaddr_in _srcaddr;
unsigned int _addrlen;
unsigned char _buf[1024];
int _fd;

long hrt_system_time() {
	long ret;
	struct timeval tv;
	gettimeofday(&tv,NULL);
	ret = tv.tv_sec * 1000000;
	ret += tv.tv_usec;
	return ret;
}

void send_mavlink_message(const mavlink_message_t &aMsg)
{
	uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t bufLen = 0;

	// convery mavlink message to raw data
	bufLen = mavlink_msg_to_send_buffer(buf, &aMsg);

	// send data
	ssize_t len = sendto(_fd, buf, bufLen, 0, (struct sockaddr *)&_srcaddr, _addrlen);

	if (len <= 0) {
		printf("Failed sending mavlink message\n");
	}
}

void handle_message(mavlink_message_t *msg, bool publish)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		printf("MAVLINK_MSG_ID_HIL_SENSOR received!\n");
		break;

	case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		printf("MAVLINK_MSG_ID_HIL_OPTICAL_FLOW received!\n");
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		printf("MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE received!\n");
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		printf("MAVLINK_MSG_ID_DISTANCE_SENSOR received!\n");
		break;

	case MAVLINK_MSG_ID_HIL_GPS:
		printf("MAVLINK_MSG_ID_HIL_GPS received!\n");
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS:
		printf("MAVLINK_MSG_ID_RC_CHANNELS received!\n");
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		printf("MAVLINK_MSG_ID_LANDING_TARGET received!\n");

		break;

	case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		printf("MAVLINK_MSG_ID_HIL_STATE_QUATERNION received!\n");

		break;
	}

}

int main(int argc, char **argv) {
	struct sockaddr_in _myaddr;
	struct sockaddr_in _srcaddr;
	unsigned int _addrlen;

	int udp_port = 14560;

	// try to setup udp socket for communcation with simulator
	memset((char *)&_myaddr, 0, sizeof(_myaddr));
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(udp_port);

	if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("create socket failed\n");
		return -1;
	}

	if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		printf("bind failed\n");
		return -1;
	}

	struct pollfd fds[2];
	memset(fds, 0, sizeof(fds));
	unsigned fd_count = 1;
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint64_t pstart_time = 0;

	bool no_sim_data = true;

	while (no_sim_data) {
		int pret = ::poll(&fds[0], fd_count, 100);

		if (fds[0].revents & POLLIN) {
			if (pstart_time == 0) {
				pstart_time = hrt_system_time();
			}

			long len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
			// send hearbeat
			mavlink_heartbeat_t hb = {};
			mavlink_message_t message = {};
			hb.autopilot = 12;
			hb.base_mode |= (1) ? 128 : 0;
			mavlink_msg_heartbeat_encode(0, 50, &message, &hb);
			send_mavlink_message(message);

			if (len > 0) {
				mavlink_message_t msg;
				mavlink_status_t udp_status = {};

				for (int i = 0; i < len; i++) {
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &udp_status)) {
						// have a message, handle it
						handle_message(&msg, true);

						if (msg.msgid != 0 && (hrt_system_time() - pstart_time > 1000000)) {
							printf("Got initial simulation data, running sim..");
							no_sim_data = false;
						}
					}
				}
			}
		}
	}
	return 0;
}
