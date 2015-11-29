/**
\file
\brief Реализация функций обмена данными с компасом

В данном файле находятся все фукции для обмена данными с компасом,
включая функции ожидания ответа от датчика с таймаутом и прочее.

\ingroup compass_node

*/

#include "compass/packet_handler.h"

#define CONSIST(part, all, mask) (((part ^ all) & mask)  == 0)
#define CALLBACK_QUEUE_SIZE 10
#define ACK_QUEUE_SIZE		10

static uint8_t debug_level = 0;

uint8_t message_buffer[BUFF_SIZE];
ACK_QUEUE_t ack_queue[ACK_QUEUE_SIZE];

CALLBACK_t callback_queue[CALLBACK_QUEUE_SIZE];

void handle_message(uint8_t* buffer, int len);

void parse_frame(inemo_frame_struct new_frame);

void shift_data(uint8_t* buffer, int len, int shift);

static void wait(int delay_msec);

void packet_handler_init()
{
	memset((void*)ack_queue, 0, sizeof(ACK_QUEUE_t) * ACK_QUEUE_SIZE);
	memset((void*)callback_queue, 0, sizeof(CALLBACK_t) * CALLBACK_QUEUE_SIZE);
}

void* data_handler(void* arg)
{
	int len;
	uint8_t buffer[BUFF_SIZE];

	puts("Second thread start reading...\n");
	while(true) {
    	len = usb_read_data(buffer);
		//len = get_message(buffer, BUFF_SIZE);
		//pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
		if(len >= 0) {
			handle_message(buffer, len);
		}

		//pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
		//pthread_testcancel(); //cancel point
		wait(1);
  }
}

void set_packet_debug_level(uint8_t level)
{
	debug_level = level;
}

int data_end = 0;
void handle_message(uint8_t* buffer, int len)
{
	memcpy((void*)(message_buffer + data_end), (void*)buffer, sizeof(char) * len);
	data_end += len;
  	if(debug_level)
    	printf("Recieved %d bytes from compass\n", len);

	if(data_end < 2)
		return;

	inemo_frame_struct new_frame;
	new_frame = *(inemo_frame_struct*)&message_buffer;

	if(new_frame.length > (data_end - 2)) {
		return;
	}

	if(debug_level)
		parse_frame(new_frame);

	if(CONSIST(TYPE_DATA, new_frame.frame_control, FRAME_TYPE_MASK))
		handle_data(new_frame);
	else if(CONSIST(TYPE_ACK, new_frame.frame_control, FRAME_TYPE_MASK))
		handle_ack(new_frame, true);
	else if(CONSIST(TYPE_NACK, new_frame.frame_control, FRAME_TYPE_MASK))
		handle_ack(new_frame, false);

	shift_data(message_buffer, data_end, (new_frame.length + 2));
	data_end -= (new_frame.length + 2);
	if(data_end > (BUFF_SIZE - 100))
		printf("ERROR: message buffer is almost full: %d bytes last!\n", BUFF_SIZE - data_end);
}

void parse_frame(inemo_frame_struct new_frame)
{
	printf("frame ID: %d\n", new_frame.id);
	printf("frame len: %d\n", new_frame.length);
	printf("frame frame_control (%d): ", new_frame.frame_control);

	if(CONSIST(NORMAL_PRIORITY, new_frame.frame_control, QOS_MASK))
		printf("NORMAL_PRIORITY |");
	if(CONSIST(MEDIUM_PRIORITY, new_frame.frame_control, QOS_MASK))
		printf("MEDIUM_PRIORITY |");
	if(CONSIST(HIGH_PRIORITY, new_frame.frame_control, QOS_MASK))
		printf("HIGH_PRIORITY |");
	if(CONSIST(RFU_PRIORITY, new_frame.frame_control, QOS_MASK))
		printf("RFU_PRIORITY |");
	if(CONSIST(VER_1, new_frame.frame_control, FRAME_VERSION_MASK))
		printf("VER_1 |");
	if(CONSIST(LM, new_frame.frame_control, LMLF_MASK))
		printf("LM |");
	if(CONSIST(LF, new_frame.frame_control, LMLF_MASK))
		printf("LF |");
	if(CONSIST(NEED_ACK, new_frame.frame_control, ASK_MASK))
		printf("NEED_ACK |");
	if(CONSIST(NO_ACK, new_frame.frame_control, ASK_MASK))
		printf("NO_ACK |");
	if(CONSIST(TYPE_CONTROL, new_frame.frame_control, FRAME_TYPE_MASK))
		printf("TYPE_CONTROL |");
	if(CONSIST(TYPE_DATA, new_frame.frame_control, FRAME_TYPE_MASK))
		printf("TYPE_DATA |");
	if(CONSIST(TYPE_ACK, new_frame.frame_control, FRAME_TYPE_MASK))
		printf("TYPE_ACK |");
	if(CONSIST(TYPE_NACK, new_frame.frame_control, FRAME_TYPE_MASK))
		printf("TYPE_NACK |");
	printf("\n");

}

void handle_data(inemo_frame_struct frame)
{
	for(uint8_t i = 0; i < CALLBACK_QUEUE_SIZE; i++) {
		if(frame.id == callback_queue[i].id && callback_queue[i].status) {
			callback_queue[i].callback(frame);
		}
	}
}

bool registrate_data_callback(int msg_id, void (*callback)(inemo_frame_struct))
{
	for(uint8_t i = 0; i < CALLBACK_QUEUE_SIZE; i++) {
		if (callback_queue[i].status == 0) {
			if(debug_level)
				printf("Registered new callback for msg ID: %d\n", msg_id);
			callback_queue[i].status = 1;
			callback_queue[i].id = msg_id;
			callback_queue[i].callback = callback;
			return 0;
		}
	}
	return 1;
}

bool deregistrate_data_callback(int msg_id)
{
	for(uint8_t i = 0; i < CALLBACK_QUEUE_SIZE; i++) {
		if (callback_queue[i].id == msg_id) {
			if(debug_level)
				printf("Deregistered callback for msg ID: %d\n", msg_id);
			callback_queue[i].status = 0;
			return 0;
		}
	}
	return 1;
}

bool INEMO_send_command(inemo_frame_struct frame)
{
	int packet_size;
	uint8_t transmitt_buffer[TX_BUFF_SIZE];
	memset(transmitt_buffer, 0, TX_BUFF_SIZE);

	transmitt_buffer[0] = frame.frame_control;
	transmitt_buffer[1] = frame.length;
	transmitt_buffer[2] = frame.id;
	for(uint8_t i = 0; i < frame.length - sizeof(frame.id); i++) {
		transmitt_buffer[i + 3] = frame.payload[i];
	}

	packet_size = frame.length + sizeof(frame.length) + sizeof(frame.frame_control);

	if(CONSIST(NEED_ACK, frame.frame_control, ASK_MASK)) {
		bool status = insert_ack_queue(frame.id);
		if(status != 0)
			printf("Ack_queue is full.\n");
	}

	if(debug_level) {
		printf("command with ID %d was sending. Length of packet: %d bytes\n", 
			frame.id, packet_size);
	}
	usb_send_data(transmitt_buffer, packet_size);
	//send_to_com(transmitt_buffer, packet_size);

	return true;
}

char insert_ack_queue(uint8_t id)
{

	if(debug_level) {
		printf("ACK_QUEUE: ");
		for(int i = 0; i < ACK_QUEUE_SIZE; i++)
			printf("%d, ", ack_queue[i].status);
		printf("\n");
	}

	for(int i = 0; i < ACK_QUEUE_SIZE; i++)
	{
		if(ack_queue[i].status == EMPTY) {
			ack_queue[i].status = WAIT_ANSWER;
			ack_queue[i].id = id;
			ack_queue[i].error = 0;
			return 0;
		}
	}
	return 1;
}

uint8_t wait_answer(uint8_t id, int timeout)
{
	double start_time = get_fixate_time();
  
	uint8_t cur_index = 0;
	for(int i = 0; i < ACK_QUEUE_SIZE; i++) {
		if(ack_queue[i].id == id && ack_queue[i].status == WAIT_ANSWER)
			cur_index = i;
	}

	float timeout_ms = timeout / 1000;
	while(start_time + timeout_ms > get_fixate_time()) {
		switch(ack_queue[cur_index].status) {
			case WAIT_ANSWER:   
				wait(10);
				break;
			case ANSWER_OK:
				ack_queue[cur_index].status = EMPTY;
				if(debug_level) 
					printf("Answer with ID %d, recieved: OK.\n", id);
				return 0;
			case ANSWER_ERROR:
				if(debug_level)
					printf("Answer with ID %d, recieved: FALSE.\n", id);
				ack_queue[cur_index].status = EMPTY;
				return 1;
		}
	}

	if(debug_level)
		printf("Waiting answer with ID %d ended with timeout.\n", id);
	return 1;
}

void handle_ack(inemo_frame_struct frame, int ack_status)
{
	for(int i = 0; i < ACK_QUEUE_SIZE; i++)
	{
		if(ack_queue[i].status == WAIT_ANSWER && frame.id == ack_queue[i].id) {
			ack_queue[i].status = ack_status ? ANSWER_OK : ANSWER_ERROR;
		}
	}

	for(uint8_t i = 0; i < CALLBACK_QUEUE_SIZE; i++) {
		if(frame.id == callback_queue[i].id && callback_queue[i].status) {
			callback_queue[i].callback(frame);
		}
	}
}

// Есть в unils. Здесь находится специально для переносимости
// Возвращает текущее время c миллисекундной точностью в формате double
double get_fixate_time()
{
	struct timeb timebuf;
	ftime(&timebuf);
	return ( (double)timebuf.time + (double)timebuf.millitm / 1000 );
}

void shift_data(uint8_t* buffer, int len, int shift)
{
	for(int i = 0; i < (len - shift); i++) {
		buffer[i] = buffer[i + shift];
	}
	for(int i = len - shift; i < len; i++) {
		buffer[i] = 0;
	}
}

static void wait(int delay_msec)
{
	int delay_usec = delay_msec * 1000;
	usleep (delay_usec);
}