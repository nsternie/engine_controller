/*
 * flash.h
 *
 *  Created on: Jan 10, 2018
 *      Author: Nick Sterenberg
 */

#ifndef FLASH_H_
#define FLASH_H_


#define FLASH_COMMAND_RESET         0xff
#define FLASH_COMMAND_WRITE_ENABLE  0x06
#define FLASH_COMMAND_WRITE_DISABLE 0x04
#define FLASH_COMMAND_PAGE_READ     0x13
#define FLASH_COMMAND_READ_DATA     0x03
#define FLASH_COMMAND_BLOCK_ERASE   0xd8
#define FLASH_COMMAND_LOAD_DATA     0x02
#define FLASH_COMMAND_LOAD_RANDOM_DATA  0x84
#define FLASH_COMMAND_PROGRAM_EXECUTE   0x10
#define FLASH_COMMAND_WRITE_STATUS_REGISTER 0x1f
#define FLASH_COMMAND_READ_STATUS_REGISTER  0x0f


#define PACKET_TYPE_STRING            2
#define PACKET_TYPE_GYRO              3
#define PACKET_TYPE_ACCEL             4
#define PACKET_TYPE_BARO              5
#define PACKET_TYPE_MILLIS            6
#define PACKET_TYPE_GPS               7
#define PACKET_TYPE_EOP               255

#define PACKET_LENGTH_MILLIS            4
#define PACKET_LENGTH_MICROS            4
#define PACKET_LENGTH_GYRO              7
#define PACKET_LENGTH_ACCEL             9
#define PACKET_LENGTH_BARO              4


typedef struct filestruct{
  uint16_t start_page;
  uint16_t stop_page;
  uint16_t current_page;
  uint16_t bytes_free;
  uint16_t file_number;
} file;

#define MAX_FILES 16
typedef struct filesystemstruct{
  uint32_t num_files;
  int32_t current_file;
  uint16_t next_file_page;
  file files[MAX_FILES];
} filesystem;

void read_filesystem(filesystem* f);
void write_filesystem(filesystem* f);
//void print_file(uint32_t filenum);
void print_file_raw(uint32_t filenum);

int read_flash_id();   // Returns 0 if ok, 1 for error
void flash_command(uint8_t command);

void unlock_all();

void read_flash_status();


uint16_t load_page(uint16_t page_number);
void read_buffer(uint16_t column, uint8_t *buffer, uint16_t size);
void write_buffer(uint16_t column, uint8_t *page_buffer, uint16_t size);
void program_page(uint16_t page_number);

void erase_block(uint16_t block_number);
uint8_t flash_read_status_register(uint8_t reg);

uint8_t flash_test();   // This WILL corrupt any data on the disk

// TODO: Functions
file *new_log();  // return log stuct
uint32_t log_data(file* f, uint8_t *data, uint32_t length);

uint32_t close_log(file *log);
uint8_t flash_busy(); // check the busy bit

void log_time(file* f, uint32_t time);
void log_string(file* f, char* str);
//void log_gps(file* f, gps* g);

#endif /* FLASH_H_ */
