#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "utils.h"

static inline void get_cfg_string(char *dst, char *src)
{
	if (src == NULL)
		return;
	strcpy(dst, src);
}

int read_yuv420_file(const char *filename, char *addr0, char *addr1, uint32_t y_size)
{
	FILE *Fd = NULL;
	if (filename == NULL || addr0 == NULL || y_size == 0) {
		printf("ERR(%s):null param.\n", __func__);
		return -1;
	}

	Fd = fopen(filename, "r");
	char *buffer = NULL;

	if (Fd == NULL) {
		printf("ERR(%s):open(%s) fail\n", __func__, filename);
		return -1;
	}

	buffer = (char *)malloc(y_size + y_size / 2);
	if (fread(buffer, 1, y_size, Fd) != y_size) {
		printf("read bin(%s) to addr fail\n", filename);
		return -1;
	}

	if (fread(buffer + y_size, 1, y_size / 2, Fd) != y_size / 2) {
		printf("read bin(%s) to addr fail\n", filename);
		return -1;
	}

	memcpy(addr0, buffer, y_size);
	memcpy(addr1, buffer + y_size, y_size / 2);

	fflush(Fd);

	if (Fd)
		fclose(Fd);
	if (buffer) {
		free(buffer);
	}

	return 0;
}

int32_t load_file_2_buff(const char *path, char *filebuff, int32_t size)
{
	FILE *file = NULL;
	struct stat statbuf;
	uint32_t size_read = 0;

	file = fopen(path, "r");
	if (NULL == file) {
		printf("file %s open failed\n", path);
		return -1;
	}
	stat(path, &statbuf);
	if (0 == statbuf.st_size) {
		printf("read file size is zero:%s, %ld, %d\n", path, statbuf.st_size, size);
		fclose(file);
		return -1;
	}
	size_read = fread(filebuff, 1, size, file);
	fclose(file);
	return size_read;
}


int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1, unsigned int size, unsigned int size1)
{
	FILE *yuvFd = NULL;
	char *buffer = NULL;

	yuvFd = fopen(filename, "w+");

	if (yuvFd == NULL) {
		printf("open(%s) fail", filename);
		return -1;
	}

	buffer = (char *)malloc(size + size1);

	if (buffer == NULL) {
		printf("ERR:malloc file");
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);
	memcpy(buffer + size, srcBuf1, size1);

	fflush(stdout);
	fwrite(buffer, 1, size + size1, yuvFd);
	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	return 0;
}


