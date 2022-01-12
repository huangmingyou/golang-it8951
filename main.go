package main

// 0. 初始化it8915控制器
// 1. 上传图片
// 2. 墨水屏上显示图片

// huangmingyou@gmail.com
// 2022.01.04

/*
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <scsi/scsi.h>
#include <scsi/sg.h>
#include <byteswap.h>
#include <pthread.h>

#define MAX_TRANSFER 60800

typedef struct it8951_inquiry {
	unsigned char dontcare[8];
	unsigned char vendor_id[8];
	unsigned char product_id[16];
	unsigned char product_ver[4];
} IT8951_inquiry;

typedef struct it8951_deviceinfo {
	unsigned int uiStandardCmdNo;
	unsigned int uiExtendedCmdNo;
	unsigned int uiSignature;
	unsigned int uiVersion;
	unsigned int width;
	unsigned int height;
	unsigned int update_buffer_addr;
	unsigned int image_buffer_addr;
	unsigned int temperature_segment;
	unsigned int ui_mode;
	unsigned int frame_count[8];
	unsigned int buffer_count;
	unsigned int reserved[9];
	void *command_table;
} IT8951_deviceinfo;

typedef struct it8951_area {
	int address;
	int x;
	int y;
	int w;
	int h;
} IT8951_area;

typedef struct it8951_display_area {
	int address;
	int wavemode;
	int x;
	int y;
	int w;
	int h;
	int wait_ready;
} IT8951_display_area;

int debug = 0;
int clear = 0;
int devfd = 3;
int dismode = 2;
int gaddr = 0;
int
load_image_area(int fd, int addr, int x, int y, int w, int h,
	unsigned char *data)
{
	unsigned char load_image_cmd[16] = {
		0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xa2
	};

	IT8951_area area;
	memset(&area, 0, sizeof(IT8951_area));
	area.address = addr;
	area.x = __bswap_32(x);
	area.y = __bswap_32(y);
	area.w = __bswap_32(w);
	area.h = __bswap_32(h);

	int length = w * h;

	unsigned char *data_buffer = (unsigned char *) malloc(length + sizeof(IT8951_area));
	memcpy(data_buffer, &area, sizeof(IT8951_area));
	memcpy(&data_buffer[sizeof(IT8951_area)], data, length);

	sg_io_hdr_t io_hdr;

	memset(&io_hdr, 0, sizeof(sg_io_hdr_t));
	io_hdr.interface_id = 'S';
	io_hdr.cmd_len = 16;
	io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
	io_hdr.dxfer_len = length + sizeof(IT8951_area);
	io_hdr.dxferp = data_buffer;
	io_hdr.cmdp = load_image_cmd;
	io_hdr.timeout = 5000;

	if (ioctl(fd, SG_IO, &io_hdr) < 0) {
		perror("SG_IO image load failed");
	}
	free(data_buffer);
	return 0;
}

int
display_area(int fd, int addr, int x, int y, int w, int h, int mode)
{
	unsigned char display_image_cmd[16] = {
		0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x94
	};

	IT8951_display_area area;
	memset(&area, 0, sizeof(IT8951_display_area));
	area.address = addr;
	area.x = __bswap_32(x);
	area.y = __bswap_32(y);
	area.w = __bswap_32(w);
	area.h = __bswap_32(h);
	area.wait_ready = __bswap_32(1);
	area.wavemode = __bswap_32(mode);

	unsigned char *data_buffer = (unsigned char *) malloc(sizeof(IT8951_display_area));
	memcpy(data_buffer, &area, sizeof(IT8951_display_area));

	sg_io_hdr_t io_hdr;

	memset(&io_hdr, 0, sizeof(sg_io_hdr_t));
	io_hdr.interface_id = 'S';
	io_hdr.cmd_len = 16;
	io_hdr.dxfer_direction = SG_DXFER_TO_DEV;
	io_hdr.dxfer_len = sizeof(IT8951_display_area);
	io_hdr.dxferp = data_buffer;
	io_hdr.cmdp = display_image_cmd;
	io_hdr.timeout = 5000;

	if (ioctl(fd, SG_IO, &io_hdr) < 0) {
		perror("SG_IO display failed");
	}
	free(data_buffer);
	return 0;
}

void initscsi(const char *filename, int x, int y, int w, int h, int mode,const char *picfile) {
	dismode = mode;
	int fd, to, res;
	FILE *picfd;
	picfd = fopen(picfile,"r");
	if (picfd < 0) {
		perror("Could not open pic file");
		exit(EXIT_FAILURE);
	}
	fd = open(filename, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("Could not open scsi device");
		exit(EXIT_FAILURE);
	}

	devfd = fd;
	res = ioctl(fd, SCSI_IOCTL_GET_BUS_NUMBER, &to);
	if (res < 0) {
		fprintf(stderr, "%s is not a SCSI device\n", filename);
		exit(EXIT_FAILURE);
	}


	unsigned char inquiry_cmd[6] = {0x12, 0, 0, 0, 0, 0};
	unsigned char inquiry_result[96];
	unsigned char deviceinfo_cmd[12] = {
		0xfe, 0x00, // SCSI Customer command
		0x38, 0x39, 0x35, 0x31, // Chip signature
		0x80, 0x00, // Get System Info
		0x01, 0x00, 0x02, 0x00 // Version
	};
	unsigned char deviceinfo_result[112];


	sg_io_hdr_t io_hdr;

	memset(&io_hdr, 0, sizeof(sg_io_hdr_t));
	io_hdr.interface_id = 'S';
	io_hdr.cmd_len = 6;
	io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
	io_hdr.dxfer_len = 96;
	io_hdr.dxferp = inquiry_result;
	io_hdr.cmdp = inquiry_cmd;
	io_hdr.timeout = 1000;

	if (ioctl(fd, SG_IO, &io_hdr) < 0) {
		perror("SG_IO INQUIRY failed");
	}

	IT8951_inquiry *inquiry = (IT8951_inquiry *) inquiry_result;

	if (strncmp(inquiry->vendor_id, "Generic ", 8) != 0) {
		fprintf(stderr, "SCSI Vendor does not match\n");
		exit(EXIT_FAILURE);
	}
	if (strncmp(inquiry->product_id, "Storage RamDisc ", 8) != 0) {
		fprintf(stderr, "SCSI Product does not match\n");
		exit(EXIT_FAILURE);
	}
	if (strncmp(inquiry->product_ver, "1.00", 4) != 0) {
		fprintf(stderr, "SCSI Productver does not match\n");
		exit(EXIT_FAILURE);
	}

	if (debug == 1) {
		printf("Fetching device info\n");
	}

	memset(&io_hdr, 0, sizeof(sg_io_hdr_t));
	io_hdr.interface_id = 'S';
	io_hdr.cmd_len = sizeof(deviceinfo_cmd);
	io_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
	io_hdr.dxfer_len = 112;
	io_hdr.dxferp = deviceinfo_result;
	io_hdr.cmdp = deviceinfo_cmd;
	io_hdr.timeout = 10000;

	if (ioctl(fd, SG_IO, &io_hdr) < 0) {
		perror("SG_IO device info failed");
		exit(EXIT_FAILURE);
	}

	IT8951_deviceinfo *deviceinfo = (IT8951_deviceinfo *) deviceinfo_result;

	int width = __bswap_32(deviceinfo->width);
	int height = __bswap_32(deviceinfo->height);

	if (debug == 1) {
		printf("Found a %dx%d epaper display\n", width, height);
	}

	int addr = deviceinfo->image_buffer_addr;
	gaddr = addr;

}
void showpic(int x, int y, int w, int h, const char *picfile) {
	int clear = 0;
	int fd = devfd;
	// open pic file and load display it
	FILE *picfd;
	picfd = fopen(picfile,"r");
	if (picfd < 0) {
		perror("Could not open pic file");
		return;
	}
	int size = w * h;
	unsigned char *image = (unsigned char *) malloc(size);
	if (clear == 1) {
		memset(image, 0xff, size);
	} else {
		size_t total_left = size;
		unsigned char *buffer_pointer = image;
		while (total_left > 0) {
			size_t current = read(fileno(picfd), buffer_pointer, total_left);
			if (current < 0) {
				perror("stdin read");
				return;
			} else if (current == 0) {
				fprintf(stderr, "stdin input is truncated\n");
				return;
			} else {
				total_left -= current;
				buffer_pointer += current;
			}
		}
	}

	int offset = 0;
	int lines = MAX_TRANSFER / w;
	while (offset < size) {
		if ((offset / w) + lines > h) {
			lines = h - (offset / w);
		}
		if (debug == 1) {
			printf("Sending %dx%d chunk to %d,%d\n", w, lines, x, y + (offset / w));
		}
		load_image_area(fd, gaddr, x, y + (offset / w), w, lines, &image[offset]);
		offset += lines * w;
	}
	if (debug == 1) {
		printf("Starting refresh\n");
	}
	display_area(fd, gaddr, x, y, w, h, dismode);
	free(image);
}
*/
import "C"
import (
	"fmt"
	"os"
	"github.com/gin-gonic/gin"
	"net/http"
	"path/filepath"
	"strconv"
)

type CChar C.char

func (p *CChar) GoString() string {
	return C.GoString((*C.char)(p))
}

func PrintCString(cs *C.char) {
	C.puts(cs)
}

func init_it8915() {
	fp := "/home/pi/1.png"
        devf := os.Args[1]
	C.initscsi((*C.char)(C.CString(devf)), C.int(0), C.int(0), C.int(1872), C.int(1404), C.int(2), (*C.char)(C.CString(fp)))
	C.fflush(C.stdout)
}

func go_showpic(fn string,x,y,w,h int){
	C.showpic(C.int(x),C.int(y),C.int(w),C.int(h),(*C.char)(C.CString(fn)))
}

func showuploadpic(c *gin.Context) {
		x,_ := strconv.Atoi(c.PostForm("x"))
		y,_ := strconv.Atoi(c.PostForm("y"))
		w,_ := strconv.Atoi(c.PostForm("w"))
		h,_ := strconv.Atoi(c.PostForm("h"))

		// Source
		file, err := c.FormFile("file")
		if err != nil {
			c.String(http.StatusBadRequest, fmt.Sprintf("get form err: %s", err.Error()))
			return
		}

		filename := filepath.Base(file.Filename)
		if err := c.SaveUploadedFile(file, filename); err != nil {
			c.String(http.StatusBadRequest, fmt.Sprintf("upload file err: %s", err.Error()))
			return
		}

		c.String(http.StatusOK, fmt.Sprintf("ok"))
		// TODO:
		// 接收w,h,x,y参数
		fmt.Println(x,y,w,h)
		go_showpic(file.Filename,x,y,w,h)
}
func showlocalpic (c *gin.Context) {
		x,_ := strconv.Atoi(c.PostForm("x"))
		y,_ := strconv.Atoi(c.PostForm("y"))
		w,_ := strconv.Atoi(c.PostForm("w"))
		h,_ := strconv.Atoi(c.PostForm("h"))
		localfile := c.PostForm("localfile")

		fmt.Println(localfile,x,y,w,h)
		go_showpic(localfile,x,y,w,h)
}

func main() {
	init_it8915()
	r := gin.Default()
	r.POST("/showupload",showuploadpic)
	r.POST("/showlocal",showlocalpic)

	r.Run("0.0.0.0:9988")
}
