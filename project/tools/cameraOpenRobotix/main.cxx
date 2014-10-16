/*
 * image.c - Camera image demo
 *
 * This file is part of openrobotix-utils.
 *
 * Copyright(C) 2013
 * Stefan Herbrechtsmeier <sherbrec@cit-ec.uni-bielefeld.de>
 * http://opensource.cit-ec.de/projects/openrobotix-utils
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 * The development of this software was supported by the
 * Excellence Cluster EXC 277 Cognitive Interaction Technology.
 * The Excellence Cluster EXC 277 is a grant of the Deutsche
 * Forschungsgemeinschaft (DFG) in the context of the German
 * Excellence Initiative.
 *
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <libv4l2.h>

int main(int argc, char **argv)
{
	struct v4l2_format fmt;
	int rc, fd = -1;
	unsigned int i, length;
	char *dev_name = "/dev/video";
	char out_name[256];
	FILE *fout;
	void *buffer;

	fd = v4l2_open(dev_name, O_RDWR, 0);
	if (fd < 0) {
		fprintf(stderr, "Cannot open device %s\n", dev_name);
		exit(EXIT_FAILURE);
	}

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = 640;//320;
	fmt.fmt.pix.height      = 480;//240;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	fmt.fmt.pix.field       = V4L2_FIELD_ANY;
	rc = v4l2_ioctl(fd, VIDIOC_S_FMT, &fmt);
	
	printf("%d %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
	
	if (rc == -1) {
		fprintf(stderr, "Error: %d, %s\n", errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
		fprintf(stderr, "Error: Libv4l did not accept format\n");
		exit(EXIT_FAILURE);
	}
	if ((fmt.fmt.pix.width != 320) || (fmt.fmt.pix.height != 240))
		printf("Warning: driver is sending image at %dx%d\n",
			fmt.fmt.pix.width, fmt.fmt.pix.height);

	if (fmt.fmt.pix.sizeimage < (fmt.fmt.pix.width * fmt.fmt.pix.height)) {
		fprintf(stderr, "Error: Driver is sending image at %dx%d with size of %d\n",
			fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
		exit(EXIT_FAILURE);
	}

	buffer = malloc(fmt.fmt.pix.sizeimage);
	if (buffer == NULL) {
		fprintf(stderr, "Cannot allocate buffer\n");
		exit(EXIT_FAILURE);
	}  

	for (i = 0; i < 20; i++) {
//  int idx = 0;
// 	for ( ; ; ) {
		length = v4l2_read(fd, buffer, fmt.fmt.pix.sizeimage);
		if (length == -1) {
			fprintf(stderr, "Error: %d, %s\n", errno, strerror(errno));
			exit(EXIT_FAILURE);
		}
 		//printf("%d\n", idx++);
 		sprintf(out_name, "image%03d.ppm", i);
		fout = fopen(out_name, "w");
		if (!fout) {
			fprintf(stderr, "Cannot open image\n");
			exit(EXIT_FAILURE);
		}
		fprintf(fout, "P6\n%d %d 255\n",
			fmt.fmt.pix.width, fmt.fmt.pix.height);
		fwrite(buffer, fmt.fmt.pix.sizeimage, 1, fout);
		fclose(fout);
	}

	free(buffer);
	v4l2_close(fd);

	return 0;
}
